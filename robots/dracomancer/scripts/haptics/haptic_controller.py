#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Mk-II contact and safety haptic controller."""

import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray

from contact_feedback import ContactFeedback
from safety_feedback import SafetyFeedback, combine_torque, rate_limit
from servo_output import XM430CurrentOutput


class HapticController:
    def __init__(self):
        rospy.init_node("haptic_controller")

        self.human_joints = rospy.get_param("~human_joint_names")
        self.robot_joints = rospy.get_param("~robot_joint_names")
        self.human_count = len(self.human_joints)
        self.robot_count = len(self.robot_joints)
        self.rate_hz = float(rospy.get_param("~rate", 100.0))

        self.enable_contact = self.parse_bool(
            rospy.get_param("~enable_contact_feedback", False))
        self.enable_safety = self.parse_bool(
            rospy.get_param("~enable_safety_feedback", True))
        self.contact_timeout = float(rospy.get_param("~contact_timeout", 0.1))
        self.safety_timeout = float(rospy.get_param("~safety_timeout", 0.25))
        self.joint_timeout = float(rospy.get_param("~joint_timeout", 0.25))
        self.deadband = float(rospy.get_param("~torque_deadband", 0.01))
        self.torque_limits = self.vector_param("~torque_limits", self.human_count, 0.2)
        self.torque_rate_limits = self.vector_param(
            "~torque_rate_limits", self.human_count, 1.0)

        robot_stiffness = self.vector_param(
            "~safety_stiffness", self.robot_count, 0.2)
        human_damping = self.vector_param(
            "~human_damping", self.human_count, 0.01)
        self.contact_model = ContactFeedback(
            self.robot_count, self.human_count,
            rospy.get_param("~contact_gain", 1.0))
        self.safety_model = SafetyFeedback(
            robot_stiffness, human_damping,
            rospy.get_param("~safety_gain", 1.0))

        self.mapping_jacobian = self.matrix_param(
            "~mapping_jacobian", self.robot_count, self.human_count, required=True)
        self.robot_jacobian = self.matrix_param(
            "~robot_jacobian", 6, self.robot_count, required=False)

        self.teleop_mode = self.parse_bool(rospy.get_param("~teleop_mode", False))
        self.shape_error = np.zeros(self.robot_count)
        self.human_velocity = np.zeros(self.human_count)
        self.contact_wrench = np.zeros(6)
        self.external_safety = np.zeros(self.human_count)
        self.last_shape_error = rospy.Time(0)
        self.last_joint_state = rospy.Time(0)
        self.last_contact = rospy.Time(0)
        self.last_external_safety = rospy.Time(0)
        self.previous_positions = None
        self.previous_position_time = None
        self.previous_torque = np.zeros(self.human_count)
        self.previous_update = rospy.Time.now()

        ns = rospy.get_param("~device_ns", "/dracomancer").rstrip("/")
        robot_ns = rospy.get_param("~robot_ns", "/dragon").rstrip("/")
        self.total_pub = rospy.Publisher(
            rospy.get_param("~haptic_torque_topic", ns + "/haptic_torque"),
            JointState, queue_size=1)
        self.contact_pub = rospy.Publisher(
            rospy.get_param("~contact_torque_topic", ns + "/haptic/contact_torque"),
            JointState, queue_size=1)
        self.safety_pub = rospy.Publisher(
            rospy.get_param("~safety_torque_topic", ns + "/haptic/safety_torque"),
            JointState, queue_size=1)
        self.active_pub = rospy.Publisher(
            rospy.get_param("~haptic_active_topic", ns + "/haptic/active"),
            Bool, queue_size=1)

        self.servo_output = XM430CurrentOutput(
            self.human_joints,
            rospy.get_param("~servo_ids", list(range(self.human_count))),
            rospy.get_param("~torque_constants_nm_per_amp", [1.783] * self.human_count),
            self.torque_limits,
            rospy.get_param("~actuator_signs", [1.0] * self.human_count),
            rospy.get_param("~current_topic", ns + "/servo/target_current"),
            rospy.get_param("~torque_enable_topic", ns + "/servo/torque_enable"),
            self.parse_bool(rospy.get_param("~enable_hardware_output", False)),
            rospy.get_param("~current_unit_amp", 0.00269),
            rospy.get_param("~current_limits_raw", []))

        rospy.Subscriber(rospy.get_param("~device_joint_topic", ns + "/joint_states"),
                         JointState, self.joint_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~shape_error_topic", ns + "/shape_control_error"),
                         Float64MultiArray, self.shape_error_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~mode_topic", ns + "/teleop_mode"),
                         Bool, self.mode_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param(
            "~contact_wrench_topic", robot_ns + "/estimated_external_wrench"),
            WrenchStamped, self.wrench_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param(
            "~mapping_jacobian_topic", ns + "/haptic/mapping_jacobian"),
            Float64MultiArray, self.mapping_jacobian_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param(
            "~robot_jacobian_topic", ns + "/haptic/robot_jacobian"),
            Float64MultiArray, self.robot_jacobian_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param(
            "~external_safety_torque_topic", ns + "/haptic/safety_torque_input"),
            JointState, self.external_safety_cb, queue_size=1)

        if self.enable_contact and self.robot_jacobian is None:
            rospy.logwarn("contact haptics waits for a 6x%d robot Jacobian on %s/haptic/robot_jacobian",
                          self.robot_count, ns)
        if self.servo_output.hardware_enabled:
            rospy.logwarn("XM430 hardware haptics enabled: verify Operating Mode=0 and Current Limit before use")

    @staticmethod
    def parse_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        text = str(value).strip().lower()
        if text == "true":
            return True
        if text == "false":
            return False
        raise ValueError("boolean parameter must be true or false: %s" % value)

    def vector_param(self, name, size, default):
        data = np.asarray(
            rospy.get_param(name, [default] * size), dtype=float)
        if data.shape != (size,) or not np.all(np.isfinite(data)):
            raise ValueError("%s must contain %d finite values" % (name, size))
        return data

    @staticmethod
    def reshape_matrix(values, rows, cols):
        data = np.asarray(values, dtype=float)
        if data.size != rows * cols:
            raise ValueError("matrix requires %d values, got %d" % (rows * cols, data.size))
        if not np.all(np.isfinite(data)):
            raise ValueError("matrix contains non-finite values")
        return data.reshape((rows, cols))

    def matrix_param(self, name, rows, cols, required):
        values = rospy.get_param(name, [])
        if not values and not required:
            return None
        return self.reshape_matrix(values, rows, cols)

    @staticmethod
    def fresh(stamp, timeout):
        return stamp != rospy.Time(0) and (rospy.Time.now() - stamp).to_sec() <= timeout

    def mode_cb(self, msg):
        self.teleop_mode = bool(msg.data)

    def shape_error_cb(self, msg):
        values = np.asarray(msg.data, dtype=float)
        if values.shape != (self.robot_count,) or not np.all(np.isfinite(values)):
            rospy.logwarn_throttle(
                2.0, "shape error must contain %d finite values", self.robot_count)
            return
        self.shape_error = values
        self.last_shape_error = rospy.Time.now()

    def joint_cb(self, msg):
        now = rospy.Time.now()
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        current = np.asarray([positions.get(name, np.nan) for name in self.human_joints])
        if all(name in velocities for name in self.human_joints):
            self.human_velocity = np.asarray(
                [velocities[name] for name in self.human_joints], dtype=float)
        elif (self.previous_positions is not None and
              self.previous_position_time is not None and
              np.all(np.isfinite(current))):
            dt = (now - self.previous_position_time).to_sec()
            if dt > 1e-6:
                self.human_velocity = (current - self.previous_positions) / dt
        if np.all(np.isfinite(current)):
            self.previous_positions = current
            self.previous_position_time = now
            self.last_joint_state = now

    def wrench_cb(self, msg):
        self.contact_wrench = np.asarray([
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
        ], dtype=float)
        self.last_contact = rospy.Time.now()

    def mapping_jacobian_cb(self, msg):
        try:
            self.mapping_jacobian = self.reshape_matrix(
                msg.data, self.robot_count, self.human_count)
        except ValueError as error:
            self.mapping_jacobian = None
            rospy.logwarn_throttle(2.0, "invalid mapping Jacobian: %s", error)

    def robot_jacobian_cb(self, msg):
        try:
            self.robot_jacobian = self.reshape_matrix(msg.data, 6, self.robot_count)
        except ValueError as error:
            self.robot_jacobian = None
            rospy.logwarn_throttle(2.0, "invalid robot Jacobian: %s", error)

    def external_safety_cb(self, msg):
        if len(msg.name) != len(msg.effort):
            rospy.logwarn_throttle(
                2.0, "external safety torque name/effort sizes differ")
            return
        values = dict(zip(msg.name, msg.effort))
        torque = np.asarray(
            [values.get(name, 0.0) for name in self.human_joints], dtype=float)
        if not any(name in values for name in self.human_joints) or \
                not np.all(np.isfinite(torque)):
            rospy.logwarn_throttle(
                2.0, "external safety torque has no valid haptic joints")
            return
        self.external_safety = torque
        self.last_external_safety = rospy.Time.now()

    def component_torque(self):
        contact = np.zeros(self.human_count)
        safety = np.zeros(self.human_count)
        external = np.zeros(self.human_count)
        valid = False

        if self.enable_contact and self.robot_jacobian is not None and \
                self.fresh(self.last_contact, self.contact_timeout):
            try:
                contact = self.contact_model.compute(
                    self.contact_wrench, self.robot_jacobian, self.mapping_jacobian)
                valid = True
            except ValueError as error:
                rospy.logwarn_throttle(2.0, "contact haptic input rejected: %s", error)

        if self.enable_safety and self.fresh(self.last_shape_error, self.safety_timeout):
            try:
                safety = self.safety_model.compute(
                    self.shape_error, self.mapping_jacobian, self.human_velocity)
                valid = True
            except ValueError as error:
                rospy.logwarn_throttle(2.0, "safety haptic input rejected: %s", error)

        if self.fresh(self.last_external_safety, self.safety_timeout):
            external = self.external_safety.copy()
            valid = True
        return contact, safety, external, valid

    def publish_torque(self, publisher, torque, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(self.human_joints)
        msg.effort = [float(value) for value in torque]
        publisher.publish(msg)

    def update(self):
        now = rospy.Time.now()
        contact, safety, external, feedback_valid = self.component_torque()
        joint_valid = self.fresh(self.last_joint_state, self.joint_timeout)
        active = self.teleop_mode and joint_valid and feedback_valid

        if active:
            total = combine_torque(
                contact, safety, external, self.torque_limits, self.deadband)
        else:
            contact[:] = 0.0
            safety[:] = 0.0
            external[:] = 0.0
            total = np.zeros(self.human_count)

        dt = (now - self.previous_update).to_sec()
        if active:
            total = rate_limit(total, self.previous_torque, self.torque_rate_limits, dt)
        else:
            # A stale input or mode transition must not leave energy in the
            # slew limiter.  Hardware output is disabled in the same cycle.
            total[:] = 0.0
        self.previous_torque = total
        self.previous_update = now

        self.publish_torque(self.contact_pub, contact, now)
        self.publish_torque(self.safety_pub, safety + external, now)
        self.publish_torque(self.total_pub, total, now)
        self.active_pub.publish(Bool(data=active))
        self.servo_output.command(total, active)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()


if __name__ == "__main__":
    try:
        HapticController().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as error:
        rospy.logerr("haptic controller failed: %s", error)
        raise
