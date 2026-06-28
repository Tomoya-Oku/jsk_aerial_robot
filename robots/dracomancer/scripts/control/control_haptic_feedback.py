#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from spinal.msg import ServoControlCmd, ServoTorqueCmd


class HapticFeedback:
    def __init__(self):
        rospy.init_node("control_haptic_feedback")

        self.device_ns = rospy.get_param("~device_ns", "/dracomancer").rstrip("/")
        self.device_joint_topic = rospy.get_param("~device_joint_topic", self.device_ns + "/joint_states")
        self.shape_error_topic = rospy.get_param("~shape_error_topic", self.device_ns + "/shape_control_error")
        self.mode_topic = rospy.get_param("~mode_topic", self.device_ns + "/teleop_mode")
        self.haptic_debug_topic = rospy.get_param("~haptic_debug_topic", self.device_ns + "/haptic_torque")
        self.haptic_torque_enable_topic = rospy.get_param("~haptic_torque_enable_topic", "/servo/torque_enable")
        self.haptic_current_topic = rospy.get_param("~haptic_current_topic", "/servo/target_current")
        self.rate_hz = rospy.get_param("~rate", 40.0)

        self.enable_haptic_torque_onoff = rospy.get_param("~enable_haptic_torque_onoff", True)
        self.haptic_torque_on_threshold = rospy.get_param("~haptic_torque_on_threshold", 0.02)
        self.enable_haptic_current_command = rospy.get_param("~enable_haptic_current_command", False)
        self.haptic_current_per_nm = rospy.get_param("~haptic_current_per_nm", 0.0)
        self.haptic_stiffness = rospy.get_param("~haptic_stiffness", [0.2] * 6)
        self.haptic_damping = rospy.get_param("~haptic_damping", 0.01)
        self.haptic_torque_limits = rospy.get_param("~haptic_torque_limits", [0.2] * 7)
        self.shape_error_timeout = rospy.get_param("~shape_error_timeout", 0.5)
        self.haptic_servo_ids = rospy.get_param("~haptic_servo_ids", [0, 1, 2, 3, 4, 5, 6])

        self.source_joint_names = rospy.get_param("~source_joint_names", [
            "wrist_flexion_extension_joint",
            "wrist_supination_joint",
            "upper_arm_external_internal_rotation_joint",
            "elbow_flexion_extension_joint",
            "shoulder_flexion_extension_joint",
            "shoulder_abduction_adduction_joint",
        ])
        self.signs = rospy.get_param("~signs", [1.0, 1.0, 1.0, -1.0, 1.0, 1.0])
        self.scales = rospy.get_param("~scales", [1.0] * len(self.source_joint_names))
        self.haptic_device_joint_names = rospy.get_param("~haptic_device_joint_names", [
            "shoulder_abduction_adduction_joint",
            "shoulder_flexion_extension_joint",
            "upper_arm_external_internal_rotation_joint",
            "elbow_flexion_extension_joint",
            "wrist_supination_joint",
            "wrist_flexion_extension_joint",
            "wrist_abduction_adduction_joint",
        ])
        self.device_joint_index = {
            name: i for i, name in enumerate(self.haptic_device_joint_names)
        }

        self.teleop_mode = self.normalize_mode(rospy.get_param("~teleop_mode", "startup"))
        self.shape_error = [0.0] * len(self.source_joint_names)
        self.last_shape_error_stamp = rospy.Time(0)
        self.latest_device_joints = {}
        self.device_joint_velocity = {}
        self.last_device_joint_stamp = None

        self.haptic_debug_pub = rospy.Publisher(self.haptic_debug_topic, JointState, queue_size=1)
        self.haptic_torque_enable_pub = None
        if self.enable_haptic_torque_onoff:
            self.haptic_torque_enable_pub = rospy.Publisher(
                self.haptic_torque_enable_topic, ServoTorqueCmd, queue_size=1)
        self.haptic_current_pub = None
        if self.enable_haptic_current_command:
            if self.haptic_current_per_nm <= 0.0:
                rospy.logwarn("haptic_current_per_nm must be positive; haptic current command is disabled")
                self.enable_haptic_current_command = False
            else:
                self.haptic_current_pub = rospy.Publisher(self.haptic_current_topic, ServoControlCmd, queue_size=1)

        rospy.Subscriber(self.device_joint_topic, JointState, self.device_joint_cb, queue_size=1)
        rospy.Subscriber(self.shape_error_topic, Float64MultiArray, self.shape_error_cb, queue_size=1)
        rospy.Subscriber(self.mode_topic, String, self.mode_cb, queue_size=1)

        rospy.loginfo("device_joint_topic: %s", self.device_joint_topic)
        rospy.loginfo("shape_error_topic: %s", self.shape_error_topic)
        rospy.loginfo("haptic_debug_topic: %s", self.haptic_debug_topic)
        rospy.loginfo("haptic torque on/off command: %s -> %s",
                      self.enable_haptic_torque_onoff, self.haptic_torque_enable_topic)
        rospy.loginfo("haptic current command: %s -> %s",
                      self.enable_haptic_current_command, self.haptic_current_topic)

    @staticmethod
    def normalize_mode(mode):
        # "teleop" is accepted as a shorthand alias for "teleoperation".
        mode = str(mode).strip().lower()
        return "teleoperation" if mode == "teleop" else mode

    def mode_cb(self, msg):
        self.teleop_mode = self.normalize_mode(msg.data)

    def shape_error_cb(self, msg):
        self.shape_error = self.fit_list(msg.data, len(self.source_joint_names), 0.0)
        self.last_shape_error_stamp = rospy.Time.now()

    def device_joint_cb(self, msg):
        now = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        next_joints = {
            name: float(pos) for name, pos in zip(msg.name, msg.position)
        }
        if self.last_device_joint_stamp is not None:
            dt = (now - self.last_device_joint_stamp).to_sec()
            if dt > 1e-6:
                self.device_joint_velocity = {
                    name: (pos - self.latest_device_joints.get(name, pos)) / dt
                    for name, pos in next_joints.items()
                }
        self.latest_device_joints = next_joints
        self.last_device_joint_stamp = now

    @staticmethod
    def fit_list(values, size, default):
        data = list(values)
        if len(data) < size:
            data.extend([default] * (size - len(data)))
        return data[:size]

    def shape_error_ready(self):
        return (rospy.Time.now() - self.last_shape_error_stamp).to_sec() <= self.shape_error_timeout

    def haptic_torque(self):
        size = len(self.haptic_device_joint_names)
        if self.teleop_mode != "teleoperation" or not self.shape_error_ready():
            return [0.0] * size

        stiffness = self.fit_list(self.haptic_stiffness, len(self.source_joint_names), 0.0)
        limits = self.fit_list(self.haptic_torque_limits, size, 0.0)
        tau = [0.0] * size

        for j, source_name in enumerate(self.source_joint_names):
            device_idx = self.device_joint_index.get(source_name)
            if device_idx is None:
                continue
            shape_torque = stiffness[j] * self.shape_error[j]
            tau[device_idx] += self.signs[j] * self.scales[j] * shape_torque

        for name, velocity in self.device_joint_velocity.items():
            device_idx = self.device_joint_index.get(name)
            if device_idx is not None:
                tau[device_idx] -= self.haptic_damping * velocity

        return [
            max(-abs(limit), min(abs(limit), value))
            for value, limit in zip(tau, limits)
        ]

    def publish_haptic(self, torque):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.haptic_device_joint_names
        msg.effort = list(torque)
        self.haptic_debug_pub.publish(msg)

        if self.enable_haptic_torque_onoff and self.haptic_torque_enable_pub is not None:
            enable_msg = ServoTorqueCmd()
            enable_msg.index = list(self.haptic_servo_ids)
            enable_msg.torque_enable = [
                1 if abs(value) >= self.haptic_torque_on_threshold else 0
                for value in torque
            ]
            self.haptic_torque_enable_pub.publish(enable_msg)

        if self.enable_haptic_current_command and self.haptic_current_pub is not None:
            current_msg = ServoControlCmd()
            current_msg.index = list(self.haptic_servo_ids)
            current_msg.angles = [
                int(max(-32768, min(32767, round(value * self.haptic_current_per_nm))))
                for value in torque
            ]
            current_msg.stamp = rospy.Time.now()
            self.haptic_current_pub.publish(current_msg)

    def main(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.publish_haptic(self.haptic_torque())
            rate.sleep()


if __name__ == "__main__":
    try:
        node = HapticFeedback()
        node.main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))
