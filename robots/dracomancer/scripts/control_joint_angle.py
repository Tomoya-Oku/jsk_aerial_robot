#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, UInt8, String
from sensor_msgs.msg import JointState
from dracomancer.srv import ShapeFeasibility, ShapeFeasibilityRequest

class ControlJoints:
    def __init__(self):
        rospy.init_node("control_joint_angle")

        # Parameters
        self.robot_name = rospy.get_param("~robot_name", "dragon")
        self.device_ns = rospy.get_param("~device_ns", "/dracomancer").rstrip("/")
        self.device_joint_topic = rospy.get_param("~device_joint_topic", "/dracomancer/joint_states")
        self.command_topic = rospy.get_param("~command_topic", "/" + self.robot_name + "/joints_ctrl")
        self.rate_hz = rospy.get_param("~rate", 40.0)
        self.teleop_mode = str(rospy.get_param("~teleop_mode", "startup")).lower()
        self.mode_topic = rospy.get_param("~mode_topic", self.device_ns + "/teleop_mode")
        self.valid_modes = ("startup", "precision", "wide")
        if self.teleop_mode not in self.valid_modes:
            rospy.logwarn("unknown teleop_mode '%s', fall back to 'startup'", self.teleop_mode)
            self.teleop_mode = "startup"

        self.joint_names = rospy.get_param("~dragon_joint_names", [
            "joint1_pitch",
            "joint1_yaw",
            "joint2_pitch",
            "joint2_yaw",
            "joint3_pitch",
            "joint3_yaw",
        ])
        self.source_joint_names = rospy.get_param("~source_joint_names", [
            "wrist_flexion_extension_joint",
            "wrist_supination_joint",
            "upper_arm_external_internal_rotation_joint",
            "elbow_flexion_extension_joint",
            "shoulder_flexion_extension_joint",
            "shoulder_abduction_adduction_joint",
        ])
        self.signs = rospy.get_param("~signs", [1.0, 1.0, 1.0, -1.0, 1.0, 1.0])
        self.scales = rospy.get_param("~scales", [1.0] * len(self.joint_names))
        self.offsets = rospy.get_param("~offsets", [0.0, np.pi / 2.0, 0.0, 0.0, 0.0, np.pi / 2.0])
        self.startup_pose = rospy.get_param("~startup_pose", [0.0, np.pi / 2.0, 0.0, np.pi / 2.0, 0.0, np.pi / 2.0])
        self.wide_hold_pose = rospy.get_param("~wide_hold_pose", self.startup_pose)
        self.safe_pose = rospy.get_param("~safe_pose", self.startup_pose)
        self.joint_limit = rospy.get_param("~joint_limit", np.pi / 2.0)
        self.max_step = rospy.get_param("~max_step", 0.04)
        self.capture_neutral = rospy.get_param("~capture_neutral_on_first_msg", False)
        self.publish_only_when_hovering = rospy.get_param("~publish_only_when_hovering", True)
        self.publish_before_device_ready = rospy.get_param("~publish_before_device_ready", False)

        self.shape_error_topic = rospy.get_param("~shape_error_topic", self.device_ns + "/shape_control_error")

        # Predictive shape-feasibility gate (precision mode):
        #   candidate shape -> shape_feasibility service -> fc_f_min / fc_t_min.
        #   Deform only when BOTH radii are at or above their lower thresholds;
        #   otherwise hold the last feasible shape.
        self.enable_feasibility_gate = rospy.get_param("~enable_feasibility_gate", True)
        self.feasibility_service_name = rospy.get_param(
            "~feasibility_service", "/" + self.robot_name + "/shape_feasibility/check_shape")
        self.feasibility_service_timeout = rospy.get_param("~feasibility_service_timeout", 2.0)
        # The model plugin runs a gimbal-planning optimization per call, so throttle
        # how often the candidate is re-evaluated (Hz). Between checks the last
        # feasible shape is held.
        self.feasibility_rate = rospy.get_param("~feasibility_rate", 20.0)
        # Lower thresholds (fallback params; overridden by the threshold topics below).
        self.force_radius_threshold = rospy.get_param("~force_radius_threshold", 0.1)
        self.torque_radius_threshold = rospy.get_param("~torque_radius_threshold", 0.01)
        # Threshold topics carry [hard_min, min]; hard_min ([0]) is used as the gate bound.
        self.force_threshold_topic = rospy.get_param(
            "~force_volume_radius_threshold_topic", self.device_ns + "/force_volume_radius_threshold")
        self.torque_threshold_topic = rospy.get_param(
            "~torque_volume_radius_threshold_topic", self.device_ns + "/torque_volume_radius_threshold")
        self.gate_log_period = rospy.get_param("~gate_log_period", 1.0)

        self.latest_device_joints = {}
        self.neutral_device_joints = {}
        self.current_target = list(self.safe_pose)
        self.last_feasible_target = list(self.safe_pose)
        self.robot_hovering = False
        self.last_gate_log_stamp = rospy.Time(0)
        self.last_gate_feasible = None
        self.last_feasibility_eval_stamp = rospy.Time(0)

        # Publisher
        self.joints_ctrl_pub = rospy.Publisher(self.command_topic, JointState, queue_size=10)
        self.shape_error_pub = rospy.Publisher(self.shape_error_topic, Float64MultiArray, queue_size=1)

        # Service client (persistent for rate; reconnected on failure)
        self.feasibility_srv = None
        if self.enable_feasibility_gate:
            self.connect_feasibility_service()

        # Subscriber
        self.device_joint_sub = rospy.Subscriber(self.device_joint_topic, JointState, self.device_joint_cb, queue_size=1)
        self.robot_flight_state_sub = rospy.Subscriber('/' + self.robot_name + '/flight_state', UInt8, self.robot_flight_state_cb, queue_size=1)
        self.mode_sub = rospy.Subscriber(self.mode_topic, String, self.mode_cb, queue_size=1)
        self.force_threshold_sub = rospy.Subscriber(self.force_threshold_topic, Float64MultiArray, self.force_threshold_cb, queue_size=1)
        self.torque_threshold_sub = rospy.Subscriber(self.torque_threshold_topic, Float64MultiArray, self.torque_threshold_cb, queue_size=1)

        rospy.loginfo("teleop_mode: %s, mode_topic: %s", self.teleop_mode, self.mode_topic)
        rospy.loginfo("device_joint_topic: %s", self.device_joint_topic)
        rospy.loginfo("command_topic: %s", self.command_topic)
        rospy.loginfo("joint mapping: %s",
                      ", ".join("{}<-{}".format(dst, src)
                                for dst, src in zip(self.joint_names, self.source_joint_names)))
        rospy.loginfo("joint mapping scale/sign/offset: %s",
                      ", ".join("{}:{:.3f}/{:.3f}/{:.3f}".format(
                          name, scale, sign, offset)
                                for name, scale, sign, offset in zip(
                                    self.joint_names, self.scales, self.signs, self.offsets)))
        rospy.loginfo("feasibility gate: enable=%s, service=%s, thresholds force/torque=%.4f/%.4f",
                      self.enable_feasibility_gate, self.feasibility_service_name,
                      self.force_radius_threshold, self.torque_radius_threshold)
        rospy.loginfo("joint command gating: only_when_hovering=%s, before_device_ready=%s",
                      self.publish_only_when_hovering, self.publish_before_device_ready)

    def connect_feasibility_service(self):
        try:
            rospy.wait_for_service(self.feasibility_service_name, timeout=self.feasibility_service_timeout)
            self.feasibility_srv = rospy.ServiceProxy(self.feasibility_service_name, ShapeFeasibility, persistent=True)
            rospy.loginfo("connected to shape feasibility service: %s", self.feasibility_service_name)
            return True
        except rospy.ROSException:
            rospy.logwarn_throttle(5.0, "shape feasibility service '%s' not available", self.feasibility_service_name)
            self.feasibility_srv = None
            return False

    def mode_cb(self, msg):
        mode = str(msg.data).strip().lower()
        if mode not in self.valid_modes:
            rospy.logwarn("ignore unknown teleop mode '%s'", mode)
            return
        if mode != self.teleop_mode:
            rospy.loginfo("teleop mode: %s -> %s", self.teleop_mode, mode)
            self.teleop_mode = mode

    def clamp(self, x):
        return max(-self.joint_limit, min(self.joint_limit, x))

    def device_joint_cb(self, msg):
        self.latest_device_joints = {
            name: float(pos) for name, pos in zip(msg.name, msg.position)
        }
        if self.capture_neutral and not self.neutral_device_joints:
            if all(name in self.latest_device_joints for name in self.source_joint_names):
                self.neutral_device_joints = {
                    name: self.latest_device_joints[name] for name in self.source_joint_names
                }
                rospy.loginfo("Captured dracomancer neutral joints for DRAGON mapping")

    def robot_flight_state_cb(self, msg):
        self.robot_hovering = int(msg.data) >= 4

    def force_threshold_cb(self, msg):
        if len(msg.data) > 0:
            self.force_radius_threshold = float(msg.data[0])

    def torque_threshold_cb(self, msg):
        if len(msg.data) > 0:
            self.torque_radius_threshold = float(msg.data[0])

    def mapped_target(self):
        if not self.latest_device_joints:
            return list(self.last_feasible_target)

        target = []
        for i, source_name in enumerate(self.source_joint_names):
            source = self.latest_device_joints.get(source_name)
            if source is None:
                target.append(self.last_feasible_target[i])
                continue

            neutral = self.neutral_device_joints.get(source_name, 0.0)
            mapped = self.offsets[i] + self.signs[i] * self.scales[i] * (source - neutral)
            target.append(self.clamp(mapped))

        return target

    def evaluate_feasibility(self, candidate):
        # Returns (feasible: bool, fc_f_min, fc_t_min). On service failure returns
        # (None, ...) so the caller can hold the last feasible shape conservatively.
        if self.feasibility_srv is None:
            if not self.connect_feasibility_service():
                return None, None, None
        req = ShapeFeasibilityRequest()
        req.name = list(self.joint_names)
        req.position = list(candidate)
        try:
            res = self.feasibility_srv.call(req)
        except (rospy.ServiceException, TypeError):
            rospy.logwarn_throttle(5.0, "shape feasibility service call failed; reconnecting")
            self.feasibility_srv = None
            return None, None, None
        if not res.valid:
            return None, res.fc_f_min, res.fc_t_min
        feasible = (res.fc_f_min >= self.force_radius_threshold and
                    res.fc_t_min >= self.torque_radius_threshold)
        return feasible, res.fc_f_min, res.fc_t_min

    def log_gate(self, feasible, fc_f, fc_t):
        if feasible != self.last_gate_feasible:
            self.last_gate_feasible = feasible
        now = rospy.Time.now()
        if self.gate_log_period > 0.0 and (now - self.last_gate_log_stamp).to_sec() < self.gate_log_period:
            return
        self.last_gate_log_stamp = now
        f = "n/a" if fc_f is None else "%.4f" % fc_f
        t = "n/a" if fc_t is None else "%.4f" % fc_t
        msg = ("shape feasibility: deform=%s fc_f_min=%s fc_t_min=%s "
               "thresholds(force/torque)=%.4f/%.4f") % (
            feasible, f, t, self.force_radius_threshold, self.torque_radius_threshold)
        if feasible:
            rospy.loginfo(msg)
        else:
            rospy.logwarn(msg)

    def precision_target(self):
        # Map the arm to a candidate DRAGON shape, then gate by predicted feasibility.
        candidate = self.mapped_target()
        if not self.enable_feasibility_gate:
            self.last_feasible_target = candidate
            return candidate

        # Throttle the (expensive) feasibility evaluation; hold between checks.
        now = rospy.Time.now()
        if self.feasibility_rate > 0.0 and \
                (now - self.last_feasibility_eval_stamp).to_sec() < 1.0 / self.feasibility_rate:
            return list(self.last_feasible_target)
        self.last_feasibility_eval_stamp = now

        feasible, fc_f, fc_t = self.evaluate_feasibility(candidate)
        if feasible:
            self.last_feasible_target = candidate
        # feasible False or None (service failure / invalid) -> hold last feasible.
        self.log_gate(bool(feasible), fc_f, fc_t)
        return list(self.last_feasible_target)

    def desired_target(self):
        if self.teleop_mode == "startup":
            self.last_feasible_target = list(self.startup_pose)
            return list(self.startup_pose)
        if self.teleop_mode == "wide":
            self.last_feasible_target = list(self.wide_hold_pose)
            return list(self.wide_hold_pose)
        return self.precision_target()

    def rate_limit(self, target):
        limited = []
        for cur, dst in zip(self.current_target, target):
            delta = max(-self.max_step, min(self.max_step, dst - cur))
            limited.append(cur + delta)
        return limited

    def publish_shape_error(self, desired, target):
        msg = Float64MultiArray()
        msg.data = [float(d - t) for d, t in zip(desired, target)]
        self.shape_error_pub.publish(msg)

    def make_joint_msg(self):
        target = self.desired_target()
        # desired (raw mapping) vs target (feasible-gated) error, for haptic feedback.
        desired = self.mapped_target() if self.teleop_mode == "precision" else target
        self.publish_shape_error(desired, target)
        self.current_target = self.rate_limit(target)

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = list(self.current_target)
        return msg

    def can_publish_joint_command(self):
        if self.publish_only_when_hovering and not self.robot_hovering:
            return False
        if self.teleop_mode == "precision" and not self.publish_before_device_ready and not self.latest_device_joints:
            return False
        return True

    def main(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            if self.can_publish_joint_command():
                self.joints_ctrl_pub.publish(self.make_joint_msg())
            rate.sleep()


if __name__ == "__main__":
    try:
        node = ControlJoints()
        node.main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))
