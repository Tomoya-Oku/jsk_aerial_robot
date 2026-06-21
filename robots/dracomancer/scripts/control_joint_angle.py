#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray, UInt8, String
from sensor_msgs.msg import JointState

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
        # Shape-safety scale is computed by volume_radius_monitor (bringup.launch) and
        # consumed here to blend joint commands toward safe_pose. When the scale is
        # stale/absent, missing_safety_scale (default 1.0 = no limiting) is used.
        self.safety_scale_topic = rospy.get_param(
            "~safety_scale_topic", self.device_ns + "/dragon_shape_safety_scale")
        self.safety_scale_timeout = rospy.get_param("~safety_scale_timeout", 0.5)
        self.missing_safety_scale = rospy.get_param("~missing_safety_scale", 1.0)

        self.latest_device_joints = {}
        self.neutral_device_joints = {}
        self.current_target = list(self.safe_pose)
        self.safety_scale_value = None
        self.last_safety_scale_stamp = rospy.Time(0)
        self.robot_hovering = False

        # Publisher
        self.joints_ctrl_pub = rospy.Publisher(self.command_topic, JointState, queue_size=10)
        self.shape_error_pub = rospy.Publisher(self.shape_error_topic, Float64MultiArray, queue_size=1)

        # Subscriber
        self.device_joint_sub = rospy.Subscriber(self.device_joint_topic, JointState, self.device_joint_cb, queue_size=1)
        self.safety_scale_sub = rospy.Subscriber(self.safety_scale_topic, Float64, self.safety_scale_cb, queue_size=1)
        self.robot_flight_state_sub = rospy.Subscriber('/' + self.robot_name + '/flight_state', UInt8, self.robot_flight_state_cb, queue_size=1)
        self.mode_sub = rospy.Subscriber(self.mode_topic, String, self.mode_cb, queue_size=1)

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
        rospy.loginfo("safety scale topic: %s (timeout=%.2f, missing=%.3f)",
                      self.safety_scale_topic, self.safety_scale_timeout, self.missing_safety_scale)
        rospy.loginfo("joint command gating: only_when_hovering=%s, before_device_ready=%s",
                      self.publish_only_when_hovering, self.publish_before_device_ready)

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

    def safety_scale_cb(self, msg):
        self.safety_scale_value = float(msg.data)
        self.last_safety_scale_stamp = rospy.Time.now()

    def robot_flight_state_cb(self, msg):
        self.robot_hovering = int(msg.data) >= 4

    def safety_scale(self):
        # Use the latest scale from volume_radius_monitor; fall back to
        # missing_safety_scale when it is absent or stale.
        if self.safety_scale_value is None:
            return max(0.0, min(1.0, self.missing_safety_scale))
        if (rospy.Time.now() - self.last_safety_scale_stamp).to_sec() > self.safety_scale_timeout:
            return max(0.0, min(1.0, self.missing_safety_scale))
        return max(0.0, min(1.0, self.safety_scale_value))

    def mapped_target(self):
        if not self.latest_device_joints:
            return list(self.safe_pose)

        target = []
        for i, source_name in enumerate(self.source_joint_names):
            source = self.latest_device_joints.get(source_name)
            if source is None:
                target.append(self.current_target[i])
                continue

            neutral = self.neutral_device_joints.get(source_name, 0.0)
            mapped = self.offsets[i] + self.signs[i] * self.scales[i] * (source - neutral)
            target.append(self.clamp(mapped))

        return target

    def desired_target(self):
        if self.teleop_mode == "startup":
            return list(self.startup_pose)
        if self.teleop_mode == "wide":
            return list(self.wide_hold_pose)
        return self.mapped_target()

    @staticmethod
    def blend(a, b, ratio):
        return [ai + ratio * (bi - ai) for ai, bi in zip(a, b)]

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
        scale = self.safety_scale()
        desired = self.desired_target()

        if scale <= 0.0:
            target = list(self.safe_pose)
        elif scale < 1.0:
            target = self.blend(self.safe_pose, desired, scale)
        else:
            target = desired

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
