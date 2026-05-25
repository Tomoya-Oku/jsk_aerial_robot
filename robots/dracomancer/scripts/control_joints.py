#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray, UInt8
from sensor_msgs.msg import JointState


class ControlJoints:
    def __init__(self):
        rospy.init_node("control_joints")

        # Parameters
        self.robot_name = rospy.get_param("~robot_name", "dragon")
        self.device_joint_topic = rospy.get_param("~device_joint_topic", "/dracomancer/joint_states")
        self.command_topic = rospy.get_param("~command_topic", "/" + self.robot_name + "/joints_ctrl")
        self.rate_hz = rospy.get_param("~rate", 40.0)

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
        self.safe_pose = rospy.get_param("~safe_pose", [0.0, np.pi / 2.0, 0.0, np.pi / 2.0, 0.0, np.pi / 2.0])
        self.joint_limit = rospy.get_param("~joint_limit", np.pi / 2.0)
        self.max_step = rospy.get_param("~max_step", 0.04)
        self.capture_neutral = rospy.get_param("~capture_neutral_on_first_msg", False)
        self.publish_only_when_hovering = rospy.get_param("~publish_only_when_hovering", True)
        self.publish_before_device_ready = rospy.get_param("~publish_before_device_ready", False)

        self.force_inradius_topic = rospy.get_param("~force_inradius_topic", "/" + self.robot_name + "/debug/fc_f_min")
        self.torque_inradius_topic = rospy.get_param("~torque_inradius_topic", "/" + self.robot_name + "/debug/fc_t_min")
        self.force_inradius_min = rospy.get_param("~force_inradius_min", 0.2)
        self.torque_inradius_min = rospy.get_param("~torque_inradius_min", 0.02)
        self.force_inradius_hard_min = rospy.get_param("~force_inradius_hard_min", 0.1)
        self.torque_inradius_hard_min = rospy.get_param("~torque_inradius_hard_min", 0.01)
        self.inradius_timeout = rospy.get_param("~inradius_timeout", 0.5)
        self.enable_shape_safety = rospy.get_param("~enable_shape_safety", True)
        self.missing_inradius_scale = rospy.get_param("~missing_inradius_scale", 0.0)
        self.min_safety_scale = rospy.get_param("~min_safety_scale", 0.0)

        self.latest_device_joints = {}
        self.neutral_device_joints = {}
        self.current_target = list(self.safe_pose)
        self.force_inradius = None
        self.torque_inradius = None
        self.last_inradius_stamp = rospy.Time(0)
        self.robot_hovering = False

        # Publisher
        self.joints_ctrl_pub = rospy.Publisher(self.command_topic, JointState, queue_size=10)
        self.safety_pub = rospy.Publisher("/dracomancer/dragon_shape_safety", Float64MultiArray, queue_size=1)

        # Subscriber
        self.device_joint_sub = rospy.Subscriber(self.device_joint_topic, JointState, self.device_joint_cb, queue_size=1)
        self.force_inradius_sub = rospy.Subscriber(self.force_inradius_topic, Float64, self.force_inradius_cb, queue_size=1)
        self.torque_inradius_sub = rospy.Subscriber(self.torque_inradius_topic, Float64, self.torque_inradius_cb, queue_size=1)
        self.robot_flight_state_sub = rospy.Subscriber('/' + self.robot_name + '/flight_state', UInt8, self.robot_flight_state_cb, queue_size=1)

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
        rospy.loginfo("force/torque inradius topics: %s, %s", self.force_inradius_topic, self.torque_inradius_topic)
        rospy.loginfo("joint command gating: only_when_hovering=%s, before_device_ready=%s",
                      self.publish_only_when_hovering, self.publish_before_device_ready)
        rospy.loginfo(
            "shape safety: enable=%s, missing_scale=%.3f, min_scale=%.3f, force=(%.3f/%.3f), torque=(%.3f/%.3f)",
            self.enable_shape_safety, self.missing_inradius_scale, self.min_safety_scale,
            self.force_inradius_hard_min, self.force_inradius_min,
            self.torque_inradius_hard_min, self.torque_inradius_min)

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

    def force_inradius_cb(self, msg):
        self.force_inradius = float(msg.data)
        self.last_inradius_stamp = rospy.Time.now()

    def torque_inradius_cb(self, msg):
        self.torque_inradius = float(msg.data)
        self.last_inradius_stamp = rospy.Time.now()

    def robot_flight_state_cb(self, msg):
        self.robot_hovering = int(msg.data) >= 4

    def inradius_ready(self):
        if self.force_inradius is None or self.torque_inradius is None:
            return False
        return (rospy.Time.now() - self.last_inradius_stamp).to_sec() <= self.inradius_timeout

    def safety_scale(self):
        if not self.enable_shape_safety:
            return 1.0
        if not self.inradius_ready():
            return max(0.0, min(1.0, self.missing_inradius_scale))
        if (self.force_inradius <= self.force_inradius_hard_min or
                self.torque_inradius <= self.torque_inradius_hard_min):
            return max(0.0, min(1.0, self.min_safety_scale))
        force_margin = (self.force_inradius - self.force_inradius_hard_min) / max(
            self.force_inradius_min - self.force_inradius_hard_min, 1e-6)
        torque_margin = (self.torque_inradius - self.torque_inradius_hard_min) / max(
            self.torque_inradius_min - self.torque_inradius_hard_min, 1e-6)
        return max(self.min_safety_scale, min(1.0, force_margin, torque_margin))

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

    @staticmethod
    def blend(a, b, ratio):
        return [ai + ratio * (bi - ai) for ai, bi in zip(a, b)]

    def rate_limit(self, target):
        limited = []
        for cur, dst in zip(self.current_target, target):
            delta = max(-self.max_step, min(self.max_step, dst - cur))
            limited.append(cur + delta)
        return limited

    def make_joint_msg(self):
        scale = self.safety_scale()
        desired = self.mapped_target()

        if scale <= 0.0:
            target = list(self.safe_pose)
        elif scale < 1.0:
            target = self.blend(self.safe_pose, desired, scale)
        else:
            target = desired

        self.current_target = self.rate_limit(target)

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = list(self.current_target)
        return msg

    def can_publish_joint_command(self):
        if self.publish_only_when_hovering and not self.robot_hovering:
            return False
        if not self.publish_before_device_ready and not self.latest_device_joints:
            return False
        return True

    def publish_safety(self):
        msg = Float64MultiArray()
        msg.data = [
            float(self.force_inradius if self.force_inradius is not None else -1.0),
            float(self.torque_inradius if self.torque_inradius is not None else -1.0),
            float(self.safety_scale()),
        ]
        self.safety_pub.publish(msg)

    def main(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            if self.can_publish_joint_command():
                self.joints_ctrl_pub.publish(self.make_joint_msg())
            self.publish_safety()
            rate.sleep()


if __name__ == "__main__":
    try:
        node = ControlJoints()
        node.main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))
