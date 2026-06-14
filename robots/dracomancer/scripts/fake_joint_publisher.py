#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fake joint state publisher for Dracomancer simulation (rm:=false).

Subscribes to ~joint_cmd (sensor_msgs/JointState) from the web console or
joint_state_publisher_gui, then republishes to /dracomancer/joint_states at a
fixed rate so downstream nodes receive a steady stream.
"""

import rospy
from sensor_msgs.msg import JointState

DEFAULT_JOINT_NAMES = [
    "shoulder_abduction_adduction_joint",
    "shoulder_flexion_extension_joint",
    "upper_arm_external_internal_rotation_joint",
    "elbow_flexion_extension_joint",
    "wrist_supination_joint",
    "wrist_flexion_extension_joint",
    "wrist_abduction_adduction_joint",
]


class FakeJointPublisher:
    def __init__(self):
        rospy.init_node("fake_joint_publisher")

        self.joint_names = rospy.get_param("~joint_names", DEFAULT_JOINT_NAMES)
        self.rate_hz = rospy.get_param("~rate", 40.0)
        self.joint_states_topic = rospy.get_param(
            "~joint_states_topic", "/dracomancer/joint_states"
        )
        self.joint_cmd_topic = rospy.get_param(
            "~joint_cmd_topic", "/dracomancer/joint_cmd"
        )

        self.positions = [0.0] * len(self.joint_names)
        self._name_to_idx = {n: i for i, n in enumerate(self.joint_names)}

        self.pub = rospy.Publisher(
            self.joint_states_topic, JointState, queue_size=10
        )
        self.sub = rospy.Subscriber(
            self.joint_cmd_topic, JointState, self._cmd_cb, queue_size=1
        )

        rospy.loginfo("[fake_joint_publisher] publishing  → %s", self.joint_states_topic)
        rospy.loginfo("[fake_joint_publisher] commands    ← %s", self.joint_cmd_topic)

    def _cmd_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            idx = self._name_to_idx.get(name)
            if idx is not None:
                self.positions[idx] = float(pos)

    def _make_msg(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(self.joint_names)
        msg.position = list(self.positions)
        return msg

    def main(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.pub.publish(self._make_msg())
            rate.sleep()


if __name__ == "__main__":
    try:
        node = FakeJointPublisher()
        node.main()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("[fake_joint_publisher] %s", exc)
