#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Dry-run stand-in for a hovering DRAGON (no Gazebo / no hardware).

Tracks /<robot>/joints_ctrl with a servo-like rate limit and republishes the
result as /<robot>/joint_states, while pretending to hover
(flight_state = 5) and publishing constant, safe fc radii. This closes the
loop for the shape-target reaching task pipeline so target_manager /
task_recorder / shadow_visualizer / keyboard_baseline can be tested on a
laptop alone.
"""

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, UInt8

import dragon_fk


class DummyRobot:
    def __init__(self):
        rospy.init_node("dummy_robot")

        self.robot_ns = rospy.get_param("~robot_ns", "dragon")
        self.joint_names = rospy.get_param("~dragon_joint_names", list(dragon_fk.JOINT_ORDER))
        self.rate_hz = float(rospy.get_param("~rate", 40.0))
        self.max_step = float(rospy.get_param("~max_step", 0.04))  # rad/cycle, servo-like lag
        self.q = list(rospy.get_param(
            "~initial_pose", [0.0, np.pi / 2.0, 0.0, np.pi / 2.0, 0.0, np.pi / 2.0]))
        self.fc_f = float(rospy.get_param("~fc_f_min", 0.40))   # N, comfortably safe
        self.fc_t = float(rospy.get_param("~fc_t_min", 0.40))   # N·m
        self.flight_state = int(rospy.get_param("~flight_state", 5))  # HOVER

        self.target = list(self.q)

        ns = "/" + self.robot_ns
        self.js_pub = rospy.Publisher(ns + "/joint_states", JointState, queue_size=1)
        self.fs_pub = rospy.Publisher(ns + "/flight_state", UInt8, queue_size=1)
        self.fc_f_pub = rospy.Publisher(ns + "/debug/fc_f_min_filtered", Float64, queue_size=1)
        self.fc_t_pub = rospy.Publisher(ns + "/debug/fc_t_min_filtered", Float64, queue_size=1)
        rospy.Subscriber(ns + "/joints_ctrl", JointState, self.cmd_cb, queue_size=1)

        rospy.loginfo("dummy DRAGON: joints_ctrl -> joint_states, flight_state=%d, "
                      "fc_f=%.2f fc_t=%.2f", self.flight_state, self.fc_f, self.fc_t)

    def cmd_cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        for i, name in enumerate(self.joint_names):
            if name in pos:
                self.target[i] = float(pos[name])

    def main(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.q = [q + float(np.clip(t - q, -self.max_step, self.max_step))
                      for q, t in zip(self.q, self.target)]
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = list(self.joint_names)
            msg.position = list(self.q)
            self.js_pub.publish(msg)
            self.fs_pub.publish(UInt8(self.flight_state))
            self.fc_f_pub.publish(Float64(self.fc_f))
            self.fc_t_pub.publish(Float64(self.fc_t))
            rate.sleep()


if __name__ == "__main__":
    try:
        DummyRobot().main()
    except rospy.ROSInterruptException:
        pass
