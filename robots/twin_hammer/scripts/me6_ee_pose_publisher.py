#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped

import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParam

import tf2_ros


class ME6EEPosePublisher:
    def __init__(self):
        rospy.init_node("me6_ee_pose_publisher")

        # ---- params ----
        self.base_link = rospy.get_param("~base_link", "base_link")
        self.ee_link   = rospy.get_param("~ee_link", "Link6")

        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/me6_robot/joint_states")
        self.joint_names = rospy.get_param(
            "~joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        )

        self.ee_pose_topic  = rospy.get_param("~ee_pose_topic",  "/me6_robot/ee_pose")
        self.ee_point_topic = rospy.get_param("~ee_point_topic", "/me6_robot/ee_point")

        self.rate_hz = float(rospy.get_param("~rate", 50.0))

        self.publish_tf = bool(rospy.get_param("~publish_tf", False))
        self.child_frame_id = rospy.get_param("~child_frame_id", "me6_ee")

        # ---- load URDF -> KDL ----
        rospy.loginfo("Loading URDF from /robot_description ...")
        ok, tree = treeFromParam("/robot_description")
        if not ok:
            raise RuntimeError("Failed to parse /robot_description")

        self.chain = tree.getChain(self.base_link, self.ee_link)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)

        # ---- ROS pubs/subs ----
        self.pose_pub = rospy.Publisher(self.ee_pose_topic, PoseStamped, queue_size=1)
        self.point_pub = rospy.Publisher(self.ee_point_topic, PointStamped, queue_size=1)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None

        self.current_joints = None
        rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_cb, queue_size=1)

        rospy.loginfo("ME6 EE Pose Publisher ready.")
        rospy.loginfo("  joint_state_topic: %s", self.joint_state_topic)
        rospy.loginfo("  base_link: %s, ee_link: %s", self.base_link, self.ee_link)
        rospy.loginfo("  publish: %s (PoseStamped)", self.ee_pose_topic)
        rospy.loginfo("  publish: %s (PointStamped)", self.ee_point_topic)
        rospy.loginfo("  publish_tf: %s", self.publish_tf)

    def joint_state_cb(self, msg: JointState):
        # name -> position を map
        joint_map = dict(zip(msg.name, msg.position))

        # 必要な関節が揃ってるか
        vals = []
        for j in self.joint_names:
            if j not in joint_map:
                return
            vals.append(float(joint_map[j]))

        self.current_joints = np.array(vals, dtype=float)

    def compute_fk(self, joints_np):
        q = kdl.JntArray(len(joints_np))
        for i, v in enumerate(joints_np):
            q[i] = float(v)

        frame = kdl.Frame()
        self.fk_solver.JntToCart(q, frame)
        return frame

    def publish(self, frame: kdl.Frame):
        now = rospy.Time.now()

        # KDL -> position
        px, py, pz = frame.p[0], frame.p[1], frame.p[2]

        # KDL -> quaternion
        qx, qy, qz, qw = frame.M.GetQuaternion()

        # PoseStamped
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = self.base_link
        ps.pose.position.x = px
        ps.pose.position.y = py
        ps.pose.position.z = pz
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pose_pub.publish(ps)

        # PointStamped
        pt = PointStamped()
        pt.header.stamp = now
        pt.header.frame_id = self.base_link
        pt.point.x = px
        pt.point.y = py
        pt.point.z = pz
        self.point_pub.publish(pt)

        # TF (optional)
        if self.publish_tf and self.tf_broadcaster is not None:
            ts = TransformStamped()
            ts.header.stamp = now
            ts.header.frame_id = self.base_link
            ts.child_frame_id = self.child_frame_id
            ts.transform.translation.x = px
            ts.transform.translation.y = py
            ts.transform.translation.z = pz
            ts.transform.rotation.x = qx
            ts.transform.rotation.y = qy
            ts.transform.rotation.z = qz
            ts.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(ts)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.current_joints is None:
                rate.sleep()
                continue

            frame = self.compute_fk(self.current_joints)
            self.publish(frame)
            rate.sleep()


if __name__ == "__main__":
    try:
        ME6EEPosePublisher().run()
    except rospy.ROSInterruptException:
        pass
