#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseArray, PoseStamped

import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParam
from trac_ik_python.trac_ik import IK

import tf2_ros
import tf2_geometry_msgs  # noqa


class ME6WaypointsIKControllerTF:
    def __init__(self):
        rospy.init_node("me6_waypoints_ik_controller_tf")

        # Params
        self.base_link = rospy.get_param("~base_link", "base_link")
        self.ee_link   = rospy.get_param("~ee_link", "Link6")
        self.way_topic = rospy.get_param("~waypoints_topic", "/way_points")
        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/me6_robot/joint_states")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/me6_robot/joint_controller/command")

        self.point_duration = rospy.get_param("~point_duration", 2.0)
        self.settle_time    = rospy.get_param("~settle_time", 0.3)
        self.pos_tol        = rospy.get_param("~pos_tol", 0.01)     # meters
        self.max_wait_fk    = rospy.get_param("~max_wait_fk", 3.0)  # seconds

        self.tf_timeout     = rospy.get_param("~tf_timeout", 0.5)   # seconds

        self.joint_names = rospy.get_param("~joint_names", [
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6"
        ])

        # Load URDF
        rospy.loginfo("Loading URDF from /robot_description...")
        ok, tree = treeFromParam("/robot_description")
        if not ok:
            raise RuntimeError("Failed to parse robot_description")

        # KDL chain + FK
        self.chain = tree.getChain(self.base_link, self.ee_link)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)

        # TRAC-IK
        rospy.loginfo("Initializing TRAC-IK solver (%s -> %s)...", self.base_link, self.ee_link)
        self.ik_solver = IK(self.base_link, self.ee_link)

        # TF buffer
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # State
        self.current_joints = None
        self.waypoints_msg = None

        rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_cb, queue_size=1)
        rospy.Subscriber(self.way_topic, PoseArray, self.waypoints_cb, queue_size=1)

        self.cmd_pub = rospy.Publisher(self.cmd_topic, JointTrajectory, queue_size=1)

        rospy.loginfo("Waiting for joint states and /way_points ...")
        while not rospy.is_shutdown() and self.current_joints is None:
            rospy.sleep(0.05)
        while not rospy.is_shutdown() and self.waypoints_msg is None:
            rospy.sleep(0.05)

        rospy.loginfo("Ready. Starting...")

    def joint_state_cb(self, msg: JointState):
        joint_map = dict(zip(msg.name, msg.position))
        self.current_joints = np.array([joint_map.get(j, 0.0) for j in self.joint_names], dtype=float)

    def waypoints_cb(self, msg: PoseArray):
        self.waypoints_msg = msg

    def compute_fk_pos(self, joints_np):
        q = kdl.JntArray(len(joints_np))
        for i, v in enumerate(joints_np):
            q[i] = float(v)
        frame = kdl.Frame()
        self.fk_solver.JntToCart(q, frame)
        return np.array([frame.p[0], frame.p[1], frame.p[2]], dtype=float)

    def send_joint_command(self, joints_np):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = joints_np.tolist()
        pt.time_from_start = rospy.Duration(self.point_duration)
        traj.points.append(pt)
        self.cmd_pub.publish(traj)

    def wait_until_reached(self, target_xyz_base):
        t0 = rospy.Time.now()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            ee = self.compute_fk_pos(self.current_joints)  # base_link
            err = np.linalg.norm(ee - target_xyz_base)
            if err <= self.pos_tol:
                return True, float(err)
            if (rospy.Time.now() - t0).to_sec() > self.max_wait_fk:
                return False, float(err)
            rate.sleep()

    def pose_to_base_link(self, pose, src_frame, stamp):
        """
        pose: geometry_msgs/Pose
        src_frame: str (e.g. 'world')
        stamp: rospy.Time
        returns: PoseStamped in base_link frame
        """
        ps = PoseStamped()
        ps.header.frame_id = src_frame
        ps.header.stamp = stamp
        ps.pose = pose

        # base_link へ変換
        trans = self.tf_buffer.lookup_transform(
            self.base_link,
            src_frame,
            stamp,
            rospy.Duration(self.tf_timeout)
        )
        ps_base = tf2_geometry_msgs.do_transform_pose(ps, trans)
        return ps_base

    def run(self):
        msg = self.waypoints_msg
        poses = list(msg.poses)
        if not poses:
            rospy.logwarn("No waypoints.")
            return

        src_frame = msg.header.frame_id or "world"
        stamp = msg.header.stamp
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()

        rospy.loginfo("Following %d waypoints (src_frame=%s -> base_link=%s)...",
                      len(poses), src_frame, self.base_link)

        seed = np.copy(self.current_joints)

        for i, p in enumerate(poses, start=1):
            if rospy.is_shutdown():
                break

            try:
                p_base = self.pose_to_base_link(p, src_frame, stamp)
            except Exception as e:
                rospy.logwarn("TF failed for waypoint %d: %s (skipping)", i, str(e))
                continue

            x = p_base.pose.position.x
            y = p_base.pose.position.y
            z = p_base.pose.position.z
            target_xyz = np.array([x, y, z], dtype=float)

            # 姿勢も一応 base_link へ（PoseStampedで変換済み）
            qx = p_base.pose.orientation.x
            qy = p_base.pose.orientation.y
            qz = p_base.pose.orientation.z
            qw = p_base.pose.orientation.w
            if (qx, qy, qz, qw) == (0.0, 0.0, 0.0, 0.0):
                qw = 1.0

            rospy.loginfo("Waypoint %d/%d (base_link): x=%.3f y=%.3f z=%.3f",
                          i, len(poses), x, y, z)

            sol = self.ik_solver.get_ik(seed, x, y, z, qx, qy, qz, qw)
            if sol is None:
                rospy.logwarn("IK failed at waypoint %d (base_link). Skipping.", i)
                continue

            sol_np = np.array(sol, dtype=float)
            self.send_joint_command(sol_np)
            seed = sol_np  # update seed

            reached, err = self.wait_until_reached(target_xyz)
            if reached:
                rospy.loginfo("Reached waypoint %d (FK err=%.4f m).", i, err)
            else:
                rospy.logwarn("Timeout waiting waypoint %d (FK err=%.4f m).", i, err)

            rospy.sleep(self.settle_time)

        rospy.loginfo("Done.")
        rospy.spin()


if __name__ == "__main__":
    try:
        ME6WaypointsIKControllerTF().run()
    except rospy.ROSInterruptException:
        pass
