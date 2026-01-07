#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from math import pi  # ★追加

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

        # ★IKの許容誤差（TRAC-IK bounds）
        self.ik_pos_bound = rospy.get_param("~ik_pos_bound", 5e-3)  # [m] 例: 5mm
        self.ik_rot_bound = rospy.get_param("~ik_rot_bound", pi)    # [rad] piで姿勢ほぼ自由

        # ★TRAC-IK timeout
        self.ik_timeout = rospy.get_param("~ik_timeout", 0.05)      # [s]
        self.ik_solve_type = rospy.get_param("~ik_solve_type", "Distance")

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
        # ★timeout/solve_type 指定
        self.ik_solver = IK(self.base_link, self.ee_link, timeout=self.ik_timeout, solve_type=self.ik_solve_type)

        # ★デバッグ（初期化確認）
        try:
            rospy.loginfo("TRAC-IK number_of_joints: %d", self.ik_solver.number_of_joints)
            rospy.loginfo("TRAC-IK joint_names: %s", self.ik_solver.joint_names)
            rospy.loginfo("TRAC-IK link_names: %s", self.ik_solver.link_names)
        except Exception:
            rospy.logwarn("TRAC-IK debug fields not available (ok).")

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
        ps = PoseStamped()
        ps.header.frame_id = src_frame
        ps.header.stamp = stamp
        ps.pose = pose

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

        # ★位置だけ合わせたいので姿勢は固定（単位クォータニオン）
        #   姿勢拘束を緩めるのでこれでOK
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

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

            rospy.loginfo("Waypoint %d/%d (base_link): x=%.3f y=%.3f z=%.3f",
                          i, len(poses), x, y, z)

            # ★bounds付き（位置は5mm許容、姿勢はほぼ自由）
            b = float(self.ik_pos_bound)
            br = float(self.ik_rot_bound)

            sol = self.ik_solver.get_ik(
                seed,
                x, y, z,
                qx, qy, qz, qw,
                b, b, b,      # bx,by,bz [m]
                br, br, br    # brx,bry,brz [rad]
            )

            if sol is None:
                rospy.logwarn("IK failed at waypoint %d (base_link). Skipping.", i)
                continue

            sol_np = np.array(sol, dtype=float)
            self.send_joint_command(sol_np)
            seed = sol_np  # update seed (次のIKが安定)

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
