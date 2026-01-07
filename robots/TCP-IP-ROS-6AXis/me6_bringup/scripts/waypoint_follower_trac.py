#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from math import pi

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, TransformStamped

import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParam
from trac_ik_python.trac_ik import IK

import tf2_ros
import tf2_geometry_msgs  # noqa

from urdf_parser_py.urdf import URDF


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

        # IK bounds (TRAC-IK)
        self.ik_pos_bound = rospy.get_param("~ik_pos_bound", 2e-2)  # [m] e.g. 5mm
        self.ik_rot_bound = rospy.get_param("~ik_rot_bound", pi)    # [rad] pi -> almost free orientation
        self.ik_timeout   = rospy.get_param("~ik_timeout", 0.5)    # [s]
        self.ik_solve_type = rospy.get_param("~ik_solve_type", "Distance")

        self.joint_names = rospy.get_param("~joint_names", [
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6"
        ])

        # -----------------------------
        # ★ EE pose publisher params
        # -----------------------------
        self.ee_pose_topic  = rospy.get_param("~ee_pose_topic",  "/me6_robot/ee_pose")
        self.ee_point_topic = rospy.get_param("~ee_point_topic", "/me6_robot/ee_point")
        self.publish_ee_tf   = rospy.get_param("~publish_ee_tf", False)
        self.ee_tf_child     = rospy.get_param("~ee_tf_child", "me6_ee")

        # -----------------------------
        # ★ IK debug params（ログ詳細 + リトライ）
        # -----------------------------
        self.ik_debug = rospy.get_param("~ik_debug", True)
        self.ik_retry = rospy.get_param("~ik_retry", 3)                 # 失敗時リトライ回数
        self.ik_retry_noise = rospy.get_param("~ik_retry_noise", 0.15)  # [rad] seedに足すノイズ
        self.ik_retry_use_current = rospy.get_param("~ik_retry_use_current", True)

        # Load URDF
        rospy.loginfo("Loading URDF from /robot_description...")
        ok, tree = treeFromParam("/robot_description")
        if not ok:
            raise RuntimeError("Failed to parse robot_description")

        # ★ Joint limits (from URDF)
        self.urdf = URDF.from_parameter_server("/robot_description")
        self.joint_limits = {}
        for jn in self.joint_names:
            j = self.urdf.joint_map.get(jn, None)
            if j is None or j.limit is None:
                self.joint_limits[jn] = None
            else:
                self.joint_limits[jn] = (float(j.limit.lower), float(j.limit.upper))

        # KDL chain + FK
        self.chain = tree.getChain(self.base_link, self.ee_link)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)

        # TRAC-IK
        rospy.loginfo("Initializing TRAC-IK solver (%s -> %s)...", self.base_link, self.ee_link)
        self.ik_solver = IK(self.base_link, self.ee_link, timeout=self.ik_timeout, solve_type=self.ik_solve_type)

        # TF buffer (for waypoint frame transform)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ★ TF broadcaster (optional)
        self.ee_tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_ee_tf else None

        # State
        self.current_joints = None
        self.waypoints_msg = None

        rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_cb, queue_size=1)
        rospy.Subscriber(self.way_topic, PoseArray, self.waypoints_cb, queue_size=1)

        self.cmd_pub = rospy.Publisher(self.cmd_topic, JointTrajectory, queue_size=1)

        # ★ EE publishers
        self.ee_pose_pub  = rospy.Publisher(self.ee_pose_topic, PoseStamped, queue_size=10)
        self.ee_point_pub = rospy.Publisher(self.ee_point_topic, PointStamped, queue_size=10)

        rospy.loginfo("Waiting for joint states and %s ...", self.way_topic)
        while not rospy.is_shutdown() and self.current_joints is None:
            rospy.sleep(0.05)
        while not rospy.is_shutdown() and self.waypoints_msg is None:
            rospy.sleep(0.05)

        rospy.loginfo("Ready. Starting...")

    # =========================================================
    # Callbacks
    # =========================================================
    def joint_state_cb(self, msg: JointState):
        joint_map = dict(zip(msg.name, msg.position))
        self.current_joints = np.array([joint_map.get(j, 0.0) for j in self.joint_names], dtype=float)

        # ★ joint_state受信のたびにEEをpublish（軽め）
        try:
            self.publish_ee_fk()
        except Exception:
            pass

    def waypoints_cb(self, msg: PoseArray):
        self.waypoints_msg = msg

    # =========================================================
    # FK (KDL Frame)
    # =========================================================
    def compute_fk_frame(self, joints_np):
        q = kdl.JntArray(len(joints_np))
        for i, v in enumerate(joints_np):
            q[i] = float(v)
        frame = kdl.Frame()
        self.fk_solver.JntToCart(q, frame)
        return frame

    def compute_fk_pos(self, joints_np):
        frame = self.compute_fk_frame(joints_np)
        return np.array([frame.p[0], frame.p[1], frame.p[2]], dtype=float)

    # =========================================================
    # ★ IK failure debug helpers
    # =========================================================
    def _fmt_joints(self, joints_np):
        deg = joints_np * 180.0 / pi
        s_rad = ", ".join([f"{v:+.3f}" for v in joints_np])
        s_deg = ", ".join([f"{v:+.1f}" for v in deg])
        return f"rad=[{s_rad}] deg=[{s_deg}]"

    def _check_seed_limits(self, joints_np):
        msgs = []
        for i, jn in enumerate(self.joint_names):
            lim = self.joint_limits.get(jn, None)
            if lim is None:
                continue
            lo, hi = lim
            v = float(joints_np[i])
            if v < lo or v > hi:
                msgs.append(f"{jn}={v:+.3f} (limit {lo:+.3f}..{hi:+.3f})")
        return msgs

    def log_ik_failure(self, idx, src_frame, stamp, target_xyz, seed, p_base_pose=None):
        if not self.ik_debug:
            return

        # 現在EE（FK）と距離
        try:
            ee = self.compute_fk_pos(self.current_joints)
            dist = float(np.linalg.norm(ee - target_xyz))
            ee_str = f"(FK) ee=[{ee[0]:+.3f},{ee[1]:+.3f},{ee[2]:+.3f}] dist={dist:.4f} m"
        except Exception as e:
            ee_str = f"(FK) unavailable: {e}"

        # seedのlimitチェック
        out = self._check_seed_limits(seed)
        out_str = "OK" if len(out) == 0 else ("OUT_OF_LIMIT: " + " | ".join(out))

        rospy.logwarn(
            "IK FAILED wp=%d src_frame=%s stamp=%.3f -> base_link=%s | "
            "target=[%+.3f,%+.3f,%+.3f] | %s | seed %s | seed_limit=%s | "
            "bounds(pos=%.4g m, rot=%.4g rad) timeout=%.3f solve_type=%s",
            idx,
            src_frame,
            stamp.to_sec(),
            self.base_link,
            target_xyz[0], target_xyz[1], target_xyz[2],
            ee_str,
            self._fmt_joints(seed),
            out_str,
            float(self.ik_pos_bound), float(self.ik_rot_bound),
            float(self.ik_timeout), str(self.ik_solve_type),
        )

        if p_base_pose is not None:
            rospy.logdebug(
                "wp=%d base_pose: pos=[%+.3f,%+.3f,%+.3f] quat=[%+.3f,%+.3f,%+.3f,%+.3f]",
                idx,
                p_base_pose.position.x, p_base_pose.position.y, p_base_pose.position.z,
                p_base_pose.orientation.x, p_base_pose.orientation.y,
                p_base_pose.orientation.z, p_base_pose.orientation.w
            )

    # =========================================================
    # ★ EE publish
    # =========================================================
    def publish_ee_fk(self):
        if self.current_joints is None:
            return

        frame = self.compute_fk_frame(self.current_joints)

        now = rospy.Time.now()
        px, py, pz = frame.p[0], frame.p[1], frame.p[2]

        # 今のEE姿勢を目標姿勢として使う
        cur_frame = self.compute_fk_frame(self.current_joints)
        qx, qy, qz, qw = cur_frame.M.GetQuaternion()

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
        self.ee_pose_pub.publish(ps)

        pt = PointStamped()
        pt.header.stamp = now
        pt.header.frame_id = self.base_link
        pt.point.x = px
        pt.point.y = py
        pt.point.z = pz
        self.ee_point_pub.publish(pt)

        if self.publish_ee_tf and self.ee_tf_broadcaster is not None:
            ts = TransformStamped()
            ts.header.stamp = now
            ts.header.frame_id = self.base_link
            ts.child_frame_id = self.ee_tf_child
            ts.transform.translation.x = px
            ts.transform.translation.y = py
            ts.transform.translation.z = pz
            ts.transform.rotation.x = qx
            ts.transform.rotation.y = qy
            ts.transform.rotation.z = qz
            ts.transform.rotation.w = qw
            self.ee_tf_broadcaster.sendTransform(ts)

    # =========================================================
    # Command
    # =========================================================
    def send_joint_command(self, joints_np):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = joints_np.tolist()
        pt.time_from_start = rospy.Duration(self.point_duration)
        traj.points.append(pt)
        self.cmd_pub.publish(traj)

    # =========================================================
    # Wait until reached
    # =========================================================
    def wait_until_reached(self, target_xyz_base):
        t0 = rospy.Time.now()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # ★待機中も publish して見えるように
            self.publish_ee_fk()

            ee = self.compute_fk_pos(self.current_joints)  # base_link
            err = np.linalg.norm(ee - target_xyz_base)
            if err <= self.pos_tol:
                return True, float(err)
            if (rospy.Time.now() - t0).to_sec() > self.max_wait_fk:
                return False, float(err)
            rate.sleep()

    # =========================================================
    # TF: waypoint pose -> base_link
    # =========================================================
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

    # =========================================================
    # Main
    # =========================================================
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

        # position-only (orientation almost free by bounds)
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

            reach = np.linalg.norm(target_xyz)
            rospy.logwarn("target norm=%.3f m", reach)

            rospy.loginfo("Waypoint %d/%d (base_link): x=%.3f y=%.3f z=%.3f",
                          i, len(poses), x, y, z)

            b = float(self.ik_pos_bound)
            br = float(self.ik_rot_bound)

            sol = self.ik_solver.get_ik(
                seed,
                x, y, z,
                qx, qy, qz, qw,
                b, b, b,
                br, br, br
            )

            if sol is None:
                # ★詳細ログ（状況証拠）
                self.log_ik_failure(i, src_frame, stamp, target_xyz, seed, p_base.pose)

                # ★追加リトライ（seed依存で落ちるケースの切り分け＆救済）
                tried = 0

                cand_seeds = []
                cand_seeds.append(np.copy(seed))
                if self.ik_retry_use_current and self.current_joints is not None:
                    cand_seeds.append(np.copy(self.current_joints))

                # ノイズseed
                for _ in range(int(self.ik_retry)):
                    s = np.copy(seed)
                    s += np.random.uniform(-self.ik_retry_noise, self.ik_retry_noise, size=s.shape)
                    # limitに収める（limitがある関節だけ）
                    for jj, jn in enumerate(self.joint_names):
                        lim = self.joint_limits.get(jn, None)
                        if lim is not None:
                            lo, hi = lim
                            s[jj] = np.clip(s[jj], lo, hi)
                    cand_seeds.append(s)

                for sidx, s in enumerate(cand_seeds):
                    tried += 1
                    sol2 = self.ik_solver.get_ik(
                        s,
                        x, y, z,
                        qx, qy, qz, qw,
                        b, b, b,
                        br, br, br
                    )
                    if sol2 is not None:
                        rospy.logwarn(
                            "IK RECOVERED wp=%d: success on retry #%d (seed=%s)",
                            i, sidx, self._fmt_joints(s)
                        )
                        sol = sol2
                        break

                if sol is None:
                    rospy.logwarn("IK still failed wp=%d after %d tries. Skipping.", i, tried)
                    continue

            sol_np = np.array(sol, dtype=float)
            self.send_joint_command(sol_np)
            seed = sol_np

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
