#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from math import pi

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, TransformStamped

from std_msgs.msg import Empty, Int32, Float64

import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParam

import tf2_ros
import tf2_geometry_msgs  # noqa

from urdf_parser_py.urdf import URDF


class ME6WaypointsIKControllerTF_KDL_NRJL:
    """
    KDL IK (ChainIkSolverPos_NR_JL) version:
      - IK: Newton-Raphson + joint limits (NR_JL)
      - velocity solver: pseudoinverse Jacobian
      - orientation target: "current EE orientation" at solve time (so effectively position-following)
    """

    def __init__(self):
        rospy.init_node("me6_waypoints_ik_controller_tf_kdl_nrjl")

        # Params
        self.base_link = rospy.get_param("~base_link", "base_link")
        self.ee_link   = rospy.get_param("~ee_link", "Link6")
        self.way_topic = rospy.get_param("~waypoints_topic", "/way_points")
        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/me6_robot/joint_states")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/me6_robot/joint_controller/command")

        self.point_duration = float(rospy.get_param("~point_duration", 2.0))
        self.settle_time    = float(rospy.get_param("~settle_time", 0.3))
        self.pos_tol        = float(rospy.get_param("~pos_tol", 0.01))     # meters
        self.max_wait_fk    = float(rospy.get_param("~max_wait_fk", 3.0))  # seconds

        self.tf_timeout     = float(rospy.get_param("~tf_timeout", 0.5))   # seconds
        self.tf_use_latest  = bool(rospy.get_param("~tf_use_latest", True))  # ★ NEW

        # -----------------------------
        # KDL-NR(JL) params
        # -----------------------------
        self.kdl_eps = float(rospy.get_param("~kdl_eps", 1e-4))         # convergence threshold
        self.kdl_maxiter = int(rospy.get_param("~kdl_maxiter", 80))     # max NR iterations
        self.kdl_use_current_orientation = bool(
            rospy.get_param("~kdl_use_current_orientation", True)
        )

        self.joint_names = rospy.get_param("~joint_names", [
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6"
        ])

        # -----------------------------
        # EE pose publisher params
        # -----------------------------
        self.ee_pose_topic  = rospy.get_param("~ee_pose_topic",  "/me6_robot/ee_pose")
        self.ee_point_topic = rospy.get_param("~ee_point_topic", "/me6_robot/ee_point")
        self.publish_ee_tf   = bool(rospy.get_param("~publish_ee_tf", False))
        self.ee_tf_child     = rospy.get_param("~ee_tf_child", "me6_ee")

        # -----------------------------
        # ★ progress / done / error topics (NEW)
        # -----------------------------
        self.done_topic   = rospy.get_param("~done_topic", "/following_waypoints/done")
        self.wp_index_topic = rospy.get_param("~wp_index_topic", "/following_waypoints/wp_index")
        self.fk_err_topic = rospy.get_param("~fk_err_topic", "/following_waypoints/fk_err")
        self.target_point_topic = rospy.get_param("~target_point_topic", "/following_waypoints/target_point")

        # -----------------------------
        # debug / retry params
        # -----------------------------
        self.ik_debug = bool(rospy.get_param("~ik_debug", True))
        self.ik_retry = int(rospy.get_param("~ik_retry", 3))
        self.ik_retry_noise = float(rospy.get_param("~ik_retry_noise", 0.15))  # [rad]
        self.ik_retry_use_current = bool(rospy.get_param("~ik_retry_use_current", True))

        # Load URDF
        rospy.loginfo("Loading URDF from /robot_description...")
        ok, tree = treeFromParam("/robot_description")
        if not ok:
            raise RuntimeError("Failed to parse /robot_description")

        # Joint limits (from URDF)
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

        # -----------------------------
        # KDL IK solver (NR_JL)
        # -----------------------------
        q_min = kdl.JntArray(len(self.joint_names))
        q_max = kdl.JntArray(len(self.joint_names))
        for i, jn in enumerate(self.joint_names):
            lim = self.joint_limits.get(jn, None)
            if lim is None:
                q_min[i] = -1e9
                q_max[i] = +1e9
            else:
                q_min[i], q_max[i] = lim

        self.ik_vel_solver = kdl.ChainIkSolverVel_pinv(self.chain)
        self.ik_solver = kdl.ChainIkSolverPos_NR_JL(
            self.chain,
            q_min, q_max,
            self.fk_solver,
            self.ik_vel_solver,
            self.kdl_maxiter,
            self.kdl_eps
        )

        rospy.loginfo("Initialized KDL IK: ChainIkSolverPos_NR_JL (maxiter=%d eps=%.2e)",
                      self.kdl_maxiter, self.kdl_eps)

        # TF buffer (for waypoint frame transform)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # TF broadcaster (optional)
        self.ee_tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_ee_tf else None

        # State
        self.current_joints = None
        self.waypoints_msg = None

        rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_cb, queue_size=1)
        rospy.Subscriber(self.way_topic, PoseArray, self.waypoints_cb, queue_size=1)

        self.cmd_pub = rospy.Publisher(self.cmd_topic, JointTrajectory, queue_size=1)

        # EE publishers
        self.ee_pose_pub  = rospy.Publisher(self.ee_pose_topic, PoseStamped, queue_size=10)
        self.ee_point_pub = rospy.Publisher(self.ee_point_topic, PointStamped, queue_size=10)

        # ★ NEW pubs
        self.done_pub = rospy.Publisher(self.done_topic, Empty, queue_size=1, latch=True)
        self.wp_index_pub = rospy.Publisher(self.wp_index_topic, Int32, queue_size=10)
        self.fk_err_pub = rospy.Publisher(self.fk_err_topic, Float64, queue_size=50)
        self.target_point_pub = rospy.Publisher(self.target_point_topic, PointStamped, queue_size=10)

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
    # helpers
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

    def log_ik_failure(self, idx, src_frame, stamp, target_xyz, seed, status_code=None, p_base_pose=None):
        if not self.ik_debug:
            return

        try:
            ee = self.compute_fk_pos(self.current_joints)
            dist = float(np.linalg.norm(ee - target_xyz))
            ee_str = f"(FK) ee=[{ee[0]:+.3f},{ee[1]:+.3f},{ee[2]:+.3f}] dist={dist:.4f} m"
        except Exception as e:
            ee_str = f"(FK) unavailable: {e}"

        out = self._check_seed_limits(seed)
        out_str = "OK" if len(out) == 0 else ("OUT_OF_LIMIT: " + " | ".join(out))

        sc = "None" if status_code is None else str(status_code)

        rospy.logwarn(
            "KDL-IK FAILED wp=%d src_frame=%s stamp=%.3f -> base_link=%s | "
            "target=[%+.3f,%+.3f,%+.3f] | %s | seed %s | seed_limit=%s | "
            "status=%s maxiter=%d eps=%.2e",
            idx, src_frame, stamp.to_sec(), self.base_link,
            target_xyz[0], target_xyz[1], target_xyz[2],
            ee_str, self._fmt_joints(seed), out_str,
            sc, int(self.kdl_maxiter), float(self.kdl_eps)
        )

    # =========================================================
    # EE publish
    # =========================================================
    def publish_ee_fk(self):
        if self.current_joints is None:
            return

        frame = self.compute_fk_frame(self.current_joints)
        now = rospy.Time.now()

        px, py, pz = frame.p[0], frame.p[1], frame.p[2]
        qx, qy, qz, qw = frame.M.GetQuaternion()

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
    # publish current target point (base_link)
    # =========================================================
    def publish_target_point(self, x, y, z):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.base_link
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        self.target_point_pub.publish(msg)

    # =========================================================
    # Wait until reached (publish FK error continuously)
    # =========================================================
    def wait_until_reached(self, target_xyz_base):
        t0 = rospy.Time.now()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.publish_ee_fk()
            ee = self.compute_fk_pos(self.current_joints)
            err = float(np.linalg.norm(ee - target_xyz_base))

            # ★ publish error norm
            self.fk_err_pub.publish(Float64(data=err))

            if err <= self.pos_tol:
                return True, err
            if (rospy.Time.now() - t0).to_sec() > self.max_wait_fk:
                return False, err
            rate.sleep()

    # =========================================================
    # TF: waypoint pose -> base_link
    # =========================================================
    def pose_to_base_link(self, pose, src_frame, stamp):
        ps = PoseStamped()
        ps.header.frame_id = src_frame
        ps.header.stamp = rospy.Time(0) if self.tf_use_latest else stamp
        ps.pose = pose

        trans = self.tf_buffer.lookup_transform(
            self.base_link,
            src_frame,
            ps.header.stamp,
            rospy.Duration(self.tf_timeout)
        )
        ps_base = tf2_geometry_msgs.do_transform_pose(ps, trans)
        return ps_base

    # =========================================================
    # KDL IK solve
    # =========================================================
    def solve_kdl_ik(self, seed_np, target_xyz):
        q_seed = kdl.JntArray(len(seed_np))
        for i, v in enumerate(seed_np):
            q_seed[i] = float(v)

        # orientation target
        if self.kdl_use_current_orientation and self.current_joints is not None:
            cur_frame = self.compute_fk_frame(self.current_joints)
            R = cur_frame.M
        else:
            R = kdl.Rotation.Identity()

        target_frame = kdl.Frame(
            R,
            kdl.Vector(float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2]))
        )

        q_out = kdl.JntArray(len(seed_np))
        status = self.ik_solver.CartToJnt(q_seed, target_frame, q_out)

        if status == 0:
            sol = np.array([q_out[i] for i in range(len(seed_np))], dtype=float)
            return sol, status
        return None, status

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

        rospy.loginfo("Following %d waypoints (src_frame=%s -> base_link=%s) using KDL NR_JL...",
                      len(poses), src_frame, self.base_link)

        seed = np.copy(self.current_joints)

        for i, p in enumerate(poses, start=1):
            if rospy.is_shutdown():
                break

            # ★ publish progress
            self.wp_index_pub.publish(Int32(data=i))

            try:
                p_base = self.pose_to_base_link(p, src_frame, stamp)
            except Exception as e:
                rospy.logwarn("TF failed for waypoint %d: %s (skipping)", i, str(e))
                continue

            x = p_base.pose.position.x
            y = p_base.pose.position.y
            z = p_base.pose.position.z
            target_xyz = np.array([x, y, z], dtype=float)

            # ★ publish target
            self.publish_target_point(x, y, z)

            reach = float(np.linalg.norm(target_xyz))
            rospy.logwarn("target norm=%.3f m", reach)
            rospy.loginfo("Waypoint %d/%d (base_link): x=%.3f y=%.3f z=%.3f",
                          i, len(poses), x, y, z)

            sol, status = self.solve_kdl_ik(seed, target_xyz)

            if sol is None:
                self.log_ik_failure(i, src_frame, stamp, target_xyz, seed, status_code=status, p_base_pose=p_base.pose)

                # retries (seed variations)
                tried = 0
                cand_seeds = [np.copy(seed)]
                if self.ik_retry_use_current and self.current_joints is not None:
                    cand_seeds.append(np.copy(self.current_joints))

                for _ in range(int(self.ik_retry)):
                    s = np.copy(seed)
                    s += np.random.uniform(-self.ik_retry_noise, self.ik_retry_noise, size=s.shape)
                    for jj, jn in enumerate(self.joint_names):
                        lim = self.joint_limits.get(jn, None)
                        if lim is not None:
                            lo, hi = lim
                            s[jj] = np.clip(s[jj], lo, hi)
                    cand_seeds.append(s)

                recovered = False
                for sidx, s in enumerate(cand_seeds):
                    tried += 1
                    sol2, st2 = self.solve_kdl_ik(s, target_xyz)
                    if sol2 is not None:
                        rospy.logwarn("KDL-IK RECOVERED wp=%d: success on retry #%d (seed=%s)",
                                      i, sidx, self._fmt_joints(s))
                        sol = sol2
                        recovered = True
                        break

                if not recovered:
                    rospy.logwarn("KDL-IK still failed wp=%d after %d tries. Skipping.", i, tried)
                    continue

            # command + update seed
            self.send_joint_command(sol)
            seed = np.copy(sol)

            reached, err = self.wait_until_reached(target_xyz)
            if reached:
                rospy.loginfo("Reached waypoint %d (FK err=%.4f m).", i, err)
            else:
                rospy.logwarn("Timeout waiting waypoint %d (FK err=%.4f m).", i, err)

            rospy.sleep(self.settle_time)

        # ★ DONE 合図（bag後処理で xlim 終端に使える）
        rospy.loginfo("Done.")
        self.wp_index_pub.publish(Int32(data=-1))
        self.done_pub.publish(Empty())

        rospy.spin()


if __name__ == "__main__":
    try:
        ME6WaypointsIKControllerTF_KDL_NRJL().run()
    except rospy.ROSInterruptException:
        pass
