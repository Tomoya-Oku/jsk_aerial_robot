#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParam
from trac_ik_python.trac_ik import IK
from tf.transformations import quaternion_from_euler


class ME6SimpleIKController:
    def __init__(self):
        rospy.init_node("me6_simple_ik_controller")

        rospy.loginfo("Loading URDF from /robot_description...")
        ok, tree = treeFromParam("/robot_description")
        if not ok:
            rospy.logerr("Failed to parse robot_description")
            rospy.signal_shutdown("URDF error")
            return

        # -----------------------------
        # Robot definition
        # -----------------------------
        self.base_link = "base_link"
        self.ee_link   = "Link6"   # end-effector link (tool flange)

        self.joint_names = [
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6"
        ]

        # -----------------------------
        # KDL solvers
        # -----------------------------
        self.chain = tree.getChain(self.base_link, self.ee_link)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)

        # TRAC-IK solver
        self.ik_solver = IK(self.base_link, self.ee_link)

        # -----------------------------
        # State
        # -----------------------------
        self.current_joints = None

        rospy.Subscriber(
            "/me6_robot/joint_states",
            JointState,
            self.joint_state_cb,
            queue_size=1
        )

        self.cmd_pub = rospy.Publisher(
            "/me6_robot/joint_controller/command",
            JointTrajectory,
            queue_size=1
        )

        rospy.loginfo("ME6 IK Controller Ready")
        rospy.loginfo("Waiting for joint states...")

        while not rospy.is_shutdown() and self.current_joints is None:
            rospy.sleep(0.1)

        rospy.loginfo("Joint states received")

        # -----------------------------
        # End-effector working range
        # (meters, base_link frame)
        # -----------------------------
        self.workspace = {
            "x": (-0.40, 0.40),
            "y": (-0.40, 0.40),
            "z": ( 0.05, 0.75)
        }

    # =========================================================
    # Callbacks
    # =========================================================
    def joint_state_cb(self, msg: JointState):
        joint_map = dict(zip(msg.name, msg.position))
        self.current_joints = np.array([
            joint_map.get(j, 0.0) for j in self.joint_names
        ])

    # =========================================================
    # FK
    # =========================================================
    def compute_fk(self, joints):
        q = kdl.JntArray(len(joints))
        for i, v in enumerate(joints):
            q[i] = v

        frame = kdl.Frame()
        self.fk_solver.JntToCart(q, frame)

        pos = np.array([frame.p[0], frame.p[1], frame.p[2]])
        return pos

    # =========================================================
    # Command
    # =========================================================
    def send_joint_command(self, joints):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joints.tolist()
        point.time_from_start = rospy.Duration(2.0)

        traj.points.append(point)
        self.cmd_pub.publish(traj)

    # =========================================================
    # Workspace check
    # =========================================================
    def within_workspace(self, x, y, z):
        wx, wy, wz = self.workspace["x"], self.workspace["y"], self.workspace["z"]
        return (wx[0] <= x <= wx[1] and
                wy[0] <= y <= wy[1] and
                wz[0] <= z <= wz[1])

    # =========================================================
    # Main loop
    # =========================================================
    def run(self):
        while not rospy.is_shutdown():

            # --- FK feedback ---
            ee_pos = self.compute_fk(self.current_joints)

            print("\n======================================")
            print("Current end-effector position (base_link)")
            print(f"  x = {ee_pos[0]:.3f}")
            print(f"  y = {ee_pos[1]:.3f}")
            print(f"  z = {ee_pos[2]:.3f}")

            print("\nAllowed workspace:")
            print(f"  x ∈ [{self.workspace['x'][0]}, {self.workspace['x'][1]}]")
            print(f"  y ∈ [{self.workspace['y'][0]}, {self.workspace['y'][1]}]")
            print(f"  z ∈ [{self.workspace['z'][0]}, {self.workspace['z'][1]}]")

            # --- User input ---
            try:
                x = float(input("\nTarget x: "))
                y = float(input("Target y: "))
                z = float(input("Target z: "))
            except ValueError:
                print("Invalid input (must be numbers)")
                continue

            if not self.within_workspace(x, y, z):
                print("❌ Target outside workspace — IK skipped")
                continue

            # --- Fixed orientation (tool pointing forward) ---
            qx, qy, qz, qw = quaternion_from_euler(0, 0, 0)

            sol = self.ik_solver.get_ik(
                self.current_joints,
                x, y, z,
                qx, qy, qz, qw
            )

            if sol is None:
                print("❌ IK failed (no solution found)")
                continue

            print("\n✅ IK solution:")
            for name, val in zip(self.joint_names, sol):
                print(f"  {name}: {val:.3f} rad")

            self.send_joint_command(np.array(sol))
            rospy.sleep(2.5)


if __name__ == "__main__":
    try:
        ME6SimpleIKController().run()
    except rospy.ROSInterruptException:
        pass
