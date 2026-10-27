#!/usr/bin/env python3
"""Isolated ODE contact sign/frame test. Use a fresh Gazebo, not a flight session."""
from pathlib import Path
import sys
import time
import unittest
import numpy as np
import rospy
from gazebo_msgs.srv import SpawnModel, ApplyBodyWrench, BodyRequest, GetLinkState
from geometry_msgs.msg import Pose, Point, Wrench, WrenchStamped
from std_srvs.srv import Empty

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts" / "control"))
from wall import wall_sdf


class GazeboContactTest(unittest.TestCase):
    def test_head_force_frame_moment_and_pause(self):
        rospy.init_node("contact_physics_test")
        rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=40)
        samples = []
        sub = rospy.Subscriber("/dragon/contact/wrench", WrenchStamped,
                               lambda msg: samples.append(msg), queue_size=100)
        pose = Pose()
        pose.orientation.w = 1
        spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.assertTrue(spawn("contact_wall", wall_sdf([1.5, 0, 1.5], [0.1, 3, 3], 0.8), "", pose, "world").success)
        # Head local -X points toward world +X. CoM is offset by 2 cm in Y
        # so checking the moment also detects a missing CoM-to-origin shift.
        # Match DRAGON's fixed-joint reduction, with an extra offset/rotation
        # to catch accidental assumptions that the root and tip frames coincide.
        robot = '''<robot name="dragon">
          <link name="root"><inertial><mass value="0.0001"/>
            <inertia ixx="0.00001" iyy="0.00001" izz="0.00001" ixy="0" ixz="0" iyz="0"/></inertial></link>
          <link name="link1"><inertial><mass value="1"/><origin xyz="0.237 0.02 0"/>
            <inertia ixx="0.02" iyy="0.02" izz="0.02" ixy="0" ixz="0" iyz="0"/></inertial>
          <collision><origin xyz="0.237 0 0"/><geometry><box size="0.474 0.15 0.1686"/></geometry></collision></link>
          <joint name="root_to_head" type="fixed"><parent link="root"/><child link="link1"/>
            <origin xyz="0.13 0.04 0.01" rpy="0 0 0.2"/></joint>
          <gazebo reference="root"><gravity>false</gravity></gazebo>
          <gazebo reference="link1"><gravity>false</gravity></gazebo></robot>'''
        yaw = np.pi - 0.2
        c, s = np.cos(yaw), np.sin(yaw)
        offset = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ [0.13, 0.04, 0.01]
        pose.position.x, pose.position.y, pose.position.z = np.array([1.43, 0, 1.0]) - offset
        pose.orientation.w, pose.orientation.z = np.cos(yaw / 2), np.sin(yaw / 2)
        spawn_urdf = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.assertTrue(spawn_urdf("dragon", robot, "", pose, "world").success)
        rospy.wait_for_service("/gazebo/apply_body_wrench", timeout=5)
        wrench = Wrench()
        wrench.force.x = 1
        result = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)(
            "dragon::root", "world", Point(), wrench,
            rospy.Time(0), rospy.Duration(20))
        self.assertTrue(result.success)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if len(samples) > 200 and samples[-1].wrench.force.x > 0.5:
                time.sleep(0.5)
                break
            time.sleep(0.02)
        self.assertGreater(len(samples), 200, "plugin did not publish contact samples")
        window = samples[-200:]
        forces = np.array([[m.wrench.force.x, m.wrench.force.y, m.wrench.force.z] for m in window])
        torques = np.array([[m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z] for m in window])
        self.assertEqual(window[-1].header.frame_id, "dragon/link1")
        state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)("dragon::root", "world")
        self.assertAlmostEqual(float(forces[:, 0].mean()), 1.0, delta=0.15,
                               msg="max force={} final link={}".format(
                                   max(abs(m.wrench.force.x) for m in samples), state.link_state))
        self.assertAlmostEqual(float(torques[:, 2].mean()), -0.02, delta=0.01)
        rospy.ServiceProxy("/gazebo/pause_physics", Empty)()
        time.sleep(0.05)
        stamp = samples[-1].header.stamp
        time.sleep(0.15)
        self.assertEqual(samples[-1].header.stamp, stamp, "paused contact data advanced")
        rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
        rospy.ServiceProxy("/gazebo/clear_body_wrenches", BodyRequest)("dragon::root")
        sub.unregister()


if __name__ == "__main__":
    unittest.main()
