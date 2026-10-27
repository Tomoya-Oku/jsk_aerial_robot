#!/usr/bin/env python3
"""Numerical contact/mapping checks; runnable without ROS or hardware."""

import copy
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

CONTROL = Path(__file__).resolve().parents[1] / "scripts" / "control"
sys.path.insert(0, str(CONTROL))
from contact_math import finite_jacobian, reflect_wrench, shift_wrench
from contact_model import ContactModel


def load_controller():
    # Only import interfaces are stubbed. Mapping methods execute unchanged.
    names = ["rospy", "tf2_ros", "tf", "tf.transformations", "std_msgs",
             "std_msgs.msg", "sensor_msgs", "sensor_msgs.msg", "geometry_msgs",
             "geometry_msgs.msg", "nav_msgs", "nav_msgs.msg", "aerial_robot_msgs",
             "aerial_robot_msgs.msg", "dracomancer", "dracomancer.srv"]
    spec = importlib.util.spec_from_file_location("joint_mapping", CONTROL / "control_joint_angle.py")
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {name: MagicMock() for name in names}):
        spec.loader.exec_module(module)
    return module.ControlJoints


class ContactMathTest(unittest.TestCase):
    def test_virtual_work_and_limits(self):
        rng = np.random.default_rng(12)
        mapping, robot_jacobian = rng.normal(size=(6, 7)), rng.normal(size=(6, 6))
        wrench, dq = rng.normal(size=6), rng.normal(size=7)
        raw, _ = reflect_wrench(mapping, robot_jacobian, wrench, 0.1, np.ones(7))
        self.assertAlmostEqual(raw @ dq, 0.1 * wrench @ (robot_jacobian @ mapping @ dq))
        _, limited = reflect_wrench(mapping, robot_jacobian, wrench * 1e5, 0.1, np.full(7, 0.2))
        self.assertTrue(np.all(np.abs(limited) <= 0.2))

    def test_force_moment_reference(self):
        result = shift_wrench([0, 2, 0], [0, 0, 3], [1, 0, 0], [0, 0, 0])
        np.testing.assert_allclose(result, [0, 2, 0, 0, 0, 5])
        np.testing.assert_allclose(
            shift_wrench(result[:3], result[3:], [0, 0, 0], [1, 0, 0]),
            [0, 2, 0, 0, 0, 3])

    def test_invalid_inputs_fail_closed(self):
        with self.assertRaises(ValueError):
            finite_jacobian(lambda x: x, [np.nan])
        for wrench in ([0] * 5, [0, 0, 0, 0, 0, np.inf]):
            with self.assertRaises(ValueError):
                reflect_wrench(np.zeros((6, 7)), np.eye(6), wrench, 0.1, np.full(7, 0.2))


class ExistingMappingTest(unittest.TestCase):
    def setUp(self):
        self.cls = load_controller()
        self.node = self.cls.__new__(self.cls)
        n = self.node
        self.names = ["shoulder_abduction_adduction_joint", "shoulder_flexion_extension_joint",
                      "upper_arm_external_internal_rotation_joint", "elbow_flexion_extension_joint",
                      "wrist_supination_joint", "wrist_flexion_extension_joint",
                      "wrist_abduction_adduction_joint"]
        n.mapping_mode = "distal"
        n.capture_roll_neutral, n.want_capture_roll_neutral = True, False
        n.roll_neutral = {self.names[2]: 0.0, self.names[4]: 0.0}
        n.joint_names = ["joint%d_%s" % (j, axis) for j in (1, 2, 3) for axis in ("pitch", "yaw")]
        n.joint_index = {name: i for i, name in enumerate(n.joint_names)}
        n.joint_limit, n.last_feasible_target = np.pi / 2, [0.0] * 6
        n.distal_map = [(self.names[source], n.joint_names[target], sign, 1.0, 0.0)
                        for source, target, sign in ((5, 0, 1), (6, 1, 1), (3, 3, 1),
                                                     (1, 4, 1), (0, 5, -1))]
        n.switch_diag = {}
        for label, source, rolls, index in (("elbow", 3, self.names[2], 2),
                                             ("wrist", 5, [self.names[2], self.names[4]], 1)):
            setattr(n, "enable_" + label + "_roll_switching", True)
            setattr(n, label + "_source_joint", self.names[source])
            setattr(n, label + "_pitch_target_joint", "joint%d_pitch" % index)
            setattr(n, label + "_yaw_target_joint", "joint%d_yaw" % index)
            setattr(n, label + ("_roll_joint" if label == "elbow" else "_roll_joints"), rolls)
            setattr(n, label + ("_roll_offset" if label == "elbow" else "_roll_offsets"),
                    0.0 if label == "elbow" else [0.0, 0.0])
            for key in ("pitch_sign", "yaw_sign", "pitch_scale", "yaw_scale"):
                setattr(n, label + "_" + key, 1.0)
            for key in ("pitch_offset", "yaw_offset"):
                setattr(n, label + "_" + key, 0.0)
        n.wrist_yaw_source_joint = self.names[6]
        n.wrist_yaw_source_sign = n.wrist_yaw_source_scale = 1.0

    def evaluate(self, q):
        self.node.latest_device_joints = dict(zip(self.names, q))
        return self.node.mapped_target()

    def test_all_seven_channels_and_roll_coupling(self):
        q = np.array([0.1, 0.2, 0.25, 0.5, -0.1, 0.3, 0.12])
        jac = finite_jacobian(self.evaluate, q)
        angle = q[2] + q[4]
        c, s = np.cos(angle), np.sin(angle)
        expected = np.zeros((6, 7))
        expected[0, 5], expected[0, 6] = c, -s
        expected[1, 5], expected[1, 6] = s, c
        expected[0, 2] = expected[0, 4] = -s * q[5] - c * q[6]
        expected[1, 2] = expected[1, 4] = c * q[5] - s * q[6]
        expected[2, 2], expected[2, 3] = q[3] * 2 / np.pi, q[2] * 2 / np.pi
        expected[3, 2], expected[3, 3] = -q[3] * 2 / np.pi, 1 - q[2] * 2 / np.pi
        expected[4, 1], expected[5, 0] = 1, -1
        np.testing.assert_allclose(jac, expected, atol=1e-8)
        self.assertTrue(np.all(np.linalg.norm(jac, axis=0) > 0))

    def test_saturation_and_frozen_neutral(self):
        q = np.array([0.1, 2.0, 2.0, 0.5, 0.0, 0.3, 0.12])
        neutral = copy.deepcopy(self.node.roll_neutral)
        jac = finite_jacobian(self.evaluate, q)
        np.testing.assert_allclose(jac[:, 1:3], 0, atol=1e-8)
        self.assertEqual(self.node.roll_neutral, neutral)


class MeasurementTimeTest(unittest.TestCase):
    def setUp(self):
        path = CONTROL.parent / "device_io" / "convert_servo_to_joint_states.py"
        spec = importlib.util.spec_from_file_location("servo_input", path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {name: MagicMock() for name in
                                     ("rospy", "sensor_msgs", "sensor_msgs.msg", "spinal", "spinal.msg")}):
            spec.loader.exec_module(module)
        module.rospy.Time = lambda value: value
        module.JointState = lambda: SimpleNamespace(header=SimpleNamespace(stamp=None))
        self.node = module.ServoToJointStates.__new__(module.ServoToJointStates)
        n = self.node
        n.ordered_ids = list(range(7))
        n.id_to_joint_name = {i: "joint%d" % i for i in n.ordered_ids}
        n.latest_servo_pos, n.last_servo_stamp = {}, None
        n.center_tick, n.ticks_per_rev, n.offsets = 2048, 4096, {}

    def test_cached_angles_do_not_refresh_measurement_time(self):
        n = self.node
        n.latest_servo_pos = {i: 2048 for i in range(7)}
        n.last_servo_stamp = 17
        self.assertEqual(n.make_joint_msg().header.stamp, 17)
        self.assertEqual(n.make_joint_msg().header.stamp, 17)

    def test_missing_axes_have_invalid_timestamp(self):
        self.assertEqual(self.node.make_joint_msg().header.stamp, 0)
        self.node.latest_servo_pos = {i: 2048 for i in range(6)}
        self.node.last_servo_stamp = 17
        msg = self.node.make_joint_msg()
        self.assertEqual(msg.header.stamp, 0)
        self.assertEqual(len(msg.position), 7)

    def test_can_measurement_stamp_survives_receipt(self):
        samples = [SimpleNamespace(index=i, angle=2048) for i in range(7)]
        self.node.servo_cb(SimpleNamespace(stamp=11, servos=samples))
        self.assertEqual(self.node.make_joint_msg().header.stamp, 11)


class CogKinematicsTest(unittest.TestCase):
    def setUp(self):
        self.model = ContactModel('''<robot name="test">
          <link name="fc"><inertial><mass value="1"/></inertial></link>
          <link name="link1"><inertial><mass value="1"/><origin xyz="1 0 0"/></inertial></link>
          <joint name="bend" type="revolute"><parent link="fc"/><child link="link1"/>
            <origin xyz="1 0 0"/><axis xyz="0 0 1"/></joint></robot>''')

    def test_mass_motion_and_spatial_rotation(self):
        jacobian = self.model.jacobian({"bend": 0}, ["bend"])
        np.testing.assert_allclose(jacobian[:, 0], [0, -0.5, 0, 0, 0, 1], atol=1e-8)
        # 2 N in +Y and 3 Nm around Z, about the tip, yield 2 Nm.
        self.assertAlmostEqual(float(jacobian[:, 0] @ [0, 2, 0, 0, 0, 3]), 2)

    def test_required_joints_and_frame_rotation(self):
        with self.assertRaises(ValueError):
            self.model.jacobian({}, ["bend"])
        wrench = self.model.wrench_in_fc({"bend": np.pi / 2}, np.array([1, 0, 0, 0, 1, 0]))
        np.testing.assert_allclose(wrench, [0, 1, 0, -1, 0, 0], atol=1e-8)


if __name__ == "__main__":
    unittest.main()
