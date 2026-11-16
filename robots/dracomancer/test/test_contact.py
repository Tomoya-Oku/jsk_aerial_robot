#!/usr/bin/env python3
"""Numerical contact/mapping checks; runnable without ROS or hardware."""

from pathlib import Path
import sys
import unittest

import numpy as np

CONTROL = Path(__file__).resolve().parents[1] / "scripts" / "control"
sys.path.insert(0, str(CONTROL))
from contact_math import finite_jacobian, reflect_wrench, shift_wrench
from contact_model import ContactModel




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
