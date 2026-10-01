#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Unit tests for the ROS-independent Mk-II haptic models."""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from contact_feedback import ContactFeedback
from current_conversion import XM430CurrentConverter
from safety_feedback import SafetyFeedback, combine_torque, rate_limit


class ContactFeedbackTest(unittest.TestCase):
    def test_identity_mapping(self):
        model = ContactFeedback(6, 7, gain=0.5)
        robot_jacobian = np.eye(6)
        mapping = np.zeros((6, 7))
        mapping[:, :6] = np.eye(6)
        wrench = np.arange(1.0, 7.0)
        expected = np.r_[0.5 * wrench, 0.0]
        np.testing.assert_allclose(
            model.compute(wrench, robot_jacobian, mapping), expected)

    def test_invalid_jacobian_is_rejected(self):
        model = ContactFeedback(6, 7)
        with self.assertRaises(ValueError):
            model.compute(np.zeros(6), np.eye(5), np.zeros((6, 7)))


class SafetyFeedbackTest(unittest.TestCase):
    def test_resistance_opposes_shape_error(self):
        model = SafetyFeedback([2.0] * 6, [0.0] * 7)
        mapping = np.zeros((6, 7))
        mapping[:, :6] = np.eye(6)
        torque = model.compute(np.ones(6), mapping, np.zeros(7))
        np.testing.assert_allclose(torque, [-2.0] * 6 + [0.0])

    def test_combination_limit_deadband_and_rate(self):
        total = combine_torque(
            [0.5, 0.01], [0.8, 0.01], [0.0, 0.0], [1.0, 1.0], deadband=0.05)
        np.testing.assert_allclose(total, [1.0, 0.0])
        limited = rate_limit(total, [0.0, 0.0], [2.0, 2.0], 0.1)
        np.testing.assert_allclose(limited, [0.2, 0.0])

    def test_non_finite_torque_is_rejected(self):
        with self.assertRaises(ValueError):
            combine_torque([np.nan], [0.0], [0.0], [1.0])


class XM430CurrentConverterTest(unittest.TestCase):
    def test_conversion_sign_and_limit(self):
        converter = XM430CurrentConverter(
            [1.783, 1.783], [0.2, 0.2], [1.0, -1.0])
        self.assertEqual(converter.current_limits_raw, [42, 42])
        self.assertEqual(converter.torque_to_raw([0.1, 0.3]), [21, -42])

    def test_invalid_calibration_is_rejected(self):
        with self.assertRaises(ValueError):
            XM430CurrentConverter([1.783], [0.2], [0.0])


if __name__ == "__main__":
    unittest.main()
