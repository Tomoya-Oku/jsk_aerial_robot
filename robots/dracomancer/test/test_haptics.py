#!/usr/bin/env python3
"""Watchdog checks using generated ROS interfaces; no robot/servo output."""
from pathlib import Path
import sys
import threading
import io
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import rospy
import numpy as np
from spinal.msg import ServoControlCmd, ServoTorqueCmd
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts" / "control"))
from contact_haptics import ContactHaptics


class WatchdogTest(unittest.TestCase):
    def setUp(self):
        self.node = ContactHaptics.__new__(ContactHaptics)
        n = self.node
        n.lock = threading.RLock()
        n.model = SimpleNamespace(required_joints={"bend"})
        n.mode, n.flight, n.flight_time = True, 5, 100
        n.timeout = 0.1
        n.inputs = {key: (None, rospy.Time.from_sec(10)) for key in ("clock", "mapping", "wrench")}
        n.received = {key: 100 for key in n.inputs}
        n.advanced = dict(n.received)
        n.joint_times = {"bend": (100, rospy.Time.from_sec(10))}
        n.active = True
        n.hardware, n.ids = True, list(range(7))
        n.signs, n.coefficients, n.current_limits = np.ones(7), np.full(7, 1.78), np.full(7, 41)
        n.device_names = []
        n.publish_current = MagicMock()
        n.publish_torque = MagicMock()
        n.current_pub, n.torque_pub = MagicMock(), MagicMock()
        n.debug_pub, n.enabled_pub = MagicMock(), MagicMock()
        self.timer = patch("contact_haptics.time.monotonic", return_value=100)
        self.timer.start()
        self.addCleanup(self.timer.stop)

    def test_healthy_then_physics_pause(self):
        self.assertTrue(self.node.ready())
        self.node.advanced["clock"] = 99.899
        self.assertFalse(self.node.ready())

    def test_republished_old_measurement_is_not_fresh(self):
        self.node.inputs["mapping"] = (None, rospy.Time.from_sec(9.8))
        self.assertFalse(self.node.ready())

    def test_one_missing_or_stale_joint_stops_output(self):
        self.node.joint_times.clear()
        self.assertFalse(self.node.ready())
        self.node.joint_times["bend"] = (99.8, rospy.Time.from_sec(10))
        self.assertFalse(self.node.ready())

    def test_timeout_latches_until_explicit_enable(self):
        n = self.node
        n.advanced["clock"] = 99.899
        with patch("contact_haptics.rospy.Time.now", return_value=rospy.Time.from_sec(10)):
            n.tick()
            n.publish_current.assert_called_once_with([0] * 7)
            n.publish_torque.assert_called_once_with(False)
            self.assertFalse(n.active)
            n.advanced["clock"] = 100
            self.assertTrue(n.ready())
            n.tick()
            self.assertFalse(n.active)
            n.publish_current.assert_called_once()  # no resumed heartbeat

    def test_legacy_enable_without_firmware_service(self):
        n = self.node
        n.active = False
        # No haptic_config client exists: the original topic interfaces suffice.
        response = n.enable(SimpleNamespace(data=True))
        self.assertTrue(response.success)
        self.assertTrue(n.active)
        n.publish_current.assert_called_once_with([0] * 7)
        n.publish_torque.assert_called_once_with(True)
        n.torque = MagicMock(return_value=np.full(7, 0.2))
        with patch("contact_haptics.rospy.Time.now", return_value=rospy.Time.from_sec(10)):
            n.tick()
        np.testing.assert_array_equal(n.publish_current.call_args.args[0], np.full(7, 41))
        self.assertTrue(n.enable(SimpleNamespace(data=False)).success)
        n.publish_torque.assert_called_with(False)

    def test_original_spinal_messages_serialize(self):
        n = self.node
        ContactHaptics.publish_current(n, [41, -41, 0, 1, -1, 0, 0])
        current = n.current_pub.publish.call_args.args[0]
        self.assertIsInstance(current, ServoControlCmd)
        self.assertFalse(hasattr(current, "stamp"))
        self.assertEqual(current.index, n.ids)
        self.assertEqual(current.angles, [41, -41, 0, 1, -1, 0, 0])
        current.serialize(io.BytesIO())
        ContactHaptics.publish_torque(n, True)
        enabled = n.torque_pub.publish.call_args.args[0]
        self.assertIsInstance(enabled, ServoTorqueCmd)
        self.assertEqual(enabled.torque_enable, [1] * 7)
        enabled.serialize(io.BytesIO())

    def test_dry_run_does_not_send_legacy_commands(self):
        n = self.node
        n.hardware = False
        ContactHaptics.publish_current(n, [0] * 7)
        ContactHaptics.publish_torque(n, True)
        n.current_pub.publish.assert_not_called()
        n.torque_pub.publish.assert_not_called()

    def test_time_reset_trips_and_invalidates_progress(self):
        self.node.store("clock", None, rospy.Time.from_sec(1))
        self.assertFalse(self.node.active)
        self.assertFalse(self.node.ready())


if __name__ == "__main__":
    unittest.main()
