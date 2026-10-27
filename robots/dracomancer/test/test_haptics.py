#!/usr/bin/env python3
"""Watchdog checks using generated ROS interfaces; no robot/servo output."""
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import rospy
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
        n.active, n.arming = True, False
        n.device_names = []
        n.publish_current = MagicMock()
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
            self.assertFalse(n.active)
            n.advanced["clock"] = 100
            self.assertTrue(n.ready())
            n.tick()
            self.assertFalse(n.active)
            n.publish_current.assert_called_once()  # no resumed heartbeat

    def test_time_reset_trips_and_invalidates_progress(self):
        self.node.store("clock", None, rospy.Time.from_sec(1))
        self.assertFalse(self.node.active)
        self.assertFalse(self.node.ready())


if __name__ == "__main__":
    unittest.main()
