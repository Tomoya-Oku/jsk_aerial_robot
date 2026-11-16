#!/usr/bin/env python3
"""Protocol register/sequence tests using a fake bus, never real motors."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1] / 'src'))
from dracomancer_mkii_ros.xm430 import CurrentScale, CurrentSession


class FakeBus:
    def __init__(self):
        self.reg = {0: 1020, 13: 2, 6: 46, 70: 0, 64: 1, 11: 3, 102: 600, 38: 1193, 98: 0}
        self.writes = []
        self.fail_mode_readback = False

    def read(self, motor, address, size):
        return self.reg[address]

    def write(self, motor, address, size, value):
        self.writes.append((address, size, value))
        if address == 11:
            assert self.reg[64] == 0, 'EEPROM write requires torque OFF'
            if self.fail_mode_readback:
                return
            self.reg[102] = 1193  # Actual mode-change behavior from the manual.
        self.reg[address] = value


class CurrentTests(unittest.TestCase):
    def test_signed_units_and_calibration(self):
        scale = CurrentScale(2.0, -1, 0.5)
        self.assertEqual(scale.to_raw(0.538, 1193), -100)
        self.assertEqual(scale.to_raw(-0.538, 1193), 100)
        for value in (float('nan'), float('inf'), 1.1):
            with self.assertRaises(ValueError):
                scale.to_raw(value, 1193)
        with self.assertRaises(ValueError):
            scale.to_raw(0.538, 50)
        for kt in (0.0, -1, float('nan')):
            with self.assertRaises(ValueError):
                CurrentScale(kt, 1, 0.5)

    def test_torque_off_mode_readback_and_zero_before_enable(self):
        bus = FakeBus()
        session = CurrentSession(bus, 7, CurrentScale(2.0, -1, 0.5))
        session.prepare()
        self.assertEqual(bus.reg[64], 0)
        self.assertEqual(bus.writes[:3], [(64, 1, 0), (11, 1, 0), (102, 2, 0)])
        session.arm()
        self.assertEqual(bus.writes[-2:], [(102, 2, 0), (64, 1, 1)])
        self.assertEqual(session.command(0.538), -100)
        self.assertEqual(bus.reg[102], 65536-100)
        session.close()
        self.assertEqual(bus.reg[102], 0)
        self.assertEqual(bus.reg[64], 0)

    def test_mode_and_hardware_faults_fail_closed(self):
        bus = FakeBus()
        bus.fail_mode_readback = True
        session = CurrentSession(bus, 1, CurrentScale(2.0, 1, 0.5))
        with self.assertRaises(RuntimeError):
            session.prepare()
        self.assertFalse(session.ready)
        self.assertEqual(bus.reg[64], 0)
        bus.fail_mode_readback = False
        session.prepare()
        session.arm()
        bus.reg[70] = 4
        with self.assertRaises(RuntimeError):
            session.command(0.1)
        self.assertEqual(bus.reg[64], 0)

    def test_wrong_model_never_written(self):
        bus = FakeBus()
        bus.reg[0] = 999
        with self.assertRaises(RuntimeError):
            CurrentSession(bus, 1, CurrentScale(2, 1, 0.5)).prepare()
        self.assertEqual(bus.writes, [])


if __name__ == '__main__':
    unittest.main()
