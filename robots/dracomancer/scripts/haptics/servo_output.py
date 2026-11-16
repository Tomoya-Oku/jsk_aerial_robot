#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Fail-safe XM430-W350-R current output for Dracomancer Mk-II."""

import time

import rospy
from spinal.msg import ServoControlCmd, ServoTorqueCmd

from current_conversion import XM430CurrentConverter


class XM430CurrentOutput:
    def __init__(self, joint_names, servo_ids, torque_constants, torque_limits,
                 actuator_signs, current_topic, torque_enable_topic,
                 hardware_enabled=False, current_unit_amp=0.00269,
                 current_limits_raw=None):
        self.joint_names = list(joint_names)
        self.servo_ids = [int(value) for value in servo_ids]
        if len(self.joint_names) != len(self.servo_ids):
            raise ValueError("joint names and servo IDs must have the same size")
        if len(set(self.servo_ids)) != len(self.servo_ids):
            raise ValueError("servo IDs must be unique")
        if any(value < 0 or value > 252 for value in self.servo_ids):
            raise ValueError("servo IDs must be in [0, 252]")
        self.converter = XM430CurrentConverter(
            torque_constants, torque_limits, actuator_signs,
            current_unit_amp, current_limits_raw)
        if self.converter.count != len(self.servo_ids):
            raise ValueError("XM430 calibration count must match servo count")
        self.hardware_enabled = bool(hardware_enabled)
        self.torque_enabled = False

        self.current_pub = rospy.Publisher(current_topic, ServoControlCmd, queue_size=1)
        self.torque_enable_pub = rospy.Publisher(
            torque_enable_topic, ServoTorqueCmd, queue_size=1)
        rospy.on_shutdown(self.shutdown)

        if self.hardware_enabled:
            # Start from a known zero-output state. Operating Mode=0 and the
            # hardware Current Limit must be configured before enabling output.
            rospy.sleep(0.1)
            self.publish_current([0] * len(self.servo_ids))
            self.set_torque_enabled(False, force=True)

    def torque_to_raw(self, torque):
        return self.converter.torque_to_raw(torque)

    def publish_current(self, raw_current):
        msg = ServoControlCmd()
        msg.index = list(self.servo_ids)
        msg.angles = [int(value) for value in raw_current]
        self.current_pub.publish(msg)

    def set_torque_enabled(self, enabled, force=False):
        enabled = bool(enabled)
        if not force and self.torque_enabled == enabled:
            return
        msg = ServoTorqueCmd()
        msg.index = list(self.servo_ids)
        msg.torque_enable = [1 if enabled else 0] * len(self.servo_ids)
        self.torque_enable_pub.publish(msg)
        self.torque_enabled = enabled

    def command(self, torque, active):
        if not self.hardware_enabled:
            return
        if active:
            self.publish_current(self.torque_to_raw(torque))
            self.set_torque_enabled(True)
        else:
            self.publish_current([0] * len(self.servo_ids))
            self.set_torque_enabled(False)

    def shutdown(self):
        if not self.hardware_enabled:
            return
        # Repeat zero/OFF because shutdown can race the serial bridge.
        for _ in range(3):
            self.publish_current([0] * len(self.servo_ids))
            self.set_torque_enabled(False, force=True)
            time.sleep(0.02)
