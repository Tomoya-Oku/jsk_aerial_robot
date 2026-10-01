#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS-independent XM430 torque-to-current conversion."""


class XM430CurrentConverter:
    MAX_CURRENT_RAW = 1193

    def __init__(self, torque_constants, torque_limits, actuator_signs,
                 current_unit_amp=0.00269, current_limits_raw=None):
        self.count = len(torque_constants)
        if self.count == 0:
            raise ValueError("at least one actuator is required")
        if len(torque_limits) != self.count or len(actuator_signs) != self.count:
            raise ValueError("XM430 calibration vectors must have the same size")

        self.torque_constants = [float(value) for value in torque_constants]
        self.torque_limits = [abs(float(value)) for value in torque_limits]
        self.actuator_signs = [float(value) for value in actuator_signs]
        self.current_unit_amp = float(current_unit_amp)

        if self.current_unit_amp <= 0.0:
            raise ValueError("current_unit_amp must be positive")
        if any(value <= 0.0 for value in self.torque_constants):
            raise ValueError("torque constants must be positive")
        if any(value <= 0.0 for value in self.torque_limits):
            raise ValueError("torque limits must be positive")
        if any(value not in (-1.0, 1.0) for value in self.actuator_signs):
            raise ValueError("actuator signs must be +1 or -1")

        configured = list(current_limits_raw or [])
        if configured and len(configured) != self.count:
            raise ValueError("current_limits_raw must be empty or match actuator count")
        if configured:
            self.current_limits_raw = [abs(int(value)) for value in configured]
        else:
            self.current_limits_raw = [
                int(round(limit / (kt * self.current_unit_amp)))
                for limit, kt in zip(self.torque_limits, self.torque_constants)
            ]
        if any(value <= 0 or value > self.MAX_CURRENT_RAW
               for value in self.current_limits_raw):
            raise ValueError("XM430 current limits must be in [1, 1193]")

    def torque_to_raw(self, torque):
        values = list(torque)
        if len(values) != self.count:
            raise ValueError("torque vector size does not match actuator count")

        result = []
        for value, sign, kt, limit in zip(
                values, self.actuator_signs, self.torque_constants,
                self.current_limits_raw):
            raw = int(round(sign * float(value) / (kt * self.current_unit_amp)))
            result.append(max(-limit, min(limit, raw)))
        return result
