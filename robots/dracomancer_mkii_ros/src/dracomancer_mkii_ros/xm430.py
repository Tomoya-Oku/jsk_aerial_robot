#!/usr/bin/env python3
"""Transport-independent XM430-W350 current control. No bus is opened here.

The adapter must implement read(id, address, size) -> unsigned integer and
write(id, address, size, unsigned_value), checking communication/status errors.
Spinal currently cannot provide these read/write operations over its ROS API.
"""
import math


class CurrentScale:
    UNIT_A = 0.00269

    def __init__(self, joint_nm_per_amp, direction, max_current_amp):
        if (not math.isfinite(joint_nm_per_amp) or joint_nm_per_amp <= 0
                or direction not in (-1, 1) or not math.isfinite(max_current_amp)
                or max_current_amp < self.UNIT_A):
            raise ValueError('A calibrated positive Nm/A, direction and current limit are required')
        self.constant = joint_nm_per_amp
        self.direction = direction
        self.limit = min(1193, math.floor(max_current_amp/self.UNIT_A))

    def to_raw(self, torque_nm, device_limit):
        if not math.isfinite(torque_nm):
            raise ValueError('Non-finite torque')
        limit = min(self.limit, device_limit)
        if not 1 <= limit <= 1193:
            raise ValueError('Invalid Current Limit readback')
        raw_float = torque_nm/self.constant/self.UNIT_A*self.direction
        if abs(raw_float) > limit:
            raise ValueError('Torque exceeds configured or device current limit')
        return int(round(raw_float))


class CurrentSession:
    """Mode-0 lifecycle for an exclusively owned Protocol 2.0 RS485 bus.

    prepare() leaves torque OFF. arm() is a separate explicit operator action.
    Never instantiate against a bus also owned by Neuron or another driver.
    """
    def __init__(self, bus, servo_id, scale):
        if not 0 <= servo_id <= 252:
            raise ValueError('Invalid bus ID (not Spinal index)')
        self.bus, self.id, self.scale = bus, servo_id, scale
        self.ready = False
        self.armed = False
        self.previous_mode = None

    def read(self, address, size):
        return self.bus.read(self.id, address, size)

    def write(self, address, size, value):
        self.bus.write(self.id, address, size, value & ((1 << (8*size))-1))

    def check(self):
        if self.read(0, 2) != 1020 or self.read(13, 1) != 2:
            raise RuntimeError('Expected XM430-W350, Protocol 2.0')
        if self.read(6, 1) < 38:
            raise RuntimeError('Firmware with Bus Watchdog support is required')
        if self.read(70, 1) != 0:
            raise RuntimeError('Hardware Error Status is nonzero')

    def prepare(self):
        self.check()  # Verify identity before any write.
        try:
            self.write(64, 1, 0)
            if self.read(64, 1) != 0:
                raise RuntimeError('Torque OFF readback failed')
            self.previous_mode = self.read(11, 1)
            if self.previous_mode != 0:
                self.write(11, 1, 0)
            if self.read(11, 1) != 0:
                raise RuntimeError('Current Control mode readback failed')
            self.write(102, 2, 0)  # Mode switches reset Goal Current to Current Limit.
            if self.read(102, 2) != 0:
                raise RuntimeError('Zero Goal Current readback failed')
            self.scale.to_raw(0.0, self.read(38, 2))
            self.write(98, 1, 0)  # Clear a previous watchdog error.
            self.write(98, 1, 5)  # 100 ms; transport must maintain a faster cycle.
            if self.read(98, 1) != 5:
                raise RuntimeError('Bus Watchdog readback failed')
            self.ready = True
        except Exception:
            self.close()
            raise

    def arm(self):
        if not self.ready:
            raise RuntimeError('Call prepare first')
        try:
            self.check()
            if self.read(11, 1) != 0 or self.read(98, 1) != 5:
                raise RuntimeError('Mode or watchdog changed')
            self.write(102, 2, 0)
            self.write(64, 1, 1)
            if self.read(64, 1) != 1:
                raise RuntimeError('Torque ON readback failed')
            self.armed = True
        except Exception:
            self.close()
            raise

    def command(self, torque_nm):
        if not self.armed:
            raise RuntimeError('Current session is not armed')
        try:
            if self.read(11, 1) != 0 or self.read(64, 1) != 1 or self.read(70, 1) != 0:
                raise RuntimeError('Servo state does not permit current command')
            raw = self.scale.to_raw(torque_nm, self.read(38, 2))
            self.write(102, 2, raw)
            return raw
        except Exception:
            self.close()
            raise

    def close(self):
        self.armed = self.ready = False
        # Best effort zero and torque OFF independently, even after communication faults.
        for address, size in ((102, 2), (64, 1)):
            try:
                self.write(address, size, 0)
            except Exception:
                pass
        # Do not silently restore EEPROM mode/gains and re-enable position torque.
