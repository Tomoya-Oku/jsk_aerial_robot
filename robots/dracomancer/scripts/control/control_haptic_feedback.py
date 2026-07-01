#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from spinal.msg import ServoControlCmd, ServoTorqueCmd


class HapticFeedback:
    def __init__(self):
        rospy.init_node("control_haptic_feedback")

        self.device_ns = rospy.get_param("~device_ns", "/dracomancer").rstrip("/")
        self.device_joint_topic = rospy.get_param("~device_joint_topic", self.device_ns + "/joint_states")
        self.shape_error_topic = rospy.get_param("~shape_error_topic", self.device_ns + "/shape_control_error")
        self.mode_topic = rospy.get_param("~mode_topic", self.device_ns + "/teleop_mode")
        self.haptic_debug_topic = rospy.get_param("~haptic_debug_topic", self.device_ns + "/haptic_torque")
        self.haptic_torque_enable_topic = rospy.get_param(
            "~haptic_torque_enable_topic", self.device_ns + "/servo/torque_enable")
        self.haptic_current_topic = rospy.get_param(
            "~haptic_current_topic", self.device_ns + "/servo/target_current")
        self.haptic_position_topic = rospy.get_param(
            "~haptic_position_topic", self.device_ns + "/servo/target_states")
        self.rate_hz = rospy.get_param("~rate", 100.0)

        self.enable_haptic_torque_onoff = rospy.get_param("~enable_haptic_torque_onoff", False)
        self.haptic_torque_on_threshold = rospy.get_param("~haptic_torque_on_threshold", 0.02)
        self.enable_haptic_current_command = rospy.get_param("~enable_haptic_current_command", False)
        self.haptic_current_per_nm = rospy.get_param("~haptic_current_per_nm", 0.0)
        self.enable_haptic_position_command = rospy.get_param("~enable_haptic_position_command", False)
        self.haptic_position_torque_threshold = rospy.get_param(
            "~haptic_position_torque_threshold", self.haptic_torque_on_threshold)
        self.haptic_position_offset_per_nm = rospy.get_param("~haptic_position_offset_per_nm", 0.25)
        self.haptic_position_max_offset = rospy.get_param("~haptic_position_max_offset", 0.03)
        self.haptic_position_offset_sign = rospy.get_param("~haptic_position_offset_sign", 1.0)
        self.haptic_position_republish_interval = rospy.get_param("~haptic_position_republish_interval", 0.02)
        self.haptic_position_output_mode = str(rospy.get_param(
            "~haptic_position_output_mode", "pulse")).strip().lower()
        self.haptic_position_pulse_rate = float(rospy.get_param("~haptic_position_pulse_rate", 25.0))
        self.haptic_position_pulse_duty = float(rospy.get_param("~haptic_position_pulse_duty", 0.4))
        self.haptic_torque_deadband = rospy.get_param(
            "~haptic_torque_deadband",
            rospy.get_param("~haptic_position_deadband", 0.0))
        self.haptic_position_reference_mode = str(rospy.get_param(
            "~haptic_position_reference_mode", "anchor")).strip().lower()
        self.release_torque_on_start = rospy.get_param("~release_torque_on_start", True)
        self.release_torque_on_shutdown = rospy.get_param("~release_torque_on_shutdown", True)
        self.haptic_position_min_tick = int(rospy.get_param("~haptic_position_min_tick", 0))
        self.haptic_position_max_tick = int(rospy.get_param("~haptic_position_max_tick", 4095))
        self.joint_state_timeout = rospy.get_param("~joint_state_timeout", 0.5)
        self.center_tick = rospy.get_param("~center_tick", 2048.0)
        self.ticks_per_rev = rospy.get_param("~ticks_per_rev", 4096.0)
        self.offsets = rospy.get_param("~offsets", {
            0: 0.0,
            1: 0.0,
            2: 0.0,
            3: 0.0,
            4: 0.0,
            5: 0.0,
            6: 0.0,
        })
        self.haptic_stiffness = rospy.get_param("~haptic_stiffness", [0.2] * 6)
        self.haptic_damping = rospy.get_param("~haptic_damping", 0.01)
        self.haptic_torque_limits = rospy.get_param("~haptic_torque_limits", [0.2] * 7)
        self.shape_error_timeout = rospy.get_param("~shape_error_timeout", 0.5)
        self.haptic_servo_ids = rospy.get_param("~haptic_servo_ids", [0, 1, 2, 3, 4, 5, 6])

        self.source_joint_names = rospy.get_param("~source_joint_names", [
            "wrist_flexion_extension_joint",
            "wrist_supination_joint",
            "upper_arm_external_internal_rotation_joint",
            "elbow_flexion_extension_joint",
            "shoulder_flexion_extension_joint",
            "shoulder_abduction_adduction_joint",
        ])
        self.signs = rospy.get_param("~signs", [1.0, 1.0, 1.0, -1.0, 1.0, 1.0])
        self.scales = rospy.get_param("~scales", [1.0] * len(self.source_joint_names))
        self.haptic_device_joint_names = rospy.get_param("~haptic_device_joint_names", [
            "shoulder_abduction_adduction_joint",
            "shoulder_flexion_extension_joint",
            "upper_arm_external_internal_rotation_joint",
            "elbow_flexion_extension_joint",
            "wrist_supination_joint",
            "wrist_flexion_extension_joint",
            "wrist_abduction_adduction_joint",
        ])
        self.device_joint_index = {
            name: i for i, name in enumerate(self.haptic_device_joint_names)
        }

        self.teleop_mode = self.normalize_mode(rospy.get_param("~teleop_mode", "startup"))
        self.shape_error = [0.0] * len(self.source_joint_names)
        self.last_shape_error_stamp = rospy.Time(0)
        self.latest_device_joints = {}
        self.device_joint_velocity = {}
        self.last_device_joint_stamp = None
        self.last_device_joint_recv_time = None
        self.haptic_position_anchors = {}
        self.haptic_position_last_ticks = {}
        self.haptic_position_last_pub_time = {}
        self.haptic_position_torque_enable_state = {}

        self.haptic_debug_pub = rospy.Publisher(self.haptic_debug_topic, JointState, queue_size=1)
        self.haptic_torque_enable_pub = None
        if self.enable_haptic_torque_onoff or self.enable_haptic_position_command:
            self.haptic_torque_enable_pub = rospy.Publisher(
                self.haptic_torque_enable_topic, ServoTorqueCmd, queue_size=1)
            rospy.on_shutdown(self.release_all_torque)
        self.haptic_current_pub = None
        if self.enable_haptic_current_command:
            if self.haptic_current_per_nm <= 0.0:
                rospy.logwarn("haptic_current_per_nm must be positive; haptic current command is disabled")
                self.enable_haptic_current_command = False
            else:
                self.haptic_current_pub = rospy.Publisher(self.haptic_current_topic, ServoControlCmd, queue_size=1)
        self.haptic_position_pub = None
        if self.enable_haptic_position_command:
            if self.haptic_position_offset_per_nm <= 0.0:
                rospy.logwarn("haptic_position_offset_per_nm must be positive; haptic position command is disabled")
                self.enable_haptic_position_command = False
            else:
                if self.haptic_position_reference_mode not in ("anchor", "current"):
                    rospy.logwarn("unknown haptic_position_reference_mode '%s'; fallback to 'anchor'",
                                  self.haptic_position_reference_mode)
                    self.haptic_position_reference_mode = "anchor"
                if self.haptic_position_output_mode not in ("hold", "pulse"):
                    rospy.logwarn("unknown haptic_position_output_mode '%s'; fallback to 'pulse'",
                                  self.haptic_position_output_mode)
                    self.haptic_position_output_mode = "pulse"
                self.haptic_position_pulse_duty = max(0.0, min(1.0, self.haptic_position_pulse_duty))
                self.haptic_position_pub = rospy.Publisher(
                    self.haptic_position_topic, ServoControlCmd, queue_size=1)

        rospy.Subscriber(self.device_joint_topic, JointState, self.device_joint_cb, queue_size=1)
        rospy.Subscriber(self.shape_error_topic, Float64MultiArray, self.shape_error_cb, queue_size=1)
        rospy.Subscriber(self.mode_topic, String, self.mode_cb, queue_size=1)

        rospy.loginfo("device_joint_topic: %s", self.device_joint_topic)
        rospy.loginfo("shape_error_topic: %s", self.shape_error_topic)
        rospy.loginfo("haptic_debug_topic: %s", self.haptic_debug_topic)
        rospy.loginfo("haptic torque on/off command: %s -> %s",
                      self.enable_haptic_torque_onoff, self.haptic_torque_enable_topic)
        rospy.loginfo("haptic current command: %s -> %s",
                      self.enable_haptic_current_command, self.haptic_current_topic)
        rospy.loginfo("haptic position command: %s -> %s",
                      self.enable_haptic_position_command, self.haptic_position_topic)
        rospy.loginfo("haptic position output: mode=%s pulse_rate=%.1f duty=%.2f",
                      self.haptic_position_output_mode,
                      self.haptic_position_pulse_rate,
                      self.haptic_position_pulse_duty)
        if self.release_torque_on_start:
            # Start from a known free state.  This also clears stale torque ON
            # left by a previous node stop during a pulse ON phase.
            rospy.sleep(0.1)
            self.release_all_torque(force=True)

    @staticmethod
    def normalize_mode(mode):
        # "teleop" is accepted as a shorthand alias for "teleoperation".
        mode = str(mode).strip().lower()
        return "teleoperation" if mode == "teleop" else mode

    def mode_cb(self, msg):
        self.teleop_mode = self.normalize_mode(msg.data)

    def shape_error_cb(self, msg):
        self.shape_error = self.fit_list(msg.data, len(self.source_joint_names), 0.0)
        self.last_shape_error_stamp = rospy.Time.now()

    def device_joint_cb(self, msg):
        now = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        next_joints = {
            name: float(pos) for name, pos in zip(msg.name, msg.position)
        }
        if self.last_device_joint_stamp is not None:
            dt = (now - self.last_device_joint_stamp).to_sec()
            if dt > 1e-6:
                self.device_joint_velocity = {
                    name: (pos - self.latest_device_joints.get(name, pos)) / dt
                    for name, pos in next_joints.items()
                }
        self.latest_device_joints = next_joints
        self.last_device_joint_stamp = now
        self.last_device_joint_recv_time = rospy.Time.now()

    @staticmethod
    def fit_list(values, size, default):
        data = list(values)
        if len(data) < size:
            data.extend([default] * (size - len(data)))
        return data[:size]

    def shape_error_ready(self):
        return (rospy.Time.now() - self.last_shape_error_stamp).to_sec() <= self.shape_error_timeout

    def joint_state_ready(self):
        if self.last_device_joint_recv_time is None:
            return False
        return (rospy.Time.now() - self.last_device_joint_recv_time).to_sec() <= self.joint_state_timeout

    def haptic_torque(self):
        size = len(self.haptic_device_joint_names)
        if self.teleop_mode != "teleoperation" or not self.shape_error_ready():
            return [0.0] * size

        stiffness = self.fit_list(self.haptic_stiffness, len(self.source_joint_names), 0.0)
        limits = self.fit_list(self.haptic_torque_limits, size, 0.0)
        tau = [0.0] * size

        for j, source_name in enumerate(self.source_joint_names):
            device_idx = self.device_joint_index.get(source_name)
            if device_idx is None:
                continue
            shape_torque = stiffness[j] * self.shape_error[j]
            tau[device_idx] += self.signs[j] * self.scales[j] * shape_torque

        for name, velocity in self.device_joint_velocity.items():
            device_idx = self.device_joint_index.get(name)
            if device_idx is not None:
                tau[device_idx] -= self.haptic_damping * velocity

        return [
            max(-abs(limit), min(abs(limit), value))
            for value, limit in zip(tau, limits)
        ]

    def servo_offset(self, servo_id):
        return float(self.offsets.get(servo_id, self.offsets.get(str(servo_id), 0.0)))

    def rad_to_tick(self, rad, servo_id):
        tick = self.center_tick + (float(rad) - self.servo_offset(servo_id)) * self.ticks_per_rev / (2.0 * math.pi)
        return int(max(self.haptic_position_min_tick,
                       min(self.haptic_position_max_tick, round(tick))))

    def position_torque_value(self, torque, index):
        value = torque[index] if index < len(torque) else 0.0
        return 0.0 if abs(value) < self.haptic_torque_deadband else value

    def position_torque_active(self, torque, index):
        value = self.position_torque_value(torque, index)
        return abs(value) >= abs(self.haptic_position_torque_threshold)

    def position_gate_ready(self):
        return (self.teleop_mode == "teleoperation" and
                self.shape_error_ready() and
                self.joint_state_ready())

    def position_output_phase_enabled(self):
        if self.haptic_position_output_mode != "pulse":
            return True
        if self.haptic_position_pulse_rate <= 0.0 or self.haptic_position_pulse_duty >= 1.0:
            return True
        if self.haptic_position_pulse_duty <= 0.0:
            return False

        now = rospy.Time.now().to_sec()
        period = 1.0 / self.haptic_position_pulse_rate
        return (now % period) < (period * self.haptic_position_pulse_duty)

    def publish_position_torque_enable(self, torque, output_phase_enabled):
        if self.haptic_torque_enable_pub is None:
            return
        desired = {}
        if not self.position_gate_ready():
            desired = {
                int(servo_id): False
                for servo_id, enabled in self.haptic_position_torque_enable_state.items()
                if enabled
            }
            self.haptic_position_anchors = {}
            self.haptic_position_last_ticks = {}
            self.haptic_position_last_pub_time = {}
        else:
            for i, (servo_id, joint_name) in enumerate(zip(self.haptic_servo_ids, self.haptic_device_joint_names)):
                servo_id = int(servo_id)
                active = (self.position_torque_active(torque, i) and
                          joint_name in self.latest_device_joints)
                enable = active and output_phase_enabled
                desired[servo_id] = enable
                if not active:
                    self.haptic_position_anchors.pop(servo_id, None)
                    self.haptic_position_last_ticks.pop(servo_id, None)
                    self.haptic_position_last_pub_time.pop(servo_id, None)

        changed = [
            (servo_id, enable)
            for servo_id, enable in desired.items()
            if self.haptic_position_torque_enable_state.get(servo_id) != enable
        ]
        if not changed:
            return

        enable_msg = ServoTorqueCmd()
        enable_msg.index = [servo_id for servo_id, _ in changed]
        enable_msg.torque_enable = [1 if enable else 0 for _, enable in changed]
        self.haptic_torque_enable_pub.publish(enable_msg)
        for servo_id, enable in changed:
            self.haptic_position_torque_enable_state[servo_id] = enable

    def publish_torque_enable(self, servo_ids, enable, force=False):
        if self.haptic_torque_enable_pub is None:
            return
        enable = bool(enable)
        servo_ids = [int(servo_id) for servo_id in servo_ids]
        if not force:
            servo_ids = [
                servo_id for servo_id in servo_ids
                if self.haptic_position_torque_enable_state.get(servo_id) != enable
            ]
        if not servo_ids:
            return

        msg = ServoTorqueCmd()
        msg.index = servo_ids
        msg.torque_enable = [1 if enable else 0] * len(servo_ids)
        self.haptic_torque_enable_pub.publish(msg)
        for servo_id in servo_ids:
            self.haptic_position_torque_enable_state[servo_id] = enable

    def release_all_torque(self, force=False):
        if not force and not self.release_torque_on_shutdown:
            return
        for _ in range(3):
            self.publish_torque_enable(self.haptic_servo_ids, False, force=True)
            rospy.sleep(0.02)
        self.haptic_position_anchors = {}
        self.haptic_position_last_ticks = {}
        self.haptic_position_last_pub_time = {}

    def should_publish_position_tick(self, servo_id, tick):
        now = rospy.Time.now()
        if self.haptic_position_last_ticks.get(servo_id) != tick:
            return True
        if self.haptic_position_republish_interval <= 0.0:
            return False
        last_pub = self.haptic_position_last_pub_time.get(servo_id, rospy.Time(0))
        return (now - last_pub).to_sec() >= self.haptic_position_republish_interval

    def haptic_position_command(self, torque, output_phase_enabled):
        if (not self.enable_haptic_position_command or
                self.haptic_position_pub is None):
            return None
        if not output_phase_enabled:
            return None
        if self.teleop_mode != "teleoperation" or not self.shape_error_ready():
            self.haptic_position_anchors = {}
            return None
        if not self.joint_state_ready():
            rospy.logwarn_throttle(1.0, "joint state is stale; skip haptic position command")
            return None

        cmd = ServoControlCmd()
        cmd.index = []
        cmd.angles = []
        requested_active = False
        valid_target = False
        for i, (servo_id, joint_name) in enumerate(zip(self.haptic_servo_ids, self.haptic_device_joint_names)):
            if not self.position_torque_active(torque, i):
                continue
            requested_active = True
            if joint_name not in self.latest_device_joints:
                continue
            valid_target = True
            tau = self.position_torque_value(torque, i)
            delta = self.haptic_position_offset_sign * tau * self.haptic_position_offset_per_nm
            delta = max(-abs(self.haptic_position_max_offset),
                        min(abs(self.haptic_position_max_offset), delta))
            servo_id = int(servo_id)
            if self.haptic_position_reference_mode == "current":
                reference_rad = self.latest_device_joints[joint_name]
            else:
                # Capture once when the cue starts.  This avoids the target
                # marching with the operator's arm and turning into active drive.
                if servo_id not in self.haptic_position_anchors:
                    self.haptic_position_anchors[servo_id] = self.latest_device_joints[joint_name]
                reference_rad = self.haptic_position_anchors[servo_id]
            target_rad = reference_rad + delta
            target_tick = self.rad_to_tick(target_rad, servo_id)
            if not self.should_publish_position_tick(servo_id, target_tick):
                continue
            cmd.index.append(int(servo_id))
            cmd.angles.append(target_tick)
            self.haptic_position_last_ticks[servo_id] = target_tick
            self.haptic_position_last_pub_time[servo_id] = rospy.Time.now()

        if requested_active and not valid_target:
            rospy.logwarn_throttle(1.0, "no valid joint state for haptic position command")
            return None
        if not cmd.index:
            return None
        return cmd

    def publish_haptic(self, torque):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.haptic_device_joint_names
        msg.effort = list(torque)
        self.haptic_debug_pub.publish(msg)

        if self.enable_haptic_position_command:
            output_phase_enabled = self.position_output_phase_enabled()
            position_msg = self.haptic_position_command(torque, output_phase_enabled)
            if position_msg is not None:
                self.haptic_position_pub.publish(position_msg)
            self.publish_position_torque_enable(torque, output_phase_enabled)
        elif self.enable_haptic_torque_onoff and self.haptic_torque_enable_pub is not None:
            enable_ids = []
            disable_ids = []
            for servo_id, value in zip(self.haptic_servo_ids, torque):
                if abs(value) >= self.haptic_torque_on_threshold:
                    enable_ids.append(servo_id)
                else:
                    disable_ids.append(servo_id)
            self.publish_torque_enable(enable_ids, True)
            self.publish_torque_enable(disable_ids, False)

        if self.enable_haptic_current_command and self.haptic_current_pub is not None:
            current_msg = ServoControlCmd()
            current_msg.index = list(self.haptic_servo_ids)
            current_msg.angles = [
                int(max(-32768, min(32767, round(value * self.haptic_current_per_nm))))
                for value in torque
            ]
            self.haptic_current_pub.publish(current_msg)

    def main(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.publish_haptic(self.haptic_torque())
            rate.sleep()


if __name__ == "__main__":
    try:
        node = HapticFeedback()
        node.main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))
