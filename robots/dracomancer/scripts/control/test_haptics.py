#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from sensor_msgs.msg import JointState
from spinal.msg import ServoControlCmd, ServoTorqueCmd


class HapticPositionTester:
    def __init__(self):
        rospy.init_node("test_haptics")

        self.device_ns = rospy.get_param("~device_ns", "/dracomancer").rstrip("/")
        self.device_joint_topic = rospy.get_param(
            "~device_joint_topic", self.device_ns + "/joint_states")
        self.command_topic = rospy.get_param(
            "~command_topic", self.device_ns + "/servo/target_states")
        self.torque_enable_topic = rospy.get_param(
            "~torque_enable_topic", self.device_ns + "/servo/torque_enable")
        self.debug_topic = rospy.get_param(
            "~debug_topic", self.device_ns + "/test_haptic_torque")
        self.rate_hz = rospy.get_param("~rate", 100.0)
        self.duration = rospy.get_param("~duration", 0.0)
        self.joint_state_timeout = rospy.get_param("~joint_state_timeout", 0.5)

        self.servo_id = int(rospy.get_param("~servo_id", 0))
        self.pseudo_torque = float(rospy.get_param("~pseudo_torque", 0.0))
        self.offset_per_nm = float(rospy.get_param("~offset_per_nm", 0.25))
        self.max_offset = float(rospy.get_param("~max_offset", 0.03))
        self.offset_sign = float(rospy.get_param("~offset_sign", 1.0))
        self.deadband = float(rospy.get_param("~deadband", 0.0))
        self.torque_threshold = float(rospy.get_param("~torque_threshold", 0.02))
        self.ramp_time = float(rospy.get_param("~ramp_time", 1.0))
        self.command_republish_interval = float(rospy.get_param("~command_republish_interval", 0.02))
        self.reference_mode = str(rospy.get_param("~reference_mode", "initial")).strip().lower()
        self.output_mode = str(rospy.get_param("~output_mode", "pulse")).strip().lower()
        self.pulse_rate = float(rospy.get_param("~pulse_rate", 25.0))
        self.pulse_duty = float(rospy.get_param("~pulse_duty", 0.4))
        self.release_on_shutdown = rospy.get_param("~release_on_shutdown", True)

        self.center_tick = rospy.get_param("~center_tick", 2048.0)
        self.ticks_per_rev = rospy.get_param("~ticks_per_rev", 4096.0)
        self.min_tick = int(rospy.get_param("~min_tick", 0))
        self.max_tick = int(rospy.get_param("~max_tick", 4095))
        self.offsets = rospy.get_param("~offsets", {
            0: 0.0,
            1: 0.0,
            2: 0.0,
            3: 0.0,
            4: 0.0,
            5: 0.0,
            6: 0.0,
        })
        self.id_to_joint_name = rospy.get_param("~id_to_joint_name", {
            0: "shoulder_abduction_adduction_joint",
            1: "shoulder_flexion_extension_joint",
            2: "upper_arm_external_internal_rotation_joint",
            3: "elbow_flexion_extension_joint",
            4: "wrist_supination_joint",
            5: "wrist_flexion_extension_joint",
            6: "wrist_abduction_adduction_joint",
        })
        self.joint_name = rospy.get_param(
            "~joint_name", self.id_to_joint_name.get(self.servo_id, ""))

        self.latest_joints = {}
        self.last_joint_stamp = None
        self.last_joint_recv_time = None
        self.initial_joint_rad = None
        self.last_target_rad = None
        self.last_command_tick = None
        self.last_command_time = rospy.Time(0)
        self.last_torque_enable = None

        self.command_pub = rospy.Publisher(self.command_topic, ServoControlCmd, queue_size=1)
        self.torque_enable_pub = rospy.Publisher(self.torque_enable_topic, ServoTorqueCmd, queue_size=1)
        self.debug_pub = rospy.Publisher(self.debug_topic, JointState, queue_size=1)
        rospy.Subscriber(self.device_joint_topic, JointState, self.joint_cb, queue_size=1)
        rospy.on_shutdown(self.on_shutdown)

        if self.reference_mode not in ("initial", "current"):
            rospy.logwarn("unknown reference_mode '%s'; fallback to 'initial'", self.reference_mode)
            self.reference_mode = "initial"
        if self.output_mode not in ("hold", "pulse"):
            rospy.logwarn("unknown output_mode '%s'; fallback to 'pulse'", self.output_mode)
            self.output_mode = "pulse"
        self.pulse_duty = max(0.0, min(1.0, self.pulse_duty))

        rospy.loginfo("test_haptics: servo_id=%d joint_name=%s pseudo_torque=%.4f Nm",
                      self.servo_id, self.joint_name, self.pseudo_torque)
        rospy.loginfo("test_haptics: command_topic=%s reference_mode=%s output_mode=%s",
                      self.command_topic, self.reference_mode, self.output_mode)

    def joint_cb(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        self.latest_joints = {
            name: float(pos) for name, pos in zip(msg.name, msg.position)
        }
        self.last_joint_stamp = stamp
        self.last_joint_recv_time = rospy.Time.now()
        if self.initial_joint_rad is None and self.joint_name in self.latest_joints:
            self.initial_joint_rad = self.latest_joints[self.joint_name]
            rospy.loginfo("test_haptics: captured initial %s = %.4f rad",
                          self.joint_name, self.initial_joint_rad)

    def joint_state_ready(self):
        if not self.joint_name:
            rospy.logerr_throttle(1.0, "joint_name is empty; set ~joint_name or valid ~servo_id")
            return False
        if self.last_joint_recv_time is None:
            rospy.logwarn_throttle(1.0, "waiting for joint state: %s", self.device_joint_topic)
            return False
        if (rospy.Time.now() - self.last_joint_recv_time).to_sec() > self.joint_state_timeout:
            rospy.logwarn_throttle(1.0, "joint state is stale; skip command")
            return False
        if self.joint_name not in self.latest_joints:
            rospy.logwarn_throttle(1.0, "joint %s not found in joint_states", self.joint_name)
            return False
        return True

    def servo_offset(self):
        return float(self.offsets.get(self.servo_id, self.offsets.get(str(self.servo_id), 0.0)))

    def rad_to_tick(self, rad):
        tick = self.center_tick + (float(rad) - self.servo_offset()) * self.ticks_per_rev / (2.0 * math.pi)
        return int(max(self.min_tick, min(self.max_tick, round(tick))))

    def ramp_scale(self):
        if self.ramp_time <= 0.0:
            return 1.0
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        return max(0.0, min(1.0, elapsed / self.ramp_time))

    def torque_to_delta(self, pseudo_torque=None):
        tau = self.pseudo_torque if pseudo_torque is None else float(pseudo_torque)
        if abs(tau) < self.deadband:
            tau = 0.0
        delta = self.offset_sign * tau * self.offset_per_nm
        delta = max(-abs(self.max_offset), min(abs(self.max_offset), delta))
        return self.ramp_scale() * delta

    def torque_active(self, pseudo_torque=None):
        tau = self.pseudo_torque if pseudo_torque is None else float(pseudo_torque)
        if abs(tau) < self.deadband:
            tau = 0.0
        return abs(tau) >= abs(self.torque_threshold)

    def reference_rad(self):
        if self.reference_mode == "current":
            return self.latest_joints[self.joint_name]
        return self.initial_joint_rad

    def make_command(self, pseudo_torque=None):
        if not self.joint_state_ready():
            return None
        if self.initial_joint_rad is None:
            return None
        if not self.torque_active(pseudo_torque):
            return None

        target_rad = self.reference_rad() + self.torque_to_delta(pseudo_torque)
        self.last_target_rad = target_rad

        cmd = ServoControlCmd()
        cmd.index = [self.servo_id]
        cmd.angles = [self.rad_to_tick(target_rad)]
        return cmd

    def should_publish_command(self, cmd):
        now = rospy.Time.now()
        tick = cmd.angles[0]
        if tick != self.last_command_tick:
            return True
        if self.command_republish_interval <= 0.0:
            return False
        return (now - self.last_command_time).to_sec() >= self.command_republish_interval

    def output_phase_enabled(self):
        if self.output_mode != "pulse":
            return True
        if self.pulse_rate <= 0.0 or self.pulse_duty >= 1.0:
            return True
        if self.pulse_duty <= 0.0:
            return False

        now = rospy.Time.now()
        elapsed = (now - self.start_time).to_sec()
        period = 1.0 / self.pulse_rate
        return (elapsed % period) < (period * self.pulse_duty)

    def publish_debug(self, target_rad):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [self.joint_name]
        msg.position = [target_rad]
        msg.effort = [self.pseudo_torque]
        self.debug_pub.publish(msg)

    def on_shutdown(self):
        self.release_torque()

    def publish_torque_enable(self, enable, force=False):
        enable = bool(enable)
        if not force and self.last_torque_enable == enable:
            return
        msg = ServoTorqueCmd()
        msg.index = [self.servo_id]
        msg.torque_enable = [1 if enable else 0]
        self.torque_enable_pub.publish(msg)
        self.last_torque_enable = enable

    def release_torque(self):
        if not self.release_on_shutdown:
            return
        self.publish_torque_enable(False, force=True)
        self.last_command_tick = None
        rospy.sleep(0.05)

    def main(self):
        rate = rospy.Rate(self.rate_hz)
        self.start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.duration > 0.0 and (rospy.Time.now() - self.start_time).to_sec() >= self.duration:
                break
            cmd = self.make_command()
            if cmd is not None:
                if self.output_phase_enabled():
                    # Send the fresh position target before enabling torque so
                    # each pulse uses the current pseudo-force target.
                    if self.should_publish_command(cmd):
                        self.command_pub.publish(cmd)
                        self.last_command_tick = cmd.angles[0]
                        self.last_command_time = rospy.Time.now()
                    self.publish_torque_enable(True)
                else:
                    self.publish_torque_enable(False)
                if self.last_target_rad is not None:
                    self.publish_debug(self.last_target_rad)
            else:
                self.publish_torque_enable(False)
            rate.sleep()
        self.release_torque()


if __name__ == "__main__":
    try:
        HapticPositionTester().main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))
