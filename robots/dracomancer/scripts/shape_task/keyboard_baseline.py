#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Keyboard baseline input for the shape-target reaching task.

Comparison condition against the Dracomancer exoskeleton: the operator edits
the 6 DRAGON internal joint angles with the keyboard, while everything else
(target shapes, success condition, visualization, timeout, safety gate,
metrics) stays identical to the device condition.

To keep the comparison fair without touching control_joint_angle.py, this node
re-implements the SAME hold-type feasibility gate (predicted fc of the
candidate shape via /<robot>/shape_feasibility/check_shape, hard/recover
thresholds from the [hard_min, min] threshold topics, hysteresis hold) and the
same max_step rate limit and hover gating, and publishes to the same topics:

  /<robot>/joints_ctrl                 gated joint command (q_target)
  <device_ns>/shape_control_error      candidate - command   (q_candidate reconstruction)
  <device_ns>/candidate/fc_f_min|_t_min  predicted fc of the candidate

Run it with `rosrun` in its own terminal (needs raw stdin):
  rosrun dracomancer keyboard_baseline.py

Keys: 1-6 select joint / +,= increase / -,_ decrease / [,] halve,double step
      r reset candidate to measured pose / h help
"""

import sys
import select
import termios
import threading
import tty

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, UInt8

import dragon_fk

HELP = ("keys: [1-6] select joint  [+/=] +step  [-/_] -step  "
        "[\\[/\\]] step/2 step*2  [r] reset to measured  [h] help")


class KeyboardBaseline:
    def __init__(self):
        rospy.init_node("keyboard_baseline")

        self.robot_ns = rospy.get_param("~robot_ns", "dragon")
        self.device_ns = rospy.get_param("~device_ns", "/dracomancer").rstrip("/")
        self.joint_names = rospy.get_param("~dragon_joint_names", list(dragon_fk.JOINT_ORDER))
        self.rate_hz = float(rospy.get_param("~rate", 40.0))
        self.step = float(rospy.get_param("~step", 0.05))            # rad per key press
        self.joint_limit = float(rospy.get_param("~joint_limit", np.pi / 2.0))
        self.max_step = float(rospy.get_param("~max_step", 0.04))    # rad per cycle (same as device)
        self.startup_pose = rospy.get_param(
            "~startup_pose", [0.0, np.pi / 2.0, 0.0, np.pi / 2.0, 0.0, np.pi / 2.0])
        self.publish_only_when_hovering = bool(
            rospy.get_param("~publish_only_when_hovering", True))
        self.hover_flight_state = int(rospy.get_param("~hover_flight_state", 5))

        # Same gate parameters / defaults as control_joint_angle.py (hold mode).
        self.enable_feasibility_gate = bool(rospy.get_param("~enable_feasibility_gate", True))
        self.feasibility_service_name = rospy.get_param(
            "~feasibility_service", "/%s/shape_feasibility/check_shape" % self.robot_ns)
        self.feasibility_service_timeout = float(
            rospy.get_param("~feasibility_service_timeout", 2.0))
        self.feasibility_rate = float(rospy.get_param("~feasibility_rate", 20.0))
        self.force_radius_threshold = float(rospy.get_param("~force_radius_threshold", 0.108990))
        self.torque_radius_threshold = float(rospy.get_param("~torque_radius_threshold", 0.015400))
        self.force_radius_recover_threshold = float(
            rospy.get_param("~force_radius_recover_threshold", 0.249220))
        self.torque_radius_recover_threshold = float(
            rospy.get_param("~torque_radius_recover_threshold", 0.278159))

        self.candidate = None       # keyboard-edited raw candidate
        self.current_target = None  # rate-limited, gate-passed command
        self.last_feasible = None
        self.holding = False
        self.last_eval_t = 0.0
        self.selected = 0
        self.measured = None
        self.flight_state = None
        self.feasibility_srv = None

        self.cmd_pub = rospy.Publisher("/%s/joints_ctrl" % self.robot_ns,
                                       JointState, queue_size=10)
        self.shape_error_pub = rospy.Publisher(self.device_ns + "/shape_control_error",
                                               Float64MultiArray, queue_size=1)
        self.cand_f_pub = rospy.Publisher(self.device_ns + "/candidate/fc_f_min",
                                          Float64, queue_size=1)
        self.cand_t_pub = rospy.Publisher(self.device_ns + "/candidate/fc_t_min",
                                          Float64, queue_size=1)

        rospy.Subscriber("/%s/joint_states" % self.robot_ns, JointState,
                         self.joint_states_cb, queue_size=1)
        rospy.Subscriber("/%s/flight_state" % self.robot_ns, UInt8,
                         self.flight_state_cb, queue_size=1)
        rospy.Subscriber(self.device_ns + "/force_volume_radius_threshold",
                         Float64MultiArray, self.force_threshold_cb, queue_size=1)
        rospy.Subscriber(self.device_ns + "/torque_volume_radius_threshold",
                         Float64MultiArray, self.torque_threshold_cb, queue_size=1)

        rospy.loginfo("keyboard baseline: gate=%s service=%s", self.enable_feasibility_gate,
                      self.feasibility_service_name)
        rospy.loginfo(HELP)

    # ------------------------------------------------------------- callbacks
    def joint_states_cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        vals = [pos.get(n) for n in self.joint_names]
        if any(v is None for v in vals):
            return
        self.measured = [float(v) for v in vals]

    def flight_state_cb(self, msg):
        self.flight_state = int(msg.data)

    def force_threshold_cb(self, msg):
        if len(msg.data) > 0:
            self.force_radius_threshold = float(msg.data[0])
        if len(msg.data) > 1:
            self.force_radius_recover_threshold = max(
                self.force_radius_threshold, float(msg.data[1]))

    def torque_threshold_cb(self, msg):
        if len(msg.data) > 0:
            self.torque_radius_threshold = float(msg.data[0])
        if len(msg.data) > 1:
            self.torque_radius_recover_threshold = max(
                self.torque_radius_threshold, float(msg.data[1]))

    # -------------------------------------------------------------- keyboard
    def key_loop(self):
        # Raw terminal mode so single key presses arrive without Enter.
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    self.handle_key(sys.stdin.read(1))
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def handle_key(self, key):
        if key in "123456":
            self.selected = int(key) - 1
            print("\r[select] %s" % self.joint_names[self.selected])
        elif key in "+=":
            self.adjust(+self.step)
        elif key in "-_":
            self.adjust(-self.step)
        elif key == "[":
            self.step = max(self.step / 2.0, 0.001)
            print("\r[step] %.4f rad" % self.step)
        elif key == "]":
            self.step = min(self.step * 2.0, 0.5)
            print("\r[step] %.4f rad" % self.step)
        elif key == "r":
            if self.measured is not None:
                self.candidate = list(self.measured)
                print("\r[reset] candidate = measured pose")
        elif key == "h":
            print("\r" + HELP)

    def adjust(self, delta):
        if self.candidate is None:
            return
        i = self.selected
        self.candidate[i] = float(np.clip(self.candidate[i] + delta,
                                          -self.joint_limit, self.joint_limit))
        print("\r[%s] %.3f rad" % (self.joint_names[i], self.candidate[i]))

    # ------------------------------------------------------------ gate logic
    def connect_service(self):
        try:
            rospy.wait_for_service(self.feasibility_service_name,
                                   timeout=self.feasibility_service_timeout)
            from dracomancer.srv import ShapeFeasibility
            self.feasibility_srv = rospy.ServiceProxy(
                self.feasibility_service_name, ShapeFeasibility, persistent=True)
            rospy.loginfo("connected to %s", self.feasibility_service_name)
            return True
        except (rospy.ROSException, ImportError) as e:
            rospy.logwarn_throttle(5.0, "feasibility service unavailable: %s", e)
            self.feasibility_srv = None
            return False

    def evaluate(self, candidate):
        if self.feasibility_srv is None and not self.connect_service():
            return None, None, None
        from dracomancer.srv import ShapeFeasibilityRequest
        req = ShapeFeasibilityRequest()
        req.name = list(self.joint_names)
        req.position = list(candidate)
        try:
            res = self.feasibility_srv.call(req)
        except (rospy.ServiceException, TypeError):
            rospy.logwarn_throttle(5.0, "feasibility call failed; reconnecting")
            self.feasibility_srv = None
            return None, None, None
        if not res.valid:
            return None, res.fc_f_min, res.fc_t_min
        ok = (res.fc_f_min >= self.force_radius_threshold and
              res.fc_t_min >= self.torque_radius_threshold)
        return ok, res.fc_f_min, res.fc_t_min

    def gate(self, candidate):
        """Hold gate with hysteresis, mirroring control_joint_angle.py."""
        if not self.enable_feasibility_gate:
            self.last_feasible = list(candidate)
            return list(candidate)

        now = rospy.get_time()
        if self.feasibility_rate > 0.0 and now - self.last_eval_t < 1.0 / self.feasibility_rate:
            return list(self.last_feasible)
        self.last_eval_t = now

        feasible, fc_f, fc_t = self.evaluate(candidate)
        if fc_f is not None:
            self.cand_f_pub.publish(Float64(fc_f))
        if fc_t is not None:
            self.cand_t_pub.publish(Float64(fc_t))

        if feasible is None:  # service failure: hold conservatively
            return list(self.last_feasible)
        if self.holding:
            if (fc_f >= self.force_radius_recover_threshold and
                    fc_t >= self.torque_radius_recover_threshold):
                self.holding = False
                self.last_feasible = list(candidate)
            return list(self.last_feasible)
        if feasible:
            self.last_feasible = list(candidate)
        else:
            self.holding = True
            rospy.logwarn_throttle(1.0, "gate hold: fc_f=%.4f fc_t=%.4f", fc_f, fc_t)
        return list(self.last_feasible)

    # ------------------------------------------------------------- main loop
    def can_publish(self):
        if self.publish_only_when_hovering and self.flight_state != self.hover_flight_state:
            return False
        return self.candidate is not None

    def main(self):
        threading.Thread(target=self.key_loop, daemon=True).start()
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.candidate is None:
                # Initialize from the measured pose once available.
                init = self.measured if self.measured is not None else list(self.startup_pose)
                self.candidate = list(init)
                self.current_target = list(init)
                self.last_feasible = list(init)

            if self.can_publish():
                target = self.gate(self.candidate)
                limited = [c + float(np.clip(t - c, -self.max_step, self.max_step))
                           for c, t in zip(self.current_target, target)]
                self.current_target = limited

                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.name = list(self.joint_names)
                msg.position = list(limited)
                self.cmd_pub.publish(msg)
                self.shape_error_pub.publish(Float64MultiArray(
                    data=[c - t for c, t in zip(self.candidate, limited)]))
            rate.sleep()


if __name__ == "__main__":
    try:
        KeyboardBaseline().main()
    except rospy.ROSInterruptException:
        pass
