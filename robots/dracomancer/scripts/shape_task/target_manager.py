#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Shape-target reaching task manager.

Runs the trial state machine for the Dracomancer -> DRAGON shape-target
reaching experiment: selects a target shape per trial, evaluates the success
condition (E_q / E_s / safety margin mu held for t_hold seconds), enforces the
timeout, and broadcasts trial boundaries so that task_recorder.py and
shadow_visualizer.py stay condition-agnostic (Dracomancer vs keyboard input
differ only in who publishes /dragon/joints_ctrl).

Interfaces (all names configurable; defaults under <device_ns>/shape_task):
  pub  ~/target (sensor_msgs/JointState, latched) : q_star, frame_id=target name
  pub  ~/status (std_msgs/String, JSON)           : live state for UI/visualizer
  pub  ~/event  (std_msgs/String, JSON, latched)  : session/trial start-end events
  srv  ~start / ~stop / ~reset / ~next (std_srvs/Trigger)

Times (elapsed, hold, timeout) use ROS time; only the results directory name
uses wall time.
"""

import datetime
import json
import os
import random

import numpy as np
import rospy
import yaml
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String
from std_srvs.srv import Trigger, TriggerResponse

import dragon_fk

DEFAULT_TARGETS = [
    {"name": "L-like", "q_star": [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]},
    {"name": "Zigzag-like", "q_star": [0.0, -1.0, 0.0, 0.0, 0.0, 1.0]},
    {"name": "Spiral-like", "q_star": [-0.5, 0.8, -0.5, 0.8, -0.5, 0.8]},
]


class TargetManager:
    def __init__(self):
        rospy.init_node("target_manager")

        self.robot_ns = rospy.get_param("~robot_ns", "dragon")
        self.device_ns = rospy.get_param("~device_ns", "/dracomancer").rstrip("/")
        base = self.device_ns + "/shape_task"
        topics = rospy.get_param("~topics", {})
        if not isinstance(topics, dict):
            topics = {}
        self.q_current_topic = str(topics.get("q_current", "/%s/joint_states" % self.robot_ns))
        self.r_f_topic = str(topics.get("r_f", "/%s/debug/fc_f_min_filtered" % self.robot_ns))
        self.r_tau_topic = str(topics.get("r_tau", "/%s/debug/fc_t_min_filtered" % self.robot_ns))
        self.force_threshold_topic = str(topics.get(
            "force_threshold", self.device_ns + "/force_volume_radius_threshold"))
        self.torque_threshold_topic = str(topics.get(
            "torque_threshold", self.device_ns + "/torque_volume_radius_threshold"))

        self.joint_names = rospy.get_param("~dragon_joint_names", list(dragon_fk.JOINT_ORDER))

        # Success condition.
        self.eq_threshold = float(rospy.get_param("~E_q_threshold", 0.15))
        self.es_threshold = float(rospy.get_param("~E_s_threshold", 0.05))
        self.t_hold = float(rospy.get_param("~t_hold", 1.0))
        self.timeout = float(rospy.get_param("~timeout", 60.0))
        self.require_safety_margin = bool(rospy.get_param("~require_safety_margin", True))

        # Safety thresholds ([hard, safe]); overridden by threshold topics.
        self.r_f_hard = float(rospy.get_param("~r_f_hard", 0.108990))
        self.r_f_safe = float(rospy.get_param("~r_f_safe", 0.249220))
        self.r_tau_hard = float(rospy.get_param("~r_tau_hard", 0.015400))
        self.r_tau_safe = float(rospy.get_param("~r_tau_safe", 0.278159))
        self.use_threshold_topics = bool(rospy.get_param("~use_threshold_topics", True))

        # Trial flow.
        self.trial_order = str(rospy.get_param("~trial_order", "sequential")).lower()
        self.repetitions = int(rospy.get_param("~repetitions", 1))
        self.inter_trial_pause = float(rospy.get_param("~inter_trial_pause", 3.0))
        self.start_delay = float(rospy.get_param("~start_delay", 2.0))
        self.seed = int(rospy.get_param("~seed", 0))
        self.auto_start = bool(rospy.get_param("~auto_start", False))
        self.publish_preview_target = bool(rospy.get_param("~publish_preview_target", True))

        # FK for E_s.
        self.link_length = float(rospy.get_param("~link_length", dragon_fk.DEFAULT_LINK_LENGTH))
        self.inter_joint_x_offset = float(rospy.get_param(
            "~inter_joint_x_offset", dragon_fk.DEFAULT_INTER_JOINT_X_OFFSET))
        self.es_normalize = bool(rospy.get_param("~E_s_normalize", False))

        self.status_rate = float(rospy.get_param("~status_rate", 10.0))
        self.stale_timeout = float(rospy.get_param("~stale_timeout", 1.0))
        self.results_root = os.path.expanduser(
            str(rospy.get_param("~results_root", "~/dracomancer_shape_task")))
        # Explicit results_dir (arg) beats results_root/<timestamp>.
        self.results_dir_param = str(rospy.get_param("~results_dir", "")).strip()

        self.targets = rospy.get_param("~targets", DEFAULT_TARGETS)
        self.schedule = self.build_schedule()

        # Latest topic values as (value, ros stamp).
        self.q_current = None
        self.q_current_stamp = 0.0
        self.r_f = None
        self.r_f_stamp = 0.0
        self.r_tau = None
        self.r_tau_stamp = 0.0

        # State machine: idle -> countdown -> running -> (countdown ...) -> finished
        self.state = "idle"
        self.sched_idx = 0
        self.trial_no = 0
        self.trial_start_t = None
        self.hold_start_t = None
        self.countdown_end_t = None
        self.current_target = None
        self.results_dir = None
        self.last_result = None  # dict of the last trial_end payload (for status)

        self.target_pub = rospy.Publisher(base + "/target", JointState, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(base + "/status", String, queue_size=1)
        self.event_pub = rospy.Publisher(base + "/event", String, queue_size=10, latch=True)

        rospy.Subscriber(self.q_current_topic, JointState, self.q_current_cb, queue_size=1)
        rospy.Subscriber(self.r_f_topic, Float64, self.r_f_cb, queue_size=1)
        rospy.Subscriber(self.r_tau_topic, Float64, self.r_tau_cb, queue_size=1)
        if self.use_threshold_topics:
            rospy.Subscriber(self.force_threshold_topic, Float64MultiArray,
                             self.force_threshold_cb, queue_size=1)
            rospy.Subscriber(self.torque_threshold_topic, Float64MultiArray,
                             self.torque_threshold_cb, queue_size=1)

        rospy.Service("~start", Trigger, self.start_srv)
        rospy.Service("~stop", Trigger, self.stop_srv)
        rospy.Service("~reset", Trigger, self.reset_srv)
        rospy.Service("~next", Trigger, self.next_srv)

        rospy.loginfo("shape task manager: %d trials (%d targets x %d reps, order=%s)",
                      len(self.schedule), len(self.targets), self.repetitions, self.trial_order)
        rospy.loginfo("success: E_q<%.3f rad, E_s<%.3f %s, mu>=0(%s), hold %.1fs, timeout %.1fs",
                      self.eq_threshold, self.es_threshold,
                      "(normalized)" if self.es_normalize else "m",
                      self.require_safety_margin, self.t_hold, self.timeout)

        if self.auto_start:
            self.begin_session()
            self.state = "countdown"
            self.countdown_end_t = rospy.get_time() + self.start_delay
        elif self.publish_preview_target:
            self.publish_scheduled_target()

        period = 1.0 / max(self.status_rate, 1.0)
        rospy.Timer(rospy.Duration(period), self.tick)

    # ------------------------------------------------------------------ setup
    def build_schedule(self):
        targets = []
        for t in self.targets:
            name = str(t.get("name", "target%d" % len(targets)))
            q_star = [float(v) for v in t.get("q_star", [])]
            if len(q_star) != len(self.joint_names):
                rospy.logwarn("target '%s': q_star has %d values (expected %d); skipped",
                              name, len(q_star), len(self.joint_names))
                continue
            targets.append({"name": name, "q_star": q_star})
        if not targets:
            rospy.logerr("no valid targets; nothing to do")
        schedule = []
        rng = random.Random(self.seed)
        for _ in range(max(1, self.repetitions)):
            block = list(targets)
            if self.trial_order == "random":
                rng.shuffle(block)
            schedule.extend(block)
        return schedule

    def begin_session(self):
        if self.results_dir is not None:
            return
        if self.results_dir_param:
            self.results_dir = os.path.expanduser(self.results_dir_param)
        else:
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.results_dir = os.path.join(self.results_root, stamp)
        try:
            os.makedirs(self.results_dir, exist_ok=True)
            self.dump_session_yaml()
        except OSError as e:
            rospy.logerr("cannot create results dir '%s': %s", self.results_dir, e)
        self.publish_event({"event": "session_start", "results_dir": self.results_dir})
        rospy.loginfo("shape task session -> %s", self.results_dir)

    def dump_session_yaml(self):
        # Effective config snapshot, consumed by figures/plot_task.py for
        # threshold lines and kept as the experiment record.
        session = {
            "stamp": datetime.datetime.now().isoformat(),
            "targets": self.schedule,
            "params": {
                "E_q_threshold": self.eq_threshold,
                "E_s_threshold": self.es_threshold,
                "E_s_normalize": self.es_normalize,
                "t_hold": self.t_hold,
                "timeout": self.timeout,
                "require_safety_margin": self.require_safety_margin,
                "r_f_hard": self.r_f_hard,
                "r_f_safe": self.r_f_safe,
                "r_tau_hard": self.r_tau_hard,
                "r_tau_safe": self.r_tau_safe,
                "link_length": self.link_length,
                "inter_joint_x_offset": self.inter_joint_x_offset,
                "trial_order": self.trial_order,
                "repetitions": self.repetitions,
                "joint_names": self.joint_names,
            },
        }
        with open(os.path.join(self.results_dir, "session.yaml"), "w") as f:
            yaml.safe_dump(session, f, allow_unicode=True, default_flow_style=None)

    # ------------------------------------------------------------- callbacks
    def q_current_cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        vals = [pos.get(n) for n in self.joint_names]
        if any(v is None for v in vals):
            return  # not the DRAGON internal-joint message
        self.q_current = [float(v) for v in vals]
        self.q_current_stamp = rospy.get_time()

    def r_f_cb(self, msg):
        self.r_f = float(msg.data)
        self.r_f_stamp = rospy.get_time()

    def r_tau_cb(self, msg):
        self.r_tau = float(msg.data)
        self.r_tau_stamp = rospy.get_time()

    def force_threshold_cb(self, msg):
        if len(msg.data) > 1:
            self.r_f_hard, self.r_f_safe = float(msg.data[0]), float(msg.data[1])

    def torque_threshold_cb(self, msg):
        if len(msg.data) > 1:
            self.r_tau_hard, self.r_tau_safe = float(msg.data[0]), float(msg.data[1])

    # -------------------------------------------------------------- services
    def start_srv(self, _req):
        if self.state == "running":
            return TriggerResponse(False, "already running")
        if self.sched_idx >= len(self.schedule):
            return TriggerResponse(False, "schedule finished; call ~reset first")
        self.begin_session()
        self.state = "countdown"
        self.countdown_end_t = rospy.get_time() + self.start_delay
        return TriggerResponse(True, "trial %d starts in %.1fs" %
                               (self.trial_no + 1, self.start_delay))

    def stop_srv(self, _req):
        if self.state == "running":
            self.end_trial(False, "aborted")
            self.state = "idle"  # do not auto-advance after a manual stop
            return TriggerResponse(True, "trial aborted")
        if self.state == "countdown":
            self.state = "idle"
            return TriggerResponse(True, "countdown cancelled")
        return TriggerResponse(False, "not running")

    def reset_srv(self, _req):
        if self.state == "running":
            self.end_trial(False, "reset")
        self.state = "idle"
        self.sched_idx = 0
        return TriggerResponse(True, "schedule reset to the first target "
                                     "(trial numbering keeps increasing)")

    def next_srv(self, _req):
        if self.state == "running":
            self.end_trial(False, "skipped")
            # end_trial already advanced and scheduled the countdown
            return TriggerResponse(True, "trial skipped")
        if self.sched_idx < len(self.schedule) - 1 and self.state in ("idle", "countdown"):
            self.sched_idx += 1
            if self.publish_preview_target:
                self.publish_scheduled_target()
            return TriggerResponse(True, "next target: %s" %
                                   self.schedule[self.sched_idx]["name"])
        return TriggerResponse(False, "no next target")

    # ------------------------------------------------------------ evaluation
    def fresh(self, value, stamp):
        if value is None:
            return None
        if self.stale_timeout > 0.0 and rospy.get_time() - stamp > self.stale_timeout:
            return None
        return value

    def safety_margin(self):
        """mu = min of the normalized force/torque margins; NaN when unknown."""
        r_f = self.fresh(self.r_f, self.r_f_stamp)
        r_tau = self.fresh(self.r_tau, self.r_tau_stamp)
        if r_f is None or r_tau is None:
            return float("nan")
        f_den = max(self.r_f_safe - self.r_f_hard, 1e-9)
        t_den = max(self.r_tau_safe - self.r_tau_hard, 1e-9)
        return float(min((r_f - self.r_f_hard) / f_den, (r_tau - self.r_tau_hard) / t_den))

    def current_errors(self):
        q = self.fresh(self.q_current, self.q_current_stamp)
        if q is None or self.current_target is None:
            return float("nan"), float("nan")
        q_star = self.current_target["q_star"]
        e_q = dragon_fk.joint_error(q, q_star)
        e_s = dragon_fk.shape_error(q, q_star, self.link_length,
                                    self.inter_joint_x_offset, self.es_normalize)
        return e_q, e_s

    # ---------------------------------------------------------- state machine
    def tick(self, _event):
        now = rospy.get_time()
        e_q, e_s = self.current_errors()
        mu = self.safety_margin()

        if self.state == "countdown" and now >= (self.countdown_end_t or now):
            self.begin_trial()

        in_tolerance = False
        if self.state == "running":
            in_tolerance = (not np.isnan(e_q) and e_q < self.eq_threshold and
                            not np.isnan(e_s) and e_s < self.es_threshold)
            if self.require_safety_margin:
                if np.isnan(mu):
                    rospy.logwarn_throttle(
                        5.0, "safety margin unavailable (fc topics silent); "
                             "success blocked. Set require_safety_margin:=false for dry-run.")
                    in_tolerance = False
                else:
                    in_tolerance = in_tolerance and mu >= 0.0

            if in_tolerance:
                if self.hold_start_t is None:
                    self.hold_start_t = now
                elif now - self.hold_start_t >= self.t_hold:
                    self.end_trial(True, "success")
            else:
                self.hold_start_t = None

            if self.state == "running" and now - self.trial_start_t >= self.timeout:
                self.end_trial(False, "timeout")

        self.publish_status(now, e_q, e_s, mu, in_tolerance)

    def begin_trial(self):
        if self.sched_idx >= len(self.schedule):
            self.state = "finished"
            return
        self.current_target = self.schedule[self.sched_idx]
        self.trial_no += 1
        self.trial_start_t = rospy.get_time()
        self.hold_start_t = None
        self.state = "running"

        self.publish_target(self.current_target)

        self.publish_event({
            "event": "trial_start",
            "trial": self.trial_no,
            "target_name": self.current_target["name"],
            "q_star": self.current_target["q_star"],
            "results_dir": self.results_dir,
            "timeout": self.timeout,
        })
        rospy.loginfo("trial %d start: %s q_star=%s", self.trial_no,
                      self.current_target["name"],
                      np.round(self.current_target["q_star"], 3).tolist())

    def end_trial(self, success, reason):
        now = rospy.get_time()
        completion = now - self.trial_start_t if success else float("nan")
        e_q, e_s = self.current_errors()
        payload = {
            "event": "trial_end",
            "trial": self.trial_no,
            "target_name": self.current_target["name"] if self.current_target else "",
            "success": bool(success),
            "reason": reason,
            "completion_time": completion,
            "final_E_q": e_q,
            "final_E_s": e_s,
        }
        self.publish_event(payload)
        self.last_result = payload
        (rospy.loginfo if success else rospy.logwarn)(
            "trial %d end: %s (%s) completion=%.2fs E_q=%.3f E_s=%.3f",
            self.trial_no, "SUCCESS" if success else "FAIL", reason,
            completion if success else -1.0,
            e_q if not np.isnan(e_q) else -1.0,
            e_s if not np.isnan(e_s) else -1.0)

        self.sched_idx += 1
        self.current_target = None
        if self.sched_idx >= len(self.schedule):
            self.state = "finished"
            self.publish_event({"event": "session_end", "results_dir": self.results_dir})
            rospy.loginfo("all %d trials finished; results in %s",
                          len(self.schedule), self.results_dir)
        elif reason in ("aborted", "reset"):
            self.state = "idle"
            if self.publish_preview_target:
                self.publish_scheduled_target()
        else:
            self.state = "countdown"
            self.countdown_end_t = now + self.inter_trial_pause
            if self.publish_preview_target:
                self.publish_scheduled_target()

    # ------------------------------------------------------------- publishing
    def publish_target(self, target):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = target["name"]
        msg.name = list(self.joint_names)
        msg.position = list(target["q_star"])
        self.target_pub.publish(msg)

    def publish_scheduled_target(self):
        if self.sched_idx < len(self.schedule):
            self.publish_target(self.schedule[self.sched_idx])

    def publish_event(self, payload):
        payload["stamp"] = rospy.get_time()
        self.event_pub.publish(String(json.dumps(payload)))

    def publish_status(self, now, e_q, e_s, mu, in_tolerance):
        status = {
            "state": self.state,
            "trial": self.trial_no,
            "target_name": self.current_target["name"] if self.current_target else "",
            "elapsed": now - self.trial_start_t if self.state == "running" else 0.0,
            "remaining": (self.timeout - (now - self.trial_start_t)
                          if self.state == "running" else 0.0),
            "E_q": e_q,
            "E_s": e_s,
            "mu": mu,
            "in_tolerance": bool(in_tolerance),
            "hold": (now - self.hold_start_t
                     if (self.state == "running" and self.hold_start_t is not None) else 0.0),
            "t_hold": self.t_hold,
            "E_q_threshold": self.eq_threshold,
            "E_s_threshold": self.es_threshold,
            "last_success": (self.last_result or {}).get("success"),
        }
        # NaN is not valid JSON; encode as null.
        text = json.dumps({k: (None if isinstance(v, float) and np.isnan(v) else v)
                           for k, v in status.items()})
        self.status_pub.publish(String(text))


if __name__ == "__main__":
    try:
        TargetManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
