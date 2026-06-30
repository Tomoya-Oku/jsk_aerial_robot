#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Part A: shape-sweep data collection for fc threshold calibration.

Drives a *hovering* DRAGON through a set of internal joint configurations and
records, for each settled shape, the feasible-control inradius fc_f_min /
fc_t_min together with the commanded / measured joint angles and the COG
altitude. The output CSV feeds analyze_fc_data.py, which proposes the
hard_min / min thresholds for the Dracomancer safety gate.

fc is a property of the shape alone (rotor geometry + thrust limits), so this
sweep — not any external disturbance — is what reveals the fc distribution and
the "feasibility cliff". Run while DRAGON hovers (flight_state == 5).

Safety: aborts if the COG altitude falls below abort_altitude (shape made the
robot lose control), restoring the nominal shape before exiting.
"""

import os
import csv
import rospy
import numpy as np
from std_msgs.msg import Float64, UInt8
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class FcDataCollector:
    def __init__(self):
        rospy.init_node("collect_fc_data")

        self.robot = rospy.get_param("~robot_name", "dragon")
        self.joint_names = rospy.get_param("~joint_names", [
            "joint1_pitch", "joint1_yaw",
            "joint2_pitch", "joint2_yaw",
            "joint3_pitch", "joint3_yaw"])
        self.nominal = rospy.get_param("~nominal_pose",
                                       [0.0, np.pi / 2, 0.0, np.pi / 2, 0.0, np.pi / 2])
        # Sweep ranges per joint (used by the 1-D sweeps and random sampling).
        self.pitch_range = rospy.get_param("~pitch_range", [-0.6, 0.6])
        self.yaw_range = rospy.get_param("~yaw_range", [0.0, np.pi / 2])
        self.sample_mode = str(rospy.get_param("~sample_mode", "sweep_random")).lower()
        if self.sample_mode not in ("sweep_random", "grid"):
            rospy.logwarn("unknown sample_mode '%s'; fall back to sweep_random", self.sample_mode)
            self.sample_mode = "sweep_random"
        self.n_1d = rospy.get_param("~n_points_1d", 9)      # samples per 1-D joint sweep
        self.n_random = rospy.get_param("~n_random", 60)    # random multi-joint samples
        self.n_grid = rospy.get_param("~n_points_grid", 4)  # samples per joint for full grid
        self.grid_start = int(rospy.get_param("~grid_start", 0))
        self.grid_count = int(rospy.get_param("~grid_count", -1))
        self.settle_time = rospy.get_param("~settle_time", 3.0)
        self.sample_time = rospy.get_param("~sample_time", 1.0)
        self.max_step = rospy.get_param("~max_step", 0.05)   # rad per cmd, gradual move
        self.cmd_rate = rospy.get_param("~cmd_rate", 20.0)
        self.abort_altitude = rospy.get_param("~abort_altitude", 0.4)
        self.seed = rospy.get_param("~seed", 0)
        out_default = os.path.expanduser("~/dracomancer_fc_data.csv")
        self.out_path = rospy.get_param("~out_path", out_default)

        self.is_pitch = ["pitch" in n for n in self.joint_names]

        self.fc_f = None
        self.fc_t = None
        self.meas_joints = {}
        self.altitude = None
        self.flight_state = None

        self.cmd_pub = rospy.Publisher("/" + self.robot + "/joints_ctrl", JointState, queue_size=1)
        rospy.Subscriber("/" + self.robot + "/debug/fc_f_min", Float64, self._fc_f_cb, queue_size=1)
        rospy.Subscriber("/" + self.robot + "/debug/fc_t_min", Float64, self._fc_t_cb, queue_size=1)
        rospy.Subscriber("/" + self.robot + "/joint_states", JointState, self._js_cb, queue_size=1)
        rospy.Subscriber("/" + self.robot + "/uav/cog/odom", Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber("/" + self.robot + "/flight_state", UInt8, self._fs_cb, queue_size=1)

        self.current_cmd = list(self.nominal)
        self.rows = []
        rospy.sleep(1.0)

    def _fc_f_cb(self, m): self.fc_f = float(m.data)
    def _fc_t_cb(self, m): self.fc_t = float(m.data)
    def _odom_cb(self, m): self.altitude = m.pose.pose.position.z
    def _fs_cb(self, m): self.flight_state = int(m.data)

    def _js_cb(self, m):
        d = dict(zip(m.name, m.position))
        self.meas_joints = {n: d[n] for n in self.joint_names if n in d}

    def _publish_cmd(self, target):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = list(target)
        self.cmd_pub.publish(msg)

    def move_to(self, target):
        """Move gradually to target shape, rate-limited, then let it settle.
        Returns False if an abort condition (low altitude) is hit."""
        rate = rospy.Rate(self.cmd_rate)
        while not rospy.is_shutdown():
            done = True
            for i in range(len(target)):
                delta = target[i] - self.current_cmd[i]
                if abs(delta) > self.max_step:
                    delta = self.max_step if delta > 0 else -self.max_step
                    done = False
                self.current_cmd[i] += delta
            self._publish_cmd(self.current_cmd)
            if self.altitude is not None and self.altitude < self.abort_altitude:
                rospy.logwarn("ABORT: altitude %.2f < %.2f", self.altitude, self.abort_altitude)
                return False
            if done:
                break
            rate.sleep()
        # settle
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < self.settle_time and not rospy.is_shutdown():
            self._publish_cmd(self.current_cmd)
            if self.altitude is not None and self.altitude < self.abort_altitude:
                rospy.logwarn("ABORT during settle: altitude %.2f", self.altitude)
                return False
            rospy.sleep(0.05)
        return True

    def sample(self, label):
        """Average fc over sample_time and append a row."""
        fs, ts, alt = [], [], []
        t0 = rospy.Time.now()
        r = rospy.Rate(self.cmd_rate)
        while (rospy.Time.now() - t0).to_sec() < self.sample_time and not rospy.is_shutdown():
            self._publish_cmd(self.current_cmd)
            if self.fc_f is not None: fs.append(self.fc_f)
            if self.fc_t is not None: ts.append(self.fc_t)
            if self.altitude is not None: alt.append(self.altitude)
            r.sleep()
        if not fs or not ts:
            rospy.logwarn("no fc samples for %s", label)
            return
        row = {
            "label": label,
            "fc_f_min_mean": float(np.mean(fs)), "fc_f_min_std": float(np.std(fs)),
            "fc_t_min_mean": float(np.mean(ts)), "fc_t_min_std": float(np.std(ts)),
            "altitude_mean": float(np.mean(alt)) if alt else float("nan"),
        }
        for n in self.joint_names:
            row["cmd_" + n] = self.current_cmd[self.joint_names.index(n)]
            row["meas_" + n] = self.meas_joints.get(n, float("nan"))
        self.rows.append(row)
        rospy.loginfo("[%s] fc_f=%.4f fc_t=%.4f alt=%.2f",
                      label, row["fc_f_min_mean"], row["fc_t_min_mean"], row["altitude_mean"])

    def joint_value_range(self, idx):
        lo, hi = (self.pitch_range if self.is_pitch[idx] else self.yaw_range)
        return np.linspace(lo, hi, self.n_1d)

    def joint_grid_range(self, idx):
        lo, hi = (self.pitch_range if self.is_pitch[idx] else self.yaw_range)
        return np.linspace(lo, hi, self.n_grid)

    def grid_targets(self, ranges, prefix=None, reverse=False):
        """Yield full-grid targets in a serpentine order to reduce large jumps."""
        prefix = [] if prefix is None else prefix
        if not ranges:
            yield list(prefix)
            return
        vals = list(ranges[0])
        if reverse:
            vals.reverse()
        child_reverse = False
        for v in vals:
            for target in self.grid_targets(ranges[1:], prefix + [float(v)], child_reverse):
                yield target
            child_reverse = not child_reverse

    def run(self):
        if self.flight_state != 5:
            rospy.logwarn("flight_state=%s (expected 5/HOVER). Continuing anyway.", self.flight_state)

        if self.sample_mode == "grid":
            ranges = [self.joint_grid_range(idx) for idx in range(len(self.joint_names))]
            total = int(np.prod([len(r) for r in ranges]))
            start = max(0, min(self.grid_start, total))
            stop = total if self.grid_count < 0 else min(total, start + max(0, self.grid_count))
            rospy.loginfo("sample_mode=grid: %d joints x %d points -> %d samples, range [%d, %d)",
                          len(self.joint_names), self.n_grid, total, start, stop)
            for k, target in enumerate(self.grid_targets(ranges)):
                if k < start:
                    continue
                if k >= stop:
                    break
                if not self.move_to(target):
                    self.finish(); return
                self.sample("grid_%04d" % k)
                done = k - start + 1
                if done % 100 == 0:
                    rospy.loginfo("grid progress: %d/%d samples in assigned range", done, stop - start)
        else:
            # Baseline at nominal.
            if not self.move_to(self.nominal):
                self.finish(); return
            self.sample("nominal")

            # 1-D sweeps: vary one joint across its range, others at nominal.
            for idx, name in enumerate(self.joint_names):
                for v in self.joint_value_range(idx):
                    target = list(self.nominal)
                    target[idx] = float(v)
                    if not self.move_to(target):
                        self.finish(); return
                    self.sample("sweep_%s_%.3f" % (name, v))
                # return to nominal between joints to stay near a feasible region
                self.move_to(self.nominal)

            # Random multi-joint samples within ranges.
            rng = np.random.default_rng(self.seed)
            for k in range(self.n_random):
                target = []
                for idx in range(len(self.joint_names)):
                    lo, hi = (self.pitch_range if self.is_pitch[idx] else self.yaw_range)
                    target.append(float(rng.uniform(lo, hi)))
                if not self.move_to(target):
                    self.finish(); return
                self.sample("random_%03d" % k)
                if k % 10 == 9:
                    self.move_to(self.nominal)  # periodically recover

        self.move_to(self.nominal)
        self.finish()

    def finish(self):
        if not self.rows:
            rospy.logwarn("no rows collected"); return
        keys = list(self.rows[0].keys())
        with open(self.out_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for r in self.rows:
                w.writerow(r)
        rospy.loginfo("wrote %d rows to %s", len(self.rows), self.out_path)


if __name__ == "__main__":
    try:
        FcDataCollector().run()
    except rospy.ROSInterruptException:
        pass
