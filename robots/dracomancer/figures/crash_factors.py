#!/usr/bin/env python3
"""Plot non-fc factors around a DRAGON fall test rosbag."""

import argparse
import math
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


DRAGON_JOINTS = [
    "joint1_pitch",
    "joint1_yaw",
    "joint2_pitch",
    "joint2_yaw",
    "joint3_pitch",
    "joint3_yaw",
]

GIMBALS = [
    "gimbal1_pitch",
    "gimbal1_roll",
    "gimbal2_pitch",
    "gimbal2_roll",
    "gimbal3_pitch",
    "gimbal3_roll",
    "gimbal4_pitch",
    "gimbal4_roll",
]


def quat_to_rpy(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def arr(series):
    if series is None or len(series) == 0:
        return np.empty((0, 2), dtype=float)
    return np.asarray(series, dtype=float)


def nearest(series, t):
    data = arr(series)
    if len(data) == 0:
        return None
    idx = int(np.argmin(np.abs(data[:, 0] - t)))
    return float(data[idx, 1])


def finite_velocity(data):
    data = arr(data)
    if len(data) < 2:
        return np.empty((0, 2), dtype=float)
    dt = np.diff(data[:, 0])
    dv = np.diff(data[:, 1])
    mask = dt > 1.0e-6
    return np.column_stack(((data[1:, 0][mask] + data[:-1, 0][mask]) * 0.5, dv[mask] / dt[mask]))


def max_abs(series, start=None, end=None):
    data = arr(series)
    if len(data) == 0:
        return None
    if start is not None:
        data = data[data[:, 0] >= start]
    if end is not None:
        data = data[data[:, 0] <= end]
    if len(data) == 0:
        return None
    idx = int(np.argmax(np.abs(data[:, 1])))
    return float(data[idx, 1]), float(data[idx, 0])


def min_value(series, start=None, end=None):
    data = arr(series)
    if len(data) == 0:
        return None
    if start is not None:
        data = data[data[:, 0] >= start]
    if end is not None:
        data = data[data[:, 0] <= end]
    if len(data) == 0:
        return None
    idx = int(np.argmin(data[:, 1]))
    return float(data[idx, 1]), float(data[idx, 0])


def target_z_drop_intervals(nav_z, actual_z):
    nav = arr(nav_z)
    actual = arr(actual_z)
    if len(nav) == 0:
        return []
    mask = (nav[:, 0] > 40.0) & (nav[:, 1] < 0.8)
    if not mask.any():
        return []
    indices = np.where(mask)[0]
    groups = []
    current = [indices[0]]
    for idx in indices[1:]:
        if idx == current[-1] + 1:
            current.append(idx)
        else:
            groups.append(current)
            current = [idx]
    groups.append(current)

    intervals = []
    for group in groups:
        sub = nav[group]
        if sub[-1, 0] - sub[0, 0] < 0.2:
            continue
        k = int(np.argmin(sub[:, 1]))
        t_min = float(sub[k, 0])
        actual_at_min = nearest(actual, t_min)
        intervals.append((float(sub[0, 0]), float(sub[-1, 0]), t_min, float(sub[k, 1]), actual_at_min))
    return intervals


def interp_at(times, series):
    data = arr(series)
    if len(data) == 0:
        return np.full_like(times, np.nan, dtype=float)
    return np.interp(times, data[:, 0], data[:, 1])


def read_bag(bag_path):
    scalar = defaultdict(list)
    joint_pos = defaultdict(list)
    joint_vel = defaultdict(list)
    cmd_pos = defaultdict(list)
    gimbal_pos = defaultdict(list)
    gimbal_vel = defaultdict(list)
    flight_changes = []
    rosout = []

    topics = [
        "/dragon/ground_truth",
        "/dragon/uav/cog/odom",
        "/dragon/uav/baselink/odom",
        "/dragon/uav/nav",
        "/dragon/joints_ctrl",
        "/dragon/joint_states",
        "/dragon/gimbals_ctrl",
        "/dragon/debug/pose/pid",
        "/dragon/four_axes/command",
        "/dragon/motor_pwms",
        "/dragon/estimated_external_wrench",
        "/dragon/rotor_interfere_wrench",
        "/dragon/sensor_plugin/imu1/filter_angular_velocity",
        "/dragon/flight_state",
        "/rosout",
    ]

    with rosbag.Bag(bag_path) as bag:
        start = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(topics=topics):
            t = stamp.to_sec() - start

            if topic in ("/dragon/ground_truth", "/dragon/uav/cog/odom", "/dragon/uav/baselink/odom"):
                if topic == "/dragon/uav/cog/odom":
                    prefix = "cog"
                elif topic == "/dragon/uav/baselink/odom":
                    prefix = "baselink"
                else:
                    prefix = "ground_truth"
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                r, pch, y = quat_to_rpy(q.x, q.y, q.z, q.w)
                scalar[prefix + "_x"].append((t, p.x))
                scalar[prefix + "_y"].append((t, p.y))
                scalar[prefix + "_z"].append((t, p.z))
                scalar[prefix + "_roll"].append((t, r))
                scalar[prefix + "_pitch"].append((t, pch))
                scalar[prefix + "_yaw"].append((t, y))
                scalar[prefix + "_vz"].append((t, msg.twist.twist.linear.z))
                scalar[prefix + "_wx"].append((t, msg.twist.twist.angular.x))
                scalar[prefix + "_wy"].append((t, msg.twist.twist.angular.y))
                scalar[prefix + "_wz"].append((t, msg.twist.twist.angular.z))

            elif topic == "/dragon/uav/nav":
                scalar["nav_x"].append((t, msg.target_pos_x))
                scalar["nav_y"].append((t, msg.target_pos_y))
                scalar["nav_z"].append((t, msg.target_pos_z))
                scalar["nav_roll"].append((t, msg.target_roll))
                scalar["nav_pitch"].append((t, msg.target_pitch))
                scalar["nav_yaw"].append((t, msg.target_yaw))

            elif topic == "/dragon/joints_ctrl":
                for name, pos in zip(msg.name, msg.position):
                    if name in DRAGON_JOINTS:
                        cmd_pos[name].append((t, pos))

            elif topic == "/dragon/gimbals_ctrl":
                for name, pos in zip(msg.name, msg.position):
                    if name in GIMBALS:
                        gimbal_pos[name + "_cmd"].append((t, pos))

            elif topic == "/dragon/joint_states":
                for i, name in enumerate(msg.name):
                    if name in DRAGON_JOINTS:
                        joint_pos[name].append((t, msg.position[i]))
                        if i < len(msg.velocity):
                            joint_vel[name].append((t, msg.velocity[i]))
                    elif name in GIMBALS:
                        gimbal_pos[name].append((t, msg.position[i]))
                        if i < len(msg.velocity):
                            gimbal_vel[name].append((t, msg.velocity[i]))

            elif topic == "/dragon/debug/pose/pid":
                scalar["pid_z_total"].append((t, msg.z.total[0]))
                scalar["pid_z_err_p"].append((t, msg.z.err_p))
                scalar["pid_z_err_d"].append((t, msg.z.err_d))
                scalar["pid_roll_total"].append((t, msg.roll.total[0]))
                scalar["pid_pitch_total"].append((t, msg.pitch.total[0]))

            elif topic == "/dragon/four_axes/command":
                thrust = list(msg.base_thrust)
                if thrust:
                    scalar["thrust_sum"].append((t, sum(thrust)))
                    scalar["thrust_min"].append((t, min(thrust)))
                    scalar["thrust_max"].append((t, max(thrust)))

            elif topic == "/dragon/motor_pwms":
                values = list(msg.motor_value)
                if values:
                    scalar["pwm_min"].append((t, min(values)))
                    scalar["pwm_max"].append((t, max(values)))
                    scalar["pwm_span"].append((t, max(values) - min(values)))

            elif topic in ("/dragon/estimated_external_wrench", "/dragon/rotor_interfere_wrench"):
                key = "external" if topic.endswith("estimated_external_wrench") else "interfere"
                f = msg.wrench.force
                tau = msg.wrench.torque
                scalar[key + "_force_norm"].append((t, math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)))
                scalar[key + "_torque_norm"].append((t, math.sqrt(tau.x * tau.x + tau.y * tau.y + tau.z * tau.z)))

            elif topic == "/dragon/sensor_plugin/imu1/filter_angular_velocity":
                v = msg.vector
                scalar["imu_ang_vel_norm"].append((t, math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)))

            elif topic == "/dragon/flight_state":
                state = int(msg.data)
                if not flight_changes or flight_changes[-1][1] != state:
                    flight_changes.append((t, state))

            elif topic == "/rosout":
                text = getattr(msg, "msg", "")
                if any(k in text.lower() for k in ("failsafe", "land", "error", "warn")):
                    rosout.append((t, getattr(msg, "name", ""), text[:180]))

    return scalar, joint_pos, joint_vel, cmd_pos, gimbal_pos, gimbal_vel, flight_changes, rosout


def event_time(z_series):
    z = arr(z_series)
    if len(z) == 0:
        return None
    hovered = False
    for t, value in z:
        hovered = hovered or value > 0.8
        if hovered and value < 0.3:
            return float(t)
    idx = int(np.argmin(z[:, 1]))
    return float(z[idx, 0])


def write_summary(path, scalar, joint_pos, joint_vel, cmd_pos, gimbal_vel, flight_changes, rosout, event_t):
    lines = []
    lines.append("event_time_z_below_0.3: {:.3f}".format(event_t))
    lines.append("flight_changes: {}".format(flight_changes))

    for key in ["ground_truth_z", "cog_z", "nav_z"]:
        lines.append("{} min: {}".format(key, min_value(scalar[key])))
    lines.append("ground_truth_vz max_abs: {}".format(max_abs(scalar["ground_truth_vz"])))
    lines.append("cog_vz max_abs: {}".format(max_abs(scalar["cog_vz"])))

    window = (event_t - 4.0, event_t + 4.0)
    lines.append("")
    lines.append("event_window: {:.3f}..{:.3f}".format(*window))
    for key in ["ground_truth_z", "cog_z", "nav_z", "ground_truth_vz", "cog_vz",
                "ground_truth_roll", "ground_truth_pitch", "imu_ang_vel_norm",
                "pid_z_total", "thrust_sum", "pwm_span", "external_force_norm",
                "interfere_force_norm"]:
        lines.append("{} min: {} max_abs: {}".format(
            key, min_value(scalar[key], *window), max_abs(scalar[key], *window)))

    nav_z_vel = finite_velocity(scalar["nav_z"])
    lines.append("nav_z_rate max_abs all: {}".format(max_abs(nav_z_vel)))
    lines.append("nav_z_rate max_abs event: {}".format(max_abs(nav_z_vel, *window)))

    lines.append("")
    lines.append("target_z_drop_intervals start end t_min target_z_min actual_z_at_min:")
    for row in target_z_drop_intervals(scalar["nav_z"], scalar["ground_truth_z"]):
        lines.append("  ({:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f})".format(*row))

    lines.append("")
    lines.append("nav_z_rate_vs_joint_command_rate_event:")
    base = nav_z_vel[(nav_z_vel[:, 0] >= window[0]) & (nav_z_vel[:, 0] <= window[1])]
    for name in DRAGON_JOINTS:
        cmd_rate = finite_velocity(cmd_pos[name])
        if len(base) == 0 or len(cmd_rate) == 0:
            continue
        sampled = np.interp(base[:, 0], cmd_rate[:, 0], cmd_rate[:, 1])
        if np.std(base[:, 1]) < 1.0e-9 or np.std(sampled) < 1.0e-9:
            corr = float("nan")
        else:
            corr = float(np.corrcoef(base[:, 1], sampled)[0, 1])
        descending = base[:, 1] < -0.1
        max_rate = float(np.max(np.abs(sampled[descending]))) if descending.any() else float("nan")
        lines.append("  {} corr={:.3f} max_abs_rate_when_nav_descending={:.3f}".format(name, corr, max_rate))

    lines.append("")
    lines.append("joint_velocity_max_abs_all:")
    for name in DRAGON_JOINTS:
        lines.append("  {} actual={} command_rate={}".format(
            name, max_abs(joint_vel[name]), max_abs(finite_velocity(cmd_pos[name]))))

    lines.append("")
    lines.append("joint_velocity_max_abs_event:")
    for name in DRAGON_JOINTS:
        lines.append("  {} actual={} command_rate={}".format(
            name, max_abs(joint_vel[name], *window), max_abs(finite_velocity(cmd_pos[name]), *window)))

    lines.append("")
    lines.append("joint_tracking_error_event:")
    for name in DRAGON_JOINTS:
        actual = arr(joint_pos[name])
        cmd = arr(cmd_pos[name])
        if len(actual) == 0 or len(cmd) == 0:
            continue
        mask = (actual[:, 0] >= window[0]) & (actual[:, 0] <= window[1])
        t = actual[mask, 0]
        y = actual[mask, 1]
        if len(t) == 0:
            continue
        c = np.interp(t, cmd[:, 0], cmd[:, 1])
        err = y - c
        idx = int(np.argmax(np.abs(err)))
        lines.append("  {} max_abs_err={:.4f} at {:.3f} rmse={:.4f}".format(
            name, float(err[idx]), float(t[idx]), float(np.sqrt(np.mean(err * err)))))

    lines.append("")
    lines.append("gimbal_velocity_max_abs_event:")
    for name in GIMBALS:
        lines.append("  {} actual={}".format(name, max_abs(gimbal_vel[name], *window)))

    lines.append("")
    lines.append("nearest_values_at_event:")
    for key in ["ground_truth_z", "cog_z", "nav_z", "ground_truth_vz", "cog_vz",
                "ground_truth_roll", "ground_truth_pitch", "pid_z_total",
                "thrust_sum", "pwm_min", "pwm_max", "external_force_norm",
                "interfere_force_norm"]:
        lines.append("  {}: {}".format(key, nearest(scalar[key], event_t)))

    lines.append("")
    lines.append("rosout_key_messages:")
    for row in rosout[:40]:
        lines.append("  {:.3f} {} {}".format(row[0], row[1], row[2]))

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def plot_all(out_dir, scalar, joint_pos, joint_vel, cmd_pos, gimbal_vel, event_t):
    os.makedirs(out_dir, exist_ok=True)
    event_line = dict(color="#8c564b", linewidth=1.2, alpha=0.8)

    fig, axes = plt.subplots(5, 1, figsize=(14, 12), sharex=True)
    for key, label in [("ground_truth_z", "z actual"), ("nav_z", "z target"), ("cog_z", "cog z")]:
        data = arr(scalar[key])
        if len(data):
            axes[0].plot(data[:, 0], data[:, 1], label=label)
    axes[0].set_ylabel("z [m]")
    axes[0].legend(loc="best")

    for key, label in [("ground_truth_vz", "vz actual"), ("cog_vz", "vz cog")]:
        data = arr(scalar[key])
        if len(data):
            axes[1].plot(data[:, 0], data[:, 1], label=label)
    axes[1].set_ylabel("vertical velocity [m/s]")
    axes[1].legend(loc="best")

    for key, label in [("ground_truth_roll", "roll"), ("ground_truth_pitch", "pitch"),
                       ("imu_ang_vel_norm", "|omega|")]:
        data = arr(scalar[key])
        if len(data):
            axes[2].plot(data[:, 0], data[:, 1], label=label)
    axes[2].set_ylabel("attitude / omega")
    axes[2].legend(loc="best")

    for key, label in [("pid_z_total", "z pid total"), ("thrust_sum", "thrust sum"),
                       ("pwm_span", "pwm span")]:
        data = arr(scalar[key])
        if len(data):
            axes[3].plot(data[:, 0], data[:, 1], label=label)
    axes[3].set_ylabel("controller output")
    axes[3].legend(loc="best")

    for key, label in [("external_force_norm", "external force"), ("interfere_force_norm", "interference force")]:
        data = arr(scalar[key])
        if len(data):
            axes[4].plot(data[:, 0], data[:, 1], label=label)
    axes[4].set_ylabel("force norm")
    axes[4].set_xlabel("time from bag start [s]")
    axes[4].legend(loc="best")

    for ax in axes:
        ax.axvline(event_t, **event_line)
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "factors_overview.png"), dpi=180)
    fig.savefig(os.path.join(out_dir, "factors_overview.svg"))
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    for name in DRAGON_JOINTS:
        data = arr(joint_vel[name])
        if len(data):
            axes[0].plot(data[:, 0], data[:, 1], label=name)
        cmd_rate = finite_velocity(cmd_pos[name])
        if len(cmd_rate):
            axes[1].plot(cmd_rate[:, 0], cmd_rate[:, 1], label=name)
    axes[0].set_ylabel("actual joint velocity [rad/s]")
    axes[1].set_ylabel("command rate [rad/s]")
    axes[1].set_xlabel("time from bag start [s]")
    for ax in axes:
        ax.axvline(event_t, **event_line)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", ncol=3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "joint_velocity.png"), dpi=180)
    fig.savefig(os.path.join(out_dir, "joint_velocity.svg"))
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    for key, label in [("ground_truth_z", "z actual"), ("nav_z", "z target")]:
        data = arr(scalar[key])
        if len(data):
            axes[0].plot(data[:, 0], data[:, 1], label=label)
    for name in DRAGON_JOINTS:
        data = arr(joint_vel[name])
        if len(data):
            axes[1].plot(data[:, 0], data[:, 1], label=name)
    for name in GIMBALS:
        data = arr(gimbal_vel[name])
        if len(data):
            axes[2].plot(data[:, 0], data[:, 1], label=name)
    for key, label in [("pid_z_total", "z pid"), ("thrust_sum", "thrust sum"),
                       ("pwm_span", "pwm span")]:
        data = arr(scalar[key])
        if len(data):
            axes[3].plot(data[:, 0], data[:, 1], label=label)
    axes[0].set_ylabel("z [m]")
    axes[1].set_ylabel("joint vel [rad/s]")
    axes[2].set_ylabel("gimbal vel [rad/s]")
    axes[3].set_ylabel("control")
    axes[3].set_xlabel("time from bag start [s]")
    axes[0].set_xlim(event_t - 8.0, event_t + 8.0)
    for ax in axes:
        ax.axvline(event_t, **event_line)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", ncol=3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "event_window.png"), dpi=180)
    fig.savefig(os.path.join(out_dir, "event_window.svg"))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("-o", "--out-dir", default="eeb9_factors")
    args = parser.parse_args()

    scalar, joint_pos, joint_vel, cmd_pos, gimbal_pos, gimbal_vel, flight_changes, rosout = read_bag(args.bag)
    t_event = event_time(scalar["ground_truth_z"])
    if t_event is None:
        raise RuntimeError("ground_truth_z is empty")
    os.makedirs(args.out_dir, exist_ok=True)
    write_summary(
        os.path.join(args.out_dir, "summary.txt"),
        scalar, joint_pos, joint_vel, cmd_pos, gimbal_vel, flight_changes, rosout, t_event)
    plot_all(args.out_dir, scalar, joint_pos, joint_vel, cmd_pos, gimbal_vel, t_event)
    print("wrote {}".format(args.out_dir))
    print("event_time {:.3f}".format(t_event))


if __name__ == "__main__":
    main()
