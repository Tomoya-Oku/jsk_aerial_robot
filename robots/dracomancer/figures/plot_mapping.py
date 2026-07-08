#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Plot Dracomancer-to-DRAGON mapping traces from a rosbag.

The generated figures are intentionally offline artifacts: they read only the
selected bag topics and write PNG/SVG files, so the exact plot set can be
regenerated without replaying ROS nodes.
"""

import argparse
import bisect
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


BAG_DEFAULT = os.path.expanduser("~/rosbags/20260708_151650_joint_angles_mapping.bag")
OUT_DEFAULT = os.path.join(os.path.dirname(__file__), "joint_angles_mapping")

DRAGON_JOINTS = [
    "joint1_pitch",
    "joint1_yaw",
    "joint2_pitch",
    "joint2_yaw",
    "joint3_pitch",
    "joint3_yaw",
]

SOURCE_RELATIONS = {
    "joint1_pitch": [
        "wrist_flexion_extension_joint",
        "wrist_abduction_adduction_joint",
        "upper_arm_external_internal_rotation_joint",
        "wrist_supination_joint",
    ],
    "joint1_yaw": [
        "wrist_flexion_extension_joint",
        "wrist_abduction_adduction_joint",
        "upper_arm_external_internal_rotation_joint",
        "wrist_supination_joint",
    ],
    "joint2_pitch": [
        "elbow_flexion_extension_joint",
        "upper_arm_external_internal_rotation_joint",
    ],
    "joint2_yaw": [
        "elbow_flexion_extension_joint",
        "upper_arm_external_internal_rotation_joint",
    ],
    "joint3_pitch": ["shoulder_flexion_extension_joint"],
    "joint3_yaw": ["shoulder_abduction_adduction_joint"],
}

PRIMARY_SOURCE = {
    "joint1_pitch": "wrist_flexion_extension_joint",
    "joint1_yaw": "wrist_abduction_adduction_joint",
    "joint2_pitch": "elbow_flexion_extension_joint",
    "joint2_yaw": "elbow_flexion_extension_joint",
    "joint3_pitch": "shoulder_flexion_extension_joint",
    "joint3_yaw": "shoulder_abduction_adduction_joint",
}

TOPICS = [
    "/dracomancer/joint_states",
    "/dracomancer/candidate/joint_target",
    "/dragon/joints_ctrl",
    "/dragon/joint_states",
    "/dragon/uav/cog/odom",
    "/gazebo/link_states",
]


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", nargs="?", default=BAG_DEFAULT, help="input rosbag path")
    parser.add_argument("-o", "--out-dir", default=OUT_DEFAULT, help="output directory")
    parser.add_argument("--formats", nargs="+", default=["png", "svg"],
                        choices=["png", "svg", "pdf"], help="figure formats")
    parser.add_argument("--link4-name", default="dragon::link4",
                        help="gazebo link_states name for link4")
    parser.add_argument("--max-position-points", type=int, default=12000,
                        help="maximum points kept for position plots")
    return parser.parse_args()


def setup_style():
    matplotlib.rcParams.update({
        "font.family": "Times New Roman",
        "font.size": 14,
        "axes.titlesize": 15,
        "axes.labelsize": 14,
        "xtick.labelsize": 12,
        "ytick.labelsize": 12,
        "legend.fontsize": 10,
        "svg.fonttype": "none",
    })


def ensure_dir(path):
    if not os.path.isdir(path):
        os.makedirs(path)


def save(fig, out_dir, name, formats):
    paths = []
    fig.tight_layout()
    for fmt in formats:
        path = os.path.join(out_dir, "%s.%s" % (name, fmt))
        fig.savefig(path, dpi=180)
        paths.append(path)
    plt.close(fig)
    return paths


def stamp_time(stamp, fallback_stamp):
    t = stamp.to_sec()
    if t > 0.0:
        return t
    return fallback_stamp.to_sec()


def as_joint_dict(msg):
    return {name: float(pos) for name, pos in zip(msg.name, msg.position)}


def pose_row(t, position):
    return {
        "t": float(t),
        "x": float(position.x),
        "y": float(position.y),
        "z": float(position.z),
    }


def decimated_append(rows, item, max_points):
    rows.append(item)
    if max_points <= 0 or len(rows) <= max_points:
        return
    # Keep endpoints and reduce every other sample. This bounds memory/plot size
    # while preserving the trajectory envelope for long Gazebo traces.
    rows[:] = rows[::2]


def collect_bag(bag_path, link4_name, max_position_points):
    import rosbag

    data = {
        "/dracomancer/joint_states": [],
        "/dracomancer/candidate/joint_target": [],
        "/dragon/joints_ctrl": [],
        "/dragon/joint_states": [],
        "/dragon/uav/cog/odom": [],
        "link4": [],
    }
    stats = {
        "start_time": None,
        "end_time": None,
        "raw_counts": {topic: 0 for topic in TOPICS},
        "link4_name": link4_name,
    }

    with rosbag.Bag(bag_path) as bag:
        start = bag.get_start_time()
        stats["start_time"] = start
        stats["end_time"] = bag.get_end_time()
        link4_index = None
        for topic, msg, stamp in bag.read_messages(topics=TOPICS):
            stats["raw_counts"][topic] += 1
            t = stamp_time(getattr(msg, "header", None).stamp, stamp) - start if hasattr(msg, "header") else stamp.to_sec() - start
            if topic in (
                    "/dracomancer/joint_states",
                    "/dracomancer/candidate/joint_target",
                    "/dragon/joints_ctrl",
                    "/dragon/joint_states"):
                data[topic].append((t, as_joint_dict(msg)))
            elif topic == "/dragon/uav/cog/odom":
                decimated_append(data[topic], pose_row(t, msg.pose.pose.position), max_position_points)
            elif topic == "/gazebo/link_states":
                if link4_index is None:
                    try:
                        link4_index = list(msg.name).index(link4_name)
                    except ValueError:
                        continue
                if link4_index < len(msg.pose):
                    decimated_append(data["link4"], pose_row(t, msg.pose[link4_index].position),
                                     max_position_points)

    return data, stats


def values(series, key):
    xs, ys = [], []
    for t, row in series:
        if key in row:
            xs.append(t)
            ys.append(row[key])
    return xs, ys


def position_values(series, key):
    return [row["t"] for row in series], [row[key] for row in series]


def last_before(series, t):
    if not series:
        return None
    times = [row[0] for row in series]
    index = bisect.bisect_right(times, t) - 1
    if index < 0:
        return None
    return series[index]


def aligned_joint_pairs(source_series, target_series, source_key, target_key):
    xs, ys = [], []
    for t, target in target_series:
        if target_key not in target:
            continue
        source = last_before(source_series, t)
        if source is None or source_key not in source[1]:
            continue
        xs.append(source[1][source_key])
        ys.append(target[target_key])
    return xs, ys


def plot_joint_traces(data, out_dir, formats):
    paths = []
    for joint in DRAGON_JOINTS:
        fig, ax = plt.subplots(figsize=(12.0, 5.2))
        for topic, label, style, width, color in (
                ("/dragon/joint_states", "DRAGON actual", "-", 1.8, "tab:blue"),
                ("/dragon/joints_ctrl", "DRAGON command", "-", 1.4, "tab:orange"),
                ("/dracomancer/candidate/joint_target", "candidate", "--", 1.2, "tab:green")):
            xs, ys = values(data[topic], joint)
            if xs:
                ax.plot(xs, ys, style, linewidth=width, color=color, label=label)

        for source in SOURCE_RELATIONS[joint]:
            xs, ys = values(data["/dracomancer/joint_states"], source)
            if xs:
                ax.plot(xs, ys, ":", linewidth=1.0, alpha=0.85,
                        label="Dracomancer " + source)

        ax.set_title(joint + " mapping trace")
        ax.set_xlabel("Time from bag start [s]")
        ax.set_ylabel("Joint angle [rad]")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", ncol=2)
        paths.extend(save(fig, out_dir, joint, formats))
    return paths


def plot_joint_scatter(data, out_dir, formats):
    fig, axes = plt.subplots(2, 3, figsize=(13.0, 7.2), squeeze=False)
    for i, joint in enumerate(DRAGON_JOINTS):
        ax = axes[i // 3][i % 3]
        source = PRIMARY_SOURCE[joint]
        for topic, label, color, marker_size in (
                ("/dragon/joints_ctrl", "DRAGON command", "tab:orange", 12),
                ("/dragon/joint_states", "DRAGON actual", "tab:blue", 7),
                ("/dracomancer/candidate/joint_target", "candidate", "tab:green", 10)):
            xs, ys = aligned_joint_pairs(
                data["/dracomancer/joint_states"], data[topic], source, joint)
            if xs:
                ax.scatter(xs, ys, s=marker_size, alpha=0.45, color=color, label=label)
        ax.set_title(joint)
        ax.set_xlabel(source + " [rad]")
        ax.set_ylabel("DRAGON angle [rad]")
        ax.grid(True, alpha=0.25)
    axes[0][0].legend(loc="best")
    return save(fig, out_dir, "joint_relation_scatter", formats)


def plot_all_joint_traces(data, out_dir, formats):
    fig, axes = plt.subplots(3, 2, figsize=(12.0, 10.0), sharex=True, squeeze=False)
    for i, joint in enumerate(DRAGON_JOINTS):
        ax = axes[i // 2][i % 2]
        for topic, label, style, width, color in (
                ("/dragon/joint_states", "actual", "-", 1.4, "tab:blue"),
                ("/dragon/joints_ctrl", "command", "-", 1.1, "tab:orange"),
                ("/dracomancer/candidate/joint_target", "candidate", "--", 1.0, "tab:green")):
            xs, ys = values(data[topic], joint)
            if xs:
                ax.plot(xs, ys, style, linewidth=width, color=color, label=label)
        ax.set_title(joint)
        ax.set_ylabel("[rad]")
        ax.grid(True, alpha=0.25)
    for ax in axes[-1]:
        ax.set_xlabel("Time from bag start [s]")
    axes[0][0].legend(loc="best")
    return save(fig, out_dir, "dragon_joint_traces", formats)


def plot_positions(data, out_dir, formats):
    paths = []
    fig, axes = plt.subplots(4, 1, figsize=(12.0, 8.8), sharex=True)
    for ax, key, ylabel in zip(axes[:3], ["x", "y", "z"], ["x [m]", "y [m]", "z [m]"]):
        for series_name, label, color in (
                ("/dragon/uav/cog/odom", "COG", "tab:blue"),
                ("link4", "Link4", "tab:orange")):
            xs, ys = position_values(data[series_name], key)
            if xs:
                ax.plot(xs, ys, linewidth=1.4, color=color, label=label)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25)
    axes[0].set_title("COG and Link4 position")
    axes[0].legend(loc="best")

    cog_by_time = {round(row["t"], 2): row for row in data["/dragon/uav/cog/odom"]}
    dist_t, dist = [], []
    for row in data["link4"]:
        cog = cog_by_time.get(round(row["t"], 2))
        if cog is None:
            continue
        d = math.sqrt((row["x"] - cog["x"]) ** 2 +
                      (row["y"] - cog["y"]) ** 2 +
                      (row["z"] - cog["z"]) ** 2)
        dist_t.append(row["t"])
        dist.append(d)
    if dist_t:
        axes[3].plot(dist_t, dist, linewidth=1.3, color="tab:green")
    axes[3].set_ylabel("|Link4-COG| [m]")
    axes[3].set_xlabel("Time from bag start [s]")
    axes[3].grid(True, alpha=0.25)
    paths.extend(save(fig, out_dir, "cog_link4_position", formats))

    fig, ax = plt.subplots(figsize=(7.0, 6.2))
    for series_name, label, color in (
            ("/dragon/uav/cog/odom", "COG", "tab:blue"),
            ("link4", "Link4", "tab:orange")):
        xs = [row["x"] for row in data[series_name]]
        ys = [row["y"] for row in data[series_name]]
        if xs:
            ax.plot(xs, ys, linewidth=1.4, color=color, label=label)
            ax.scatter(xs[0], ys[0], s=45, color=color, marker="o")
            ax.scatter(xs[-1], ys[-1], s=45, color=color, marker="x")
    ax.set_title("COG and Link4 XY path")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    paths.extend(save(fig, out_dir, "cog_link4_xy", formats))
    return paths


def minmax(rows, key):
    vals = [row[key] for row in rows]
    if not vals:
        return None
    return min(vals), max(vals)


def write_summary(out_dir, bag_path, data, stats, paths):
    path = os.path.join(out_dir, "summary.txt")
    with open(path, "w") as fp:
        fp.write("bag: %s\n" % bag_path)
        fp.write("duration: %.3f s\n" % (stats["end_time"] - stats["start_time"]))
        fp.write("link4_source: /gazebo/link_states[%s]\n" % stats["link4_name"])
        fp.write("raw_topic_counts:\n")
        for topic in TOPICS:
            fp.write("  %s: %d\n" % (topic, stats["raw_counts"][topic]))
        fp.write("kept_position_samples:\n")
        fp.write("  /dragon/uav/cog/odom: %d\n" % len(data["/dragon/uav/cog/odom"]))
        fp.write("  link4: %d\n" % len(data["link4"]))
        for name, series in (("cog", data["/dragon/uav/cog/odom"]), ("link4", data["link4"])):
            fp.write("%s_ranges:\n" % name)
            for key in ("x", "y", "z"):
                mm = minmax(series, key)
                if mm:
                    fp.write("  %s: %.6f %.6f\n" % (key, mm[0], mm[1]))
        fp.write("outputs:\n")
        for output in paths:
            fp.write("  %s\n" % os.path.basename(output))
    return path


def main():
    args = parse_args()
    setup_style()
    ensure_dir(args.out_dir)
    data, stats = collect_bag(args.bag, args.link4_name, args.max_position_points)
    outputs = []
    outputs.extend(plot_joint_traces(data, args.out_dir, args.formats))
    outputs.extend(plot_joint_scatter(data, args.out_dir, args.formats))
    outputs.extend(plot_all_joint_traces(data, args.out_dir, args.formats))
    outputs.extend(plot_positions(data, args.out_dir, args.formats))
    outputs.append(write_summary(args.out_dir, args.bag, data, stats, outputs))
    for path in outputs:
        print(path)


if __name__ == "__main__":
    main()
