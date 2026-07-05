#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Plot Dracomancer-to-DRAGON joint traces from the falling_test rosbag.

The script writes one figure per DRAGON joint.  Each figure overlays:
  - DRAGON actual joint angle
  - DRAGON commanded joint angle after the feasibility gate
  - DRAGON candidate joint angle before the feasibility gate
  - related Dracomancer source joints

Use --show when running on a machine with a display to step through figures.
"""

import argparse
import bisect
import math
import os

import matplotlib


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "bag",
        nargs="?",
        default=os.path.expanduser("~/rosbag/falling_test_20260705_222212.bag"),
        help="input rosbag path",
    )
    parser.add_argument(
        "-o",
        "--out-dir",
        default=os.path.join(os.path.dirname(__file__), "falling_test"),
        help="output directory",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="show each matplotlib figure in sequence",
    )
    return parser.parse_args()


def setup_matplotlib(show):
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    return plt, PdfPages


def as_joint_dict(msg):
    return {name: float(pos) for name, pos in zip(msg.name, msg.position)}


def collect_bag(bag_path):
    import rosbag
    from tf.transformations import euler_from_quaternion

    topics = [
        "/dracomancer/joint_states",
        "/dracomancer/joint_map/switch_ratio",
        "/dracomancer/candidate/joint_target",
        "/dracomancer/candidate/fc_f_min",
        "/dracomancer/candidate/fc_t_min",
        "/dracomancer/target/fc_f_min",
        "/dracomancer/target/fc_t_min",
        "/dracomancer/dragon_shape_safety_scale",
        "/dracomancer/force_volume_radius_threshold",
        "/dracomancer/torque_volume_radius_threshold",
        "/dragon/joints_ctrl",
        "/dragon/joint_states",
        "/dragon/debug/fc_f_min_filtered",
        "/dragon/debug/fc_t_min_filtered",
        "/dragon/flight_state",
        "/dragon/ground_truth",
        "/dragon/uav/baselink/odom",
        "/dragon/uav/nav",
        "/rosout",
    ]
    data = {topic: [] for topic in topics}
    events = {"failsafe": [], "force_landing": []}

    with rosbag.Bag(bag_path) as bag:
        start = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(topics=topics):
            t = stamp.to_sec() - start
            if topic in (
                "/dracomancer/joint_states",
                "/dracomancer/candidate/joint_target",
                "/dragon/joints_ctrl",
                "/dragon/joint_states",
            ):
                data[topic].append((t, as_joint_dict(msg)))
            elif topic == "/dracomancer/joint_map/switch_ratio":
                names = [
                    "wrist_roll",
                    "wrist_rho",
                    "wrist_pitch_weight",
                    "wrist_yaw_weight",
                    "elbow_roll",
                    "elbow_rho",
                    "elbow_pitch_weight",
                    "elbow_yaw_weight",
                ]
                data[topic].append((t, dict(zip(names, [float(v) for v in msg.data]))))
            elif topic in (
                "/dracomancer/candidate/fc_f_min",
                "/dracomancer/candidate/fc_t_min",
                "/dracomancer/target/fc_f_min",
                "/dracomancer/target/fc_t_min",
                "/dracomancer/dragon_shape_safety_scale",
                "/dragon/debug/fc_f_min_filtered",
                "/dragon/debug/fc_t_min_filtered",
                "/dragon/flight_state",
            ):
                data[topic].append((t, float(msg.data)))
            elif topic in (
                "/dracomancer/force_volume_radius_threshold",
                "/dracomancer/torque_volume_radius_threshold",
            ):
                data[topic].append((t, tuple(float(v) for v in msg.data)))
            elif topic == "/dragon/ground_truth":
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                data[topic].append((t, {"x": p.x, "y": p.y, "z": p.z,
                                        "roll": roll, "pitch": pitch, "yaw": yaw}))
            elif topic == "/dragon/uav/baselink/odom":
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                data[topic].append((t, {"x": p.x, "y": p.y, "z": p.z,
                                        "roll": roll, "pitch": pitch, "yaw": yaw}))
            elif topic == "/dragon/uav/nav":
                data[topic].append((t, {
                    "x": float(msg.target_pos_x),
                    "y": float(msg.target_pos_y),
                    "z": float(msg.target_pos_z),
                    "xy_mode": int(msg.pos_xy_nav_mode),
                    "z_mode": int(msg.pos_z_nav_mode),
                }))
            elif topic == "/rosout":
                lower = msg.msg.lower()
                if "failsafe" in lower:
                    events["failsafe"].append((t, msg.msg))
                if "force landing" in lower:
                    events["force_landing"].append((t, msg.msg))

    data["_events"] = events
    return start, data


def values(series, key):
    xs, ys = [], []
    for t, item in series:
        if isinstance(item, dict):
            if key in item:
                xs.append(t)
                ys.append(item[key])
        else:
            xs.append(t)
            ys.append(item)
    return xs, ys


def aligned_delta(reference_series, measured_series):
    """Return measured-reference values at reference timestamps.

    measured_series is sampled with last-value-hold because candidate fc and
    controller fc are published by different nodes at different rates.
    """
    rows = []
    for t, ref_value in reference_series:
        measured = last_before(measured_series, t)
        if measured is None:
            continue
        rows.append((t, measured[1], ref_value, measured[1] - ref_value))
    return rows


def last_before(series, t):
    if not series:
        return None
    times = [row[0] for row in series]
    index = bisect.bisect_right(times, t) - 1
    if index < 0:
        return None
    return series[index]


def first_fall_time(ground_truth):
    was_high = False
    for t, item in ground_truth:
        if item["z"] > 0.8:
            was_high = True
        if was_high and item["z"] < 0.3:
            return t
    return None


def flight_state_changes(series):
    changes = []
    last = None
    for t, state in series:
        if state != last:
            changes.append((t, last, state))
            last = state
    return changes


def scalar_stats(series, threshold=None, end_time=None):
    rows = [row for row in series if end_time is None or row[0] <= end_time]
    vals = [row[1] for row in rows]
    if not vals:
        return None
    below = sum(1 for v in vals if threshold is not None and v < threshold)
    first_below = next((row for row in rows if threshold is not None and row[1] < threshold), None)
    return {
        "min": min(vals),
        "max": max(vals),
        "below": below,
        "count": len(vals),
        "first_below": first_below,
    }


def plot_joint_figures(plt, PdfPages, data, out_dir, show):
    os.makedirs(out_dir, exist_ok=True)
    relations = {
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
    names = list(relations.keys())
    fail_time = data["_events"]["failsafe"][0][0] if data["_events"]["failsafe"] else None
    state17 = next((t for t, _old, new in flight_state_changes(data["/dragon/flight_state"])
                    if new == 17), None)
    fall_time = first_fall_time(data["/dragon/ground_truth"])
    pdf_path = os.path.join(out_dir, "joint_mapping.pdf")

    with PdfPages(pdf_path) as pdf:
        for joint in names:
            fig, ax = plt.subplots(figsize=(12, 6))
            for topic, label, style, width in [
                ("/dragon/joint_states", "DRAGON actual", "-", 1.8),
                ("/dragon/joints_ctrl", "DRAGON command", "-", 1.5),
                ("/dracomancer/candidate/joint_target", "candidate before gate", "--", 1.2),
            ]:
                xs, ys = values(data[topic], joint)
                if xs:
                    ax.plot(xs, ys, style, linewidth=width, label=label)

            for source in relations[joint]:
                xs, ys = values(data["/dracomancer/joint_states"], source)
                if xs:
                    ax.plot(xs, ys, ":", linewidth=1.1, label="Dracomancer " + source)

            for x, label, color in [
                (fail_time, "failsafe", "tab:red"),
                (state17, "flight_state 17", "tab:orange"),
                (fall_time, "z < 0.3 m", "tab:brown"),
            ]:
                if x is not None:
                    ax.axvline(x, color=color, linewidth=1.0, alpha=0.8)
                    ax.text(x, 0.98, label, color=color, rotation=90,
                            va="top", ha="right", transform=ax.get_xaxis_transform())

            ax.set_title(joint + " mapping trace")
            ax.set_xlabel("time from bag start [s]")
            ax.set_ylabel("angle [rad]")
            ax.grid(True, alpha=0.25)
            ax.legend(loc="upper right", fontsize=8)
            fig.tight_layout()
            fig.savefig(os.path.join(out_dir, joint + ".png"), dpi=180)
            fig.savefig(os.path.join(out_dir, joint + ".svg"))
            pdf.savefig(fig)
            if show:
                plt.show()
            plt.close(fig)

    return pdf_path


def plot_safety_overview(plt, data, out_dir):
    fail_time = data["_events"]["failsafe"][0][0] if data["_events"]["failsafe"] else None
    fall_time = first_fall_time(data["/dragon/ground_truth"])
    state17 = next((t for t, _old, new in flight_state_changes(data["/dragon/flight_state"])
                    if new == 17), None)
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

    for topic, key, label in [
        ("/dragon/ground_truth", "z", "z"),
        ("/dragon/ground_truth", "roll", "roll"),
        ("/dragon/ground_truth", "pitch", "pitch"),
    ]:
        xs, ys = values(data[topic], key)
        axes[0].plot(xs, ys, label=label)
    axes[0].set_ylabel("pose")
    axes[0].legend(loc="upper right", fontsize=8)

    for topic, label in [
        ("/dragon/debug/fc_f_min_filtered", "actual force fc"),
        ("/dragon/debug/fc_t_min_filtered", "actual torque fc"),
        ("/dracomancer/candidate/fc_f_min", "candidate force fc"),
        ("/dracomancer/candidate/fc_t_min", "candidate torque fc"),
        ("/dracomancer/target/fc_f_min", "target force fc"),
        ("/dracomancer/target/fc_t_min", "target torque fc"),
    ]:
        xs, ys = values(data[topic], None)
        axes[1].plot(xs, ys, label=label)
    f_thr = data["/dracomancer/force_volume_radius_threshold"][0][1][0]
    t_thr = data["/dracomancer/torque_volume_radius_threshold"][0][1][0]
    axes[1].axhline(f_thr, color="tab:blue", linestyle=":", linewidth=1.0, label="force hard")
    axes[1].axhline(t_thr, color="tab:orange", linestyle=":", linewidth=1.0, label="torque hard")
    axes[1].set_ylabel("fc radius")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[1].set_yscale("symlog", linthresh=0.02)

    xs, ys = values(data["/dracomancer/dragon_shape_safety_scale"], None)
    axes[2].plot(xs, ys, label="live safety scale")
    axes[2].set_ylabel("scale")
    axes[2].set_ylim(-0.05, 1.05)
    axes[2].legend(loc="upper right", fontsize=8)

    for key, label in [("x", "nav x"), ("y", "nav y"), ("z", "nav z")]:
        xs, ys = values(data["/dragon/uav/nav"], key)
        axes[3].plot(xs, ys, label=label)
    axes[3].set_ylabel("COG target [m]")
    axes[3].set_xlabel("time from bag start [s]")
    axes[3].legend(loc="upper right", fontsize=8)

    for ax in axes:
        ax.grid(True, alpha=0.25)
        for x, label, color in [
            (fail_time, "failsafe", "tab:red"),
            (state17, "flight_state 17", "tab:orange"),
            (fall_time, "z < 0.3 m", "tab:brown"),
        ]:
            if x is not None:
                ax.axvline(x, color=color, linewidth=1.0, alpha=0.8)

    fig.tight_layout()
    path = os.path.join(out_dir, "safety_overview.png")
    svg_path = os.path.join(out_dir, "safety_overview.svg")
    fig.savefig(path, dpi=180)
    fig.savefig(svg_path)
    plt.close(fig)
    return path, svg_path


def plot_fc_prediction_error(plt, data, out_dir):
    fail_time = data["_events"]["failsafe"][0][0] if data["_events"]["failsafe"] else None
    fall_time = first_fall_time(data["/dragon/ground_truth"])
    state17 = next((t for t, _old, new in flight_state_changes(data["/dragon/flight_state"])
                    if new == 17), None)
    force_rows = aligned_delta(
        data["/dracomancer/candidate/fc_f_min"],
        data["/dragon/debug/fc_f_min_filtered"])
    torque_rows = aligned_delta(
        data["/dracomancer/candidate/fc_t_min"],
        data["/dragon/debug/fc_t_min_filtered"])
    target_force_rows = aligned_delta(
        data["/dracomancer/target/fc_f_min"],
        data["/dragon/debug/fc_f_min_filtered"])
    target_torque_rows = aligned_delta(
        data["/dracomancer/target/fc_t_min"],
        data["/dragon/debug/fc_t_min_filtered"])

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for rows, axis, title, threshold_topic in [
        (force_rows, axes[0], "force fc: measured vs predicted",
         "/dracomancer/force_volume_radius_threshold"),
        (torque_rows, axes[1], "torque fc: measured vs predicted",
         "/dracomancer/torque_volume_radius_threshold"),
    ]:
        ts = [row[0] for row in rows]
        measured = [row[1] for row in rows]
        predicted = [row[2] for row in rows]
        axis.plot(ts, predicted, label="predicted candidate", linewidth=1.4)
        target_rows = target_force_rows if threshold_topic.endswith("force_volume_radius_threshold") else target_torque_rows
        if target_rows:
            axis.plot([row[0] for row in target_rows], [row[2] for row in target_rows],
                      label="predicted final target", linewidth=1.2, linestyle="--")
        axis.plot(ts, measured, label="measured filtered", linewidth=1.2)
        if data[threshold_topic]:
            hard, recover = data[threshold_topic][0][1][0:2]
            axis.axhline(hard, color="tab:red", linestyle=":", linewidth=1.0, label="hard")
            axis.axhline(recover, color="tab:green", linestyle=":", linewidth=1.0, label="recover")
        axis.set_title(title)
        axis.set_ylabel("fc radius")
        axis.set_yscale("symlog", linthresh=0.02)
        axis.legend(loc="upper right", fontsize=8)

    for rows, label in [
        (force_rows, "force actual - predicted"),
        (torque_rows, "torque actual - predicted"),
        (target_force_rows, "force actual - final-target predicted"),
        (target_torque_rows, "torque actual - final-target predicted"),
    ]:
        if rows:
            axes[2].plot([row[0] for row in rows], [row[3] for row in rows], label=label)
    axes[2].axhline(0.0, color="black", linewidth=0.8)
    axes[2].set_title("prediction error at candidate timestamps")
    axes[2].set_xlabel("time from bag start [s]")
    axes[2].set_ylabel("actual - predicted")
    axes[2].legend(loc="upper right", fontsize=8)

    for ax in axes:
        ax.grid(True, alpha=0.25)
        for x, color in [
            (fail_time, "tab:red"),
            (state17, "tab:orange"),
            (fall_time, "tab:brown"),
        ]:
            if x is not None:
                ax.axvline(x, color=color, linewidth=1.0, alpha=0.8)

    fig.tight_layout()
    png_path = os.path.join(out_dir, "fc_prediction_error.png")
    svg_path = os.path.join(out_dir, "fc_prediction_error.svg")
    fig.savefig(png_path, dpi=180)
    fig.savefig(svg_path)
    plt.close(fig)
    return png_path, svg_path


def write_summary(start_abs, data, out_dir, pdf_path, safety_paths, fc_error_paths):
    os.makedirs(out_dir, exist_ok=True)
    events = data["_events"]
    fail_time = events["failsafe"][0][0] if events["failsafe"] else None
    force_land_time = events["force_landing"][0][0] if events["force_landing"] else None
    fall_time = first_fall_time(data["/dragon/ground_truth"])
    state_changes = flight_state_changes(data["/dragon/flight_state"])
    state17 = next((t for t, _old, new in state_changes if new == 17), None)

    f_hard = data["/dracomancer/force_volume_radius_threshold"][0][1][0]
    t_hard = data["/dracomancer/torque_volume_radius_threshold"][0][1][0]
    stats = {
        "candidate_force": scalar_stats(data["/dracomancer/candidate/fc_f_min"], f_hard, fail_time),
        "candidate_torque": scalar_stats(data["/dracomancer/candidate/fc_t_min"], t_hard, fail_time),
        "actual_force": scalar_stats(data["/dragon/debug/fc_f_min_filtered"], f_hard, fail_time),
        "actual_torque": scalar_stats(data["/dragon/debug/fc_t_min_filtered"], t_hard, fail_time),
    }
    before_fail_pose = last_before(data["/dragon/ground_truth"], fail_time or 0.0)
    before_fail_cmd = last_before(data["/dragon/joints_ctrl"], fail_time or 0.0)
    before_fail_cand = last_before(data["/dracomancer/candidate/joint_target"], fail_time or 0.0)
    before_fail_scale = last_before(data["/dracomancer/dragon_shape_safety_scale"], fail_time or 0.0)

    path = os.path.join(out_dir, "summary.txt")
    with open(path, "w") as f:
        f.write("bag_start_abs: %.3f\n" % start_abs)
        f.write("failsafe_rt: %s\n" % ("n/a" if fail_time is None else "%.3f" % fail_time))
        f.write("force_landing_rt: %s\n" % ("n/a" if force_land_time is None else "%.3f" % force_land_time))
        f.write("flight_state_17_rt: %s\n" % ("n/a" if state17 is None else "%.3f" % state17))
        f.write("z_below_0.3_after_hover_rt: %s\n" % ("n/a" if fall_time is None else "%.3f" % fall_time))
        f.write("thresholds: force_hard=%.5f torque_hard=%.5f\n" % (f_hard, t_hard))
        for key, value in stats.items():
            f.write("%s: %s\n" % (key, value))
        f.write("pose_before_failsafe: %s\n" % (before_fail_pose,))
        f.write("joints_ctrl_before_failsafe: %s\n" % (before_fail_cmd,))
        f.write("candidate_before_failsafe: %s\n" % (before_fail_cand,))
        f.write("safety_scale_before_failsafe: %s\n" % (before_fail_scale,))
        if events["failsafe"]:
            f.write("failsafe_message: %s\n" % events["failsafe"][0][1])
        f.write("joint_mapping_pdf: %s\n" % pdf_path)
        f.write("safety_overview_png: %s\n" % safety_paths[0])
        f.write("safety_overview_svg: %s\n" % safety_paths[1])
        f.write("fc_prediction_error_png: %s\n" % fc_error_paths[0])
        f.write("fc_prediction_error_svg: %s\n" % fc_error_paths[1])
    return path


def main():
    args = parse_args()
    plt, PdfPages = setup_matplotlib(args.show)
    start_abs, data = collect_bag(os.path.expanduser(args.bag))
    pdf_path = plot_joint_figures(plt, PdfPages, data, args.out_dir, args.show)
    safety_paths = plot_safety_overview(plt, data, args.out_dir)
    fc_error_paths = plot_fc_prediction_error(plt, data, args.out_dir)
    summary_path = write_summary(start_abs, data, args.out_dir, pdf_path,
                                 safety_paths, fc_error_paths)
    print("wrote", args.out_dir)
    print("summary", summary_path)


if __name__ == "__main__":
    main()
