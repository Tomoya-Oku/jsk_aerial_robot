#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Plot device-to-robot joint traces from the falling_test rosbag.

The script writes one figure per robot joint.  Each figure overlays:
  - robot actual joint angle
  - robot commanded joint angle after the feasibility gate
  - related device source joints

Use --show when running on a machine with a display to step through figures.
"""

import argparse
import bisect
import math
import os

import matplotlib
from matplotlib.ticker import LinearLocator, MultipleLocator


ROLL_COLOR = "#FF0000"
PITCH_COLOR = "#00FF00"
YAW_COLOR = "#0000FF"
ROLL_COMMAND_COLOR = "#B00000"
PITCH_COMMAND_COLOR = "#00A000"
YAW_COMMAND_COLOR = "#4040FF"
RATIO_COLOR = "#808080"
# Okabe-Ito style colors for publication figures with several overlaid lines.
FORCE_PREDICTED_COLOR = "#D55E00"
FORCE_MEASURED_COLOR = "#E69F00"
TORQUE_PREDICTED_COLOR = "#0072B2"
TORQUE_MEASURED_COLOR = "#56B4E9"
THRESHOLD_HARD_COLOR = "#000000"
FORCE_THRESHOLD_MIN_COLOR = "#CC79A7"
TORQUE_THRESHOLD_MIN_COLOR = "#009E73"
INRADIUS_LOWPASS_CUTOFF_HZ = 0.08

DEVICE_JOINT_LABELS = {
    "wrist_flexion_extension_joint": r"$q_{\mathrm{wf}}$",
    "wrist_abduction_adduction_joint": r"$q_{\mathrm{wa}}$",
    "elbow_flexion_extension_joint": r"$q_{\mathrm{ef}}$",
    "shoulder_flexion_extension_joint": r"$q_{\mathrm{sf}}$",
    "shoulder_abduction_adduction_joint": r"$q_{\mathrm{sa}}$",
    "upper_arm_external_internal_rotation_joint": r"$q_{\mathrm{sr}}$",
    "wrist_supination_joint": r"$q_{\mathrm{fp}}$",
}


def kinematic_color(name):
    lower = name.lower()
    if "roll" in lower or "supination" in lower or "pronation" in lower:
        return ROLL_COLOR
    if "pitch" in lower or "flexion" in lower or "extension" in lower:
        return PITCH_COLOR
    if "yaw" in lower or "abduction" in lower or "adduction" in lower:
        return YAW_COLOR
    return None


def command_color(name):
    lower = name.lower()
    if "roll" in lower or "supination" in lower or "pronation" in lower:
        return ROLL_COMMAND_COLOR
    if "pitch" in lower or "flexion" in lower or "extension" in lower:
        return PITCH_COMMAND_COLOR
    if "yaw" in lower or "abduction" in lower or "adduction" in lower:
        return YAW_COMMAND_COLOR
    return kinematic_color(name)


def robot_joint_label(name):
    parts = name.split("_", 1)
    if len(parts) != 2 or not parts[0].startswith("joint"):
        return name
    index = parts[0].replace("joint", "")
    return "Joint-%s %s" % (index, parts[1])


def device_joint_label(name):
    return DEVICE_JOINT_LABELS.get(name, name)


def robot_joint_axis_label(name):
    parts = name.split("_", 1)
    if len(parts) == 2 and parts[0].startswith("joint"):
        return "%s [deg]" % parts[1]
    return "%s [deg]" % name


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
    parser.add_argument(
        "--paper-style",
        action="store_true",
        help="use Times New Roman and larger labels for paper figures",
    )
    return parser.parse_args()


def setup_matplotlib(show, paper_style=False):
    if not show:
        matplotlib.use("Agg")
    if paper_style:
        matplotlib.rcParams.update({
            "font.family": "Times New Roman",
            "font.size": 24,
            "axes.titlesize": 28,
            "axes.labelsize": 28,
            "xtick.labelsize": 24,
            "ytick.labelsize": 24,
            "legend.fontsize": 21,
            "lines.linewidth": 2.2,
            "axes.linewidth": 1.2,
            "xtick.direction": "in",
            "ytick.direction": "in",
            "xtick.top": True,
            "ytick.right": True,
            "mathtext.fontset": "cm",
            "text.latex.preamble": r"\usepackage{amsmath}\usepackage{bm}",
            "svg.fonttype": "none",
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        })
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
        "/dragon/uav/cog/odom",
        "/dragon/uav/nav",
        "/gazebo/link_states",
        "/rosout",
    ]
    data = {topic: [] for topic in topics}
    events = {"failsafe": [], "force_landing": []}
    link4_index = None
    last_link4_sample = None
    last_cog_sample = None

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
            elif topic == "/dragon/uav/cog/odom":
                if last_cog_sample is not None and t - last_cog_sample < 0.02:
                    continue
                last_cog_sample = t
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
            elif topic == "/gazebo/link_states":
                if link4_index is None:
                    try:
                        link4_index = list(msg.name).index("dragon::link4")
                    except ValueError:
                        continue
                if last_link4_sample is not None and t - last_link4_sample < 0.02:
                    continue
                last_link4_sample = t
                p = msg.pose[link4_index].position
                q = msg.pose[link4_index].orientation
                roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                data[topic].append((t, {"x": p.x, "y": p.y, "z": p.z,
                                        "roll": roll, "pitch": pitch, "yaw": yaw}))
            elif topic == "/rosout":
                lower = msg.msg.lower()
                if "failsafe" in lower:
                    events["failsafe"].append((t, msg.msg))
                if "force landing" in lower:
                    events["force_landing"].append((t, msg.msg))

    data["_events"] = events
    return start, data


def teleoperation_origin(data):
    """Use the first teleoperation-side command/candidate as the plot origin."""
    candidates = []
    for topic in (
            "/dracomancer/candidate/joint_target",
            "/dracomancer/candidate/fc_f_min",
            "/dracomancer/target/fc_f_min",
            "/dragon/joints_ctrl",
            "/dragon/uav/nav"):
        if data.get(topic):
            candidates.append(data[topic][0][0])
    return min(candidates) if candidates else 0.0


def teleoperation_end(data):
    """Use the last teleoperation-side command/candidate as the plot end."""
    candidates = []
    for topic in (
            "/dracomancer/candidate/joint_target",
            "/dracomancer/candidate/fc_f_min",
            "/dracomancer/target/fc_f_min",
            "/dragon/joints_ctrl",
            "/dragon/uav/nav"):
        if data.get(topic):
            candidates.append(data[topic][-1][0])
    return max(candidates) if candidates else None


def rebase_time(data, origin, end=None):
    """Shift plot data to teleoperation start and keep only the teleop window."""
    def in_window(t):
        return t >= origin and (end is None or t <= end)

    rebased = {}
    for topic, series in data.items():
        if topic == "_events":
            rebased[topic] = {
                name: [(t - origin, msg) for t, msg in rows if in_window(t)]
                for name, rows in series.items()
            }
        elif topic.startswith("_"):
            rebased[topic] = series
        else:
            rebased[topic] = [(t - origin, item) for t, item in series if in_window(t)]
    rebased["_time_origin_rt"] = origin
    rebased["_time_end_rt"] = end
    rebased["_time_end"] = None if end is None else end - origin
    return rebased


def style_axis(ax, time_axis=False, data=None):
    ax.tick_params(direction="in", top=True, right=True)
    if time_axis:
        right = data.get("_time_end") if data else None
        if right is None:
            ax.set_xlim(left=0.0)
        else:
            ax.set_xlim(left=0.0, right=right)


def align_twin_y_ticks(left_ax, right_ax, nticks=5):
    left_ax.yaxis.set_major_locator(LinearLocator(nticks))
    right_ax.yaxis.set_major_locator(LinearLocator(nticks))


def set_q_rho_ticks(q_ax, rho_ax, center_deg, step_deg=30.0):
    ymin, ymax = q_ax.get_ylim()
    half_range = max(abs(ymin - center_deg), abs(ymax - center_deg), step_deg)
    half_range = step_deg * math.ceil(half_range / step_deg)
    lower = center_deg - half_range
    upper = center_deg + half_range
    count = int(round((upper - lower) / step_deg))
    q_ticks = [lower + step_deg * i for i in range(count + 1)]
    q_ax.set_ylim(lower, upper)
    q_ax.set_yticks(q_ticks)

    rho_ax.set_ylim(0.0, 1.0)
    rho_ax.set_yticks([0.0, 0.5, 1.0])


def set_q_rho_ticks_from_data(q_ax, rho_ax, q_values, step_deg=30.0, margin_frac=0.08):
    if not q_values:
        return
    ymin = min(q_values)
    ymax = max(q_values)
    margin = max(step_deg * 0.5, (ymax - ymin) * margin_frac)
    lower = step_deg * math.floor((ymin - margin) / step_deg)
    upper = step_deg * math.ceil((ymax + margin) / step_deg)
    if upper <= lower:
        upper = lower + step_deg
    count = int(round((upper - lower) / step_deg))
    q_ticks = [lower + step_deg * i for i in range(count + 1)]
    q_ax.set_ylim(lower, upper)
    q_ax.set_yticks(q_ticks)

    rho_ax.set_ylim(0.0, 1.0)
    rho_ax.set_yticks([0.0, 0.5, 1.0])


def save_tight(fig, png_path, svg_path, dpi=220, h_pad=None):
    if h_pad is None:
        fig.tight_layout()
    else:
        fig.tight_layout(h_pad=h_pad)
    fig.savefig(png_path, dpi=dpi, bbox_inches="tight", pad_inches=0.02)
    fig.savefig(svg_path, bbox_inches="tight", pad_inches=0.02)


def relative_pose_series(series):
    if not series:
        return []
    base = series[0][1]
    if not isinstance(base, dict):
        return series
    return [
        (t, dict(item, x=item.get("x", 0.0) - base.get("x", 0.0),
                 y=item.get("y", 0.0) - base.get("y", 0.0),
                 z=item.get("z", 0.0) - base.get("z", 0.0)))
        for t, item in series
    ]


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


def step_aligned_error(reference_series, target_series, key):
    ref_xs, ref_ys = values(reference_series, key)
    target_xs, target_ys = values(target_series, key)
    if not ref_xs or not target_xs:
        return [], []

    aligned_xs = []
    errors = []
    target_index = 0
    for t, actual in zip(ref_xs, ref_ys):
        while (target_index + 1 < len(target_xs) and
               target_xs[target_index + 1] <= t):
            target_index += 1
        if target_xs[target_index] <= t:
            aligned_xs.append(t)
            errors.append(actual - target_ys[target_index])
    return aligned_xs, errors


def rolling_rmse(xs, errors, window_sec):
    if not xs or not errors:
        return []
    rmses = []
    start_index = 0
    squared_sum = 0.0
    for index, (t, error) in enumerate(zip(xs, errors)):
        squared_sum += error * error
        while xs[start_index] < t - window_sec:
            old = errors[start_index]
            squared_sum -= old * old
            start_index += 1
        count = index - start_index + 1
        rmses.append(math.sqrt(max(0.0, squared_sum) / float(count)))
    return rmses


def lowpass_values(xs, ys, cutoff_hz=1.0):
    if not xs or not ys:
        return ys
    filtered = [ys[0]]
    tau = 1.0 / (2.0 * math.pi * cutoff_hz)
    for index in range(1, len(ys)):
        dt = max(0.0, xs[index] - xs[index - 1])
        alpha = dt / (tau + dt) if dt > 0.0 else 0.0
        filtered.append(filtered[-1] + alpha * (ys[index] - filtered[-1]))
    return filtered


def percentile(values, ratio):
    rows = sorted(v for v in values if math.isfinite(v))
    if not rows:
        return None
    index = int(round((len(rows) - 1) * ratio))
    return rows[max(0, min(len(rows) - 1, index))]


def nice_upper_limit(value):
    if value <= 0.0:
        return 1.0
    step = 0.1 if value <= 1.0 else 0.5 if value <= 5.0 else 1.0
    return step * math.ceil(value / step)


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
    primary_sources = {
        "joint1_pitch": {
            "wrist_flexion_extension_joint",
            "wrist_abduction_adduction_joint",
        },
        "joint1_yaw": {
            "wrist_flexion_extension_joint",
            "wrist_abduction_adduction_joint",
        },
        "joint2_pitch": {"elbow_flexion_extension_joint"},
        "joint2_yaw": {"elbow_flexion_extension_joint"},
        "joint3_pitch": {"shoulder_flexion_extension_joint"},
        "joint3_yaw": {"shoulder_abduction_adduction_joint"},
    }
    names = list(relations.keys())
    fail_time = data["_events"]["failsafe"][0][0] if data["_events"]["failsafe"] else None
    state17 = next((t for t, _old, new in flight_state_changes(data["/dragon/flight_state"])
                    if new == 17), None)
    fall_time = first_fall_time(data["/dragon/ground_truth"])
    pdf_path = os.path.join(out_dir, "joint_mapping.pdf")

    with PdfPages(pdf_path) as pdf:
        for joint in names:
            fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
            source_ax, dragon_ax = axes

            for source in relations[joint]:
                xs, ys = values(data["/dracomancer/joint_states"], source)
                if xs:
                    style = "-" if source in primary_sources[joint] else ":"
                    width = 1.8 if source in primary_sources[joint] else 1.4
                    source_ax.plot(
                        xs, [math.degrees(y) for y in ys], style,
                        color=kinematic_color(source), linewidth=width, label=source)

            for topic, label, style, width in [
                ("/dragon/joint_states", "Robot actual", "-", 1.8),
                ("/dragon/joints_ctrl", "Robot command", "-", 1.5),
            ]:
                xs, ys = values(data[topic], joint)
                if xs:
                    color = (
                        command_color(joint)
                        if topic == "/dragon/joints_ctrl"
                        else kinematic_color(joint))
                    dragon_ax.plot(
                        xs, [math.degrees(y) for y in ys], style,
                        color=color, linewidth=width, label=label)

            for x, label, color in [
                (fail_time, "failsafe", "tab:red"),
                (state17, "flight_state 17", "tab:orange"),
                (fall_time, "z < 0.3 m", "tab:brown"),
            ]:
                if x is not None:
                    for ax in axes:
                        ax.axvline(x, color=color, linewidth=1.0, alpha=0.8)
                    source_ax.text(x, 0.98, label, color=color, rotation=90,
                                   va="top", ha="right",
                                   transform=source_ax.get_xaxis_transform())

            source_ax.set_ylabel("Device joint angle [deg]")
            dragon_ax.set_ylabel("Robot joint angle [deg]")
            dragon_ax.set_xlabel("Time [s]")
            for ax in axes:
                ax.grid(True, alpha=0.25)
                ax.legend(loc="upper right", fontsize=8)
                style_axis(ax, time_axis=True, data=data)
            fig.tight_layout()
            fig.savefig(os.path.join(out_dir, joint + ".png"), dpi=180)
            fig.savefig(os.path.join(out_dir, joint + ".svg"))
            pdf.savefig(fig)
            if show:
                plt.show()
            plt.close(fig)

    return pdf_path


def plot_joint_group_figures(plt, data, out_dir, show):
    os.makedirs(out_dir, exist_ok=True)
    groups = [
        {
            "name": "joint1",
            "device_ylabel": r"$q_{\mathrm{wf}}, q_{\mathrm{wa}}$ [deg]",
            "sources": [
                "wrist_flexion_extension_joint",
                "wrist_abduction_adduction_joint",
            ],
            "rho": "wrist_rho",
            "rho_label": r"$\rho_1$",
            "rho_center_deg": 0.0,
            "rho_axis_mode": "data",
            "q_tick_step_deg": 60.0,
            "joints": ["joint1_pitch", "joint1_yaw"],
        },
        {
            "name": "joint2",
            "device_ylabel": r"$q_{\mathrm{ef}}$ [deg]",
            "sources": ["elbow_flexion_extension_joint"],
            "rho": "elbow_rho",
            "rho_label": r"$\rho_2$",
            "rho_center_deg": -45.0,
            "rho_axis_mode": "center",
            "q_tick_step_deg": 30.0,
            "joints": ["joint2_pitch", "joint2_yaw"],
        },
        {
            "name": "joint3",
            "device_ylabel": r"$q_{\mathrm{sf}}, q_{\mathrm{sa}}$ [deg]",
            "sources": [
                "shoulder_flexion_extension_joint",
                "shoulder_abduction_adduction_joint",
            ],
            "rho": None,
            "rho_label": None,
            "rho_center_deg": None,
            "rho_axis_mode": None,
            "q_tick_step_deg": None,
            "joints": ["joint3_pitch", "joint3_yaw"],
        },
    ]
    paths = []
    for group in groups:
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        source_ax, pitch_ax, yaw_ax = axes
        source_degrees = []

        for source in group["sources"]:
            xs, ys = values(data["/dracomancer/joint_states"], source)
            if xs:
                ys_deg = [math.degrees(y) for y in ys]
                source_degrees.extend(ys_deg)
                source_ax.plot(
                    xs, ys_deg,
                    color=kinematic_color(source), linewidth=1.8,
                    label=device_joint_label(source))

        if group["rho"] is not None:
            ratio_ax = source_ax.twinx()
            ratio_ax.set_zorder(source_ax.get_zorder() - 1)
            source_ax.patch.set_visible(False)
            ratio_xs, ratio_ys = values(
                data["/dracomancer/joint_map/switch_ratio"], group["rho"])
            if ratio_xs:
                ratio_ax.plot(
                    ratio_xs, ratio_ys, "--", color=RATIO_COLOR,
                    linewidth=1.6, label=group["rho_label"], zorder=1)
            ratio_ax.set_ylabel(group["rho_label"])
            style_axis(ratio_ax, time_axis=True, data=data)
            if group["rho_axis_mode"] == "data":
                set_q_rho_ticks_from_data(
                    source_ax, ratio_ax, source_degrees,
                    step_deg=group["q_tick_step_deg"])
            else:
                set_q_rho_ticks(
                    source_ax, ratio_ax, group["rho_center_deg"],
                    step_deg=group["q_tick_step_deg"])
            lines, labels = source_ax.get_legend_handles_labels()
            r_lines, r_labels = ratio_ax.get_legend_handles_labels()
            source_ax.legend(lines + r_lines, labels + r_labels,
                             loc="upper right")
        else:
            source_ax.legend(loc="upper right")

        source_ax.set_ylabel(group["device_ylabel"])
        source_ax.grid(True, alpha=0.25)
        style_axis(source_ax, time_axis=True, data=data)

        for joint, dragon_ax in zip(group["joints"], [pitch_ax, yaw_ax]):
            for topic, label, style, width in [
                ("/dragon/joint_states", "actual", "-", 1.8),
                ("/dragon/joints_ctrl", "target", "--", 1.5),
            ]:
                xs, ys = values(data[topic], joint)
                if xs:
                    color = (
                        command_color(joint)
                        if topic == "/dragon/joints_ctrl"
                        else kinematic_color(joint))
                    dragon_ax.plot(
                        xs, [math.degrees(y) for y in ys], style,
                        color=color, linewidth=width, label=label)

            dragon_ax.set_ylabel(robot_joint_axis_label(joint))
            dragon_ax.grid(True, alpha=0.25)
            dragon_ax.legend(loc="upper right")
            style_axis(dragon_ax, time_axis=True, data=data)
        yaw_ax.set_xlabel("Time [s]")

        png_path = os.path.join(out_dir, group["name"] + "_mapping.png")
        svg_path = os.path.join(out_dir, group["name"] + "_mapping.svg")
        save_tight(fig, png_path, svg_path, h_pad=0.35)
        paths.extend([png_path, svg_path])
        if show:
            plt.show()
        plt.close(fig)
    return paths


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
    axes[1].set_ylim(bottom=0.0)

    xs, ys = values(data["/dracomancer/dragon_shape_safety_scale"], None)
    axes[2].plot(xs, ys, label="live safety scale")
    axes[2].set_ylabel("scale")
    axes[2].set_ylim(-0.05, 1.05)
    axes[2].legend(loc="upper right", fontsize=8)

    for key, label in [("x", "nav x"), ("y", "nav y"), ("z", "nav z")]:
        xs, ys = values(data["/dragon/uav/nav"], key)
        axes[3].plot(xs, ys, label=label)
    axes[3].set_ylabel("COG target [m]")
    axes[3].set_xlabel("Time [s]")
    axes[3].legend(loc="upper right", fontsize=8)

    for ax in axes:
        ax.grid(True, alpha=0.25)
        style_axis(ax, time_axis=True, data=data)
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


def plot_fc_height_relation(plt, data, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    xs, ys = values(data["/dragon/uav/cog/odom"], "z")
    axes[0].plot(xs, ys, color="black")
    axes[0].set_ylabel("COG altitude [m]")

    rows = [
        (axes[1], "Force fc", "/dracomancer/candidate/fc_f_min",
         "/dracomancer/target/fc_f_min", "/dragon/debug/fc_f_min_filtered",
         "/dracomancer/force_volume_radius_threshold"),
        (axes[2], "Torque fc", "/dracomancer/candidate/fc_t_min",
         "/dracomancer/target/fc_t_min", "/dragon/debug/fc_t_min_filtered",
         "/dracomancer/torque_volume_radius_threshold"),
    ]
    for ax, ylabel, candidate_topic, target_topic, measured_topic, threshold_topic in rows:
        for topic, label, style in [
            (candidate_topic, "predicted candidate", "-"),
            (target_topic, "predicted target", "--"),
            (measured_topic, "measured", "-"),
        ]:
            xs, ys = values(data[topic], None)
            if xs:
                if topic == measured_topic:
                    ys = lowpass_values(xs, ys)
                    label = "measured low-pass"
                ax.plot(xs, ys, style, label=label)
        if data[threshold_topic]:
            hard, recover = data[threshold_topic][0][1][0:2]
            ax.axhline(hard, color="tab:red", linestyle=":", linewidth=2.0,
                       label="hard threshold")
            ax.axhline(recover, color="tab:green", linestyle=":", linewidth=2.0,
                       label="safe threshold")
        ax.set_ylabel(ylabel)
        ax.set_yscale("symlog", linthresh=0.02)
        ax.set_ylim(bottom=0.0)
        ax.legend(loc="upper right")

    axes[2].set_xlabel("Time [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        style_axis(ax, time_axis=True, data=data)
    fig.tight_layout()
    png_path = os.path.join(out_dir, "fc_height_relation.png")
    svg_path = os.path.join(out_dir, "fc_height_relation.svg")
    fig.savefig(png_path, dpi=220)
    fig.savefig(svg_path)
    plt.close(fig)
    return png_path, svg_path


def plot_inradius_axis(ax, data, predicted_topic, measured_topic,
                       threshold_topic, ylabel, predicted_color,
                       measured_color, hard_color, min_color, symbol):
    radius_values = []
    rows = [
        (predicted_topic, r"predicted $%s$" % symbol, predicted_color, "-"),
        (measured_topic, r"measured $%s$ (LPF)" % symbol, measured_color, "--"),
    ]
    for topic, label, line_color, style in rows:
        xs, ys = values(data[topic], None)
        if xs:
            # Strong smoothing is applied only for this paper comparison figure.
            ys = lowpass_values(xs, ys, cutoff_hz=INRADIUS_LOWPASS_CUTOFF_HZ)
            radius_values.extend(ys)
            ax.plot(xs, ys, style, color=line_color, linewidth=1.8, label=label)

    if data[threshold_topic]:
        hard, recover = data[threshold_topic][0][1][0:2]
        radius_values.append(hard)
        radius_values.append(recover)
        ax.axhline(hard, color=hard_color, linestyle=":", linewidth=1.4,
                   label=r"$%s^{\mathrm{hard}}$" % symbol)
        ax.axhline(recover, color=min_color, linestyle="-.", linewidth=1.4,
                   label=r"$%s^{\mathrm{min}}$" % symbol)

    ax.set_ylabel(ylabel)
    upper = percentile(radius_values, 0.95)
    ax.set_ylim(0.0, nice_upper_limit((upper or 1.0) * 1.15))
    ax.legend(
        loc="upper center", bbox_to_anchor=(0.5, -0.01), ncol=4,
        columnspacing=0.7, handlelength=1.5, handletextpad=0.35,
        labelspacing=0.15, borderpad=0.25, borderaxespad=0.0,
        frameon=True)


def plot_inradius_cog_relation(plt, data, out_dir, include_target=False,
                               name="inradius_cog_relation"):
    nrows = 4 if include_target else 3
    fig_height = 12.0 if include_target else 10.0
    fig, axes = plt.subplots(nrows, 1, figsize=(12, fig_height), sharex=True)
    force_ax, torque_ax, cog_ax = axes[:3]

    plot_inradius_axis(
        force_ax, data,
        "/dracomancer/target/fc_f_min",
        "/dragon/debug/fc_f_min_filtered",
        "/dracomancer/force_volume_radius_threshold",
        r"$r_f$", FORCE_PREDICTED_COLOR, FORCE_MEASURED_COLOR,
        THRESHOLD_HARD_COLOR, FORCE_THRESHOLD_MIN_COLOR, "r_f")
    plot_inradius_axis(
        torque_ax, data,
        "/dracomancer/target/fc_t_min",
        "/dragon/debug/fc_t_min_filtered",
        "/dracomancer/torque_volume_radius_threshold",
        r"$r_\tau$", TORQUE_PREDICTED_COLOR, TORQUE_MEASURED_COLOR,
        THRESHOLD_HARD_COLOR, TORQUE_THRESHOLD_MIN_COLOR, r"r_\tau")

    xs, ys = values(data["/dragon/uav/cog/odom"], "z")
    if xs:
        label = r"$^{\{W\}}p_{C,z}$" if include_target else None
        cog_ax.plot(xs, ys, color=ROLL_COLOR, linewidth=1.8, label=label)
    if include_target:
        xs, ys = values(data["/dragon/uav/nav"], "z")
        if xs:
            cog_ax.plot(
                xs, ys, color=ROLL_COMMAND_COLOR, linestyle="--",
                linewidth=1.8, label=r"$^{\{W\}}p_{C,z}^{\mathrm{target}}$")
    cog_ax.axhline(1.0, color="black", linestyle="--", linewidth=1.6)
    cog_ax.set_ylabel(r"$^{\{W\}}p_{C,z}$ [m]")
    if not include_target:
        cog_ax.set_xlabel("Time [s]")
    if include_target:
        cog_ax.legend(loc="upper right")
        rmse_ax = axes[3]
        add_cog_target_rmse_axis(rmse_ax, data)
        rmse_ax.set_xlabel("Time [s]")

    for ax in axes:
        ax.grid(True, alpha=0.25)
        style_axis(ax, time_axis=True, data=data)

    png_path = os.path.join(out_dir, name + ".png")
    svg_path = os.path.join(out_dir, name + ".svg")
    save_tight(fig, png_path, svg_path, h_pad=0.85 if include_target else 0.35)
    plt.close(fig)
    return png_path, svg_path


def add_cog_target_rmse_axis(ax, data, window_sec=5.0):
    for key, color, label in [
            ("x", ROLL_COLOR, r"$^{\{W\}}p_{C,x}$"),
            ("y", PITCH_COLOR, r"$^{\{W\}}p_{C,y}$"),
            ("z", YAW_COLOR, r"$^{\{W\}}p_{C,z}$")]:
        xs, errors = step_aligned_error(
            data["/dragon/uav/cog/odom"], data["/dragon/uav/nav"], key)
        rmse = rolling_rmse(xs, errors, window_sec)
        overall_rmse = (
            math.sqrt(sum(error * error for error in errors) / float(len(errors)))
            if errors else None)
        if xs and rmse:
            if overall_rmse is not None:
                line_label = r"%s (%.3f m)" % (label, overall_rmse)
            else:
                line_label = label
            ax.plot(xs, rmse, color=color, linewidth=1.8, label=line_label)
    ax.set_ylabel("RMSE [m]")
    ax.legend(loc="upper right")


def plot_cog_target_rmse(plt, data, out_dir):
    fig, ax = plt.subplots(figsize=(12, 4.5))
    add_cog_target_rmse_axis(ax, data)
    ax.set_xlabel("Time [s]")
    ax.grid(True, alpha=0.25)
    style_axis(ax, time_axis=True, data=data)

    png_path = os.path.join(out_dir, "cog_target_rmse.png")
    svg_path = os.path.join(out_dir, "cog_target_rmse.svg")
    save_tight(fig, png_path, svg_path)
    plt.close(fig)
    return png_path, svg_path


def plot_link4_cog_position(plt, data, out_dir):
    fig = plt.figure(figsize=(13, 10))
    ax_xy = fig.add_subplot(2, 2, 1)
    ax_z = fig.add_subplot(2, 2, 2)
    ax_xyz = fig.add_subplot(2, 1, 2)

    link4 = relative_pose_series(data["/gazebo/link_states"])
    cog = relative_pose_series(data["/dragon/uav/cog/odom"])

    for series, label, color in [
        (link4, "link4", "tab:blue"),
        (cog, "CoG", "tab:orange"),
    ]:
        x_t, x = values(series, "x")
        _, y = values(series, "y")
        _, z = values(series, "z")
        if x and y:
            ax_xy.plot(x, y, color=color, label=label)
        if x_t and z:
            ax_z.plot(x_t, z, color=color, label=label)
        if x_t:
            ax_xyz.plot(x_t, x, color=color, linestyle="-", label="%s x" % label)
            ax_xyz.plot(x_t, y, color=color, linestyle="--", label="%s y" % label)
            ax_xyz.plot(x_t, z, color=color, linestyle=":", label="%s z" % label)

    ax_xy.set_xlabel("relative x [m]")
    ax_xy.set_ylabel("relative y [m]")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.legend(loc="best")

    ax_z.set_xlabel("Time [s]")
    ax_z.set_ylabel("relative z [m]")
    ax_z.legend(loc="best")

    ax_xyz.set_xlabel("Time [s]")
    ax_xyz.set_ylabel("relative position [m]")
    ax_xyz.legend(loc="upper right", ncol=3)

    for ax in [ax_xy, ax_z, ax_xyz]:
        ax.grid(True, alpha=0.25)
        style_axis(ax, time_axis=(ax is not ax_xy), data=data)

    fig.tight_layout()
    png_path = os.path.join(out_dir, "link4_cog_position.png")
    svg_path = os.path.join(out_dir, "link4_cog_position.svg")
    fig.savefig(png_path, dpi=220)
    fig.savefig(svg_path)
    plt.close(fig)
    return png_path, svg_path


def plot_link4_attitude(plt, data, out_dir):
    fig, ax = plt.subplots(figsize=(12, 4.5))
    for key, label in [
            ("roll", r"$\theta^{\mathrm{tail}}_x$"),
            ("pitch", r"$\theta^{\mathrm{tail}}_y$"),
            ("yaw", r"$\theta^{\mathrm{tail}}_z$")]:
        xs, ys = values(data["/gazebo/link_states"], key)
        if xs:
            ys_deg = [math.degrees(y) for y in ys]
            ax.plot(
                xs, ys_deg,
                color=kinematic_color(key), label=label)
    ax.set_ylabel(
        r"$^{\{W\}}\boldsymbol{\theta}^{\mathrm{tail}}$ [deg]",
        usetex=True)
    ax.set_xlabel("Time [s]")
    ax.grid(True, alpha=0.25)
    ax.yaxis.set_major_locator(MultipleLocator(90.0))
    ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), borderaxespad=0.0)
    style_axis(ax, time_axis=True, data=data)
    png_path = os.path.join(out_dir, "link4_attitude.png")
    svg_path = os.path.join(out_dir, "link4_attitude.svg")
    save_tight(fig, png_path, svg_path)
    plt.close(fig)
    return png_path, svg_path


def plot_link4_position(plt, data, out_dir):
    fig, ax = plt.subplots(figsize=(12, 4.5))
    add_link4_position_axis(ax, data)
    ax.set_xlabel("Time [s]")
    png_path = os.path.join(out_dir, "link4_position.png")
    svg_path = os.path.join(out_dir, "link4_position.svg")
    save_tight(fig, png_path, svg_path)
    plt.close(fig)
    return png_path, svg_path


def add_link4_position_axis(ax, data):
    for key, label, color in [
            ("x", r"$p^{\mathrm{tail}}_x$", ROLL_COLOR),
            ("y", r"$p^{\mathrm{tail}}_y$", PITCH_COLOR),
            ("z", r"$p^{\mathrm{tail}}_z$", YAW_COLOR)]:
        xs, ys = values(data["/gazebo/link_states"], key)
        if xs:
            ax.plot(xs, ys, color=color, label=label)
            # The link4 anchor target is not recorded as its own topic in this bag.
            # In position_only mode it is the captured hover-start link4 position.
            ax.axhline(ys[0], color=color, linestyle="--", linewidth=1.4,
                       label=r"$p_{\mathrm{ref},%s}$" % key)
    ax.set_ylabel(
        r"$^{\{W\}}\boldsymbol{p}^{\mathrm{tail}}$ [m]",
        usetex=True)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), borderaxespad=0.0)
    style_axis(ax, time_axis=True, data=data)


def add_link4_attitude_axis(ax, data):
    for key, label in [
            ("roll", r"$\theta^{\mathrm{tail}}_x$"),
            ("pitch", r"$\theta^{\mathrm{tail}}_y$"),
            ("yaw", r"$\theta^{\mathrm{tail}}_z$")]:
        xs, ys = values(data["/gazebo/link_states"], key)
        if xs:
            ys_deg = [math.degrees(y) for y in ys]
            ax.plot(xs, ys_deg, color=kinematic_color(key), label=label)
    ax.set_ylabel(
        r"$^{\{W\}}\boldsymbol{\theta}^{\mathrm{tail}}$ [deg]",
        usetex=True)
    ax.grid(True, alpha=0.25)
    ax.yaxis.set_major_locator(MultipleLocator(90.0))
    ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), borderaxespad=0.0)
    style_axis(ax, time_axis=True, data=data)


def plot_link4_pose(plt, data, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8.2), sharex=True)
    pos_ax, att_ax = axes
    add_link4_position_axis(pos_ax, data)
    add_link4_attitude_axis(att_ax, data)
    for ax in axes:
        ax.yaxis.set_label_coords(-0.105, 0.5)
    att_ax.set_xlabel("Time [s]")
    png_path = os.path.join(out_dir, "link4_pose.png")
    svg_path = os.path.join(out_dir, "link4_pose.svg")
    save_tight(fig, png_path, svg_path, h_pad=0.2)
    plt.close(fig)
    return png_path, svg_path


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
    for rows, axis, _title, threshold_topic in [
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
        axis.set_ylabel("fc radius")
        axis.set_yscale("symlog", linthresh=0.02)
        axis.set_ylim(bottom=0.0)
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
    axes[2].set_xlabel("Time [s]")
    axes[2].set_ylabel("actual - predicted")
    axes[2].legend(loc="upper right", fontsize=8)

    for ax in axes:
        ax.grid(True, alpha=0.25)
        style_axis(ax, time_axis=True, data=data)
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


def write_summary(start_abs, data, out_dir, pdf_path, safety_paths, fc_error_paths,
                  fc_height_paths, link4_cog_paths):
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
        f.write("teleoperation_origin_rt: %.3f\n" % data.get("_time_origin_rt", 0.0))
        if data.get("_time_end_rt") is not None:
            f.write("teleoperation_end_rt: %.3f\n" % data["_time_end_rt"])
            f.write("teleoperation_duration: %.3f\n" % data.get("_time_end", 0.0))
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
        f.write("fc_height_relation_png: %s\n" % fc_height_paths[0])
        f.write("fc_height_relation_svg: %s\n" % fc_height_paths[1])
        f.write("link4_cog_position_png: %s\n" % link4_cog_paths[0])
        f.write("link4_cog_position_svg: %s\n" % link4_cog_paths[1])
    return path


def main():
    args = parse_args()
    plt, PdfPages = setup_matplotlib(args.show, args.paper_style)
    start_abs, data = collect_bag(os.path.expanduser(args.bag))
    data = rebase_time(data, teleoperation_origin(data), teleoperation_end(data))
    del PdfPages, start_abs
    joint_paths = plot_joint_group_figures(plt, data, args.out_dir, args.show)
    link4_attitude_paths = plot_link4_attitude(plt, data, args.out_dir)
    link4_position_paths = plot_link4_position(plt, data, args.out_dir)
    link4_pose_paths = plot_link4_pose(plt, data, args.out_dir)
    inradius_cog_paths = plot_inradius_cog_relation(plt, data, args.out_dir)
    inradius_cog_target_paths = plot_inradius_cog_relation(
        plt, data, args.out_dir, include_target=True,
        name="inradius_cog_relation_with_target")
    cog_target_rmse_paths = plot_cog_target_rmse(plt, data, args.out_dir)
    print("wrote", args.out_dir)
    for path in (joint_paths + list(link4_attitude_paths) +
                 list(link4_position_paths) + list(link4_pose_paths) +
                 list(inradius_cog_paths) + list(inradius_cog_target_paths) +
                 list(cog_target_rmse_paths)):
        print("figure", path)


if __name__ == "__main__":
    main()
