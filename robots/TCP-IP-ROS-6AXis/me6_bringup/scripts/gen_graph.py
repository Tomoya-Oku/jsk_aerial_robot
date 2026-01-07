#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import numpy as np
import rosbag
import matplotlib.pyplot as plt
import tf.transformations as tft


def get_stamp_sec(msg, bag_t):
    """msg.header.stamp が使えればそれ、無ければ bag 時刻"""
    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        try:
            s = msg.header.stamp.to_sec()
            if s > 0:
                return s
        except Exception:
            pass
    return bag_t.to_sec()


def main():
    ap = argparse.ArgumentParser()

    ap.add_argument("bag", help="path to rosbag (.bag)")

    ap.add_argument(
        "--ord",
        choices=["x", "y", "z", "roll", "pitch", "yaw"],
        default="x",
        help=(
            "axis to plot: x/y/z -> way_points.poses[].position.*; "
            "roll/pitch/yaw -> way_points.poses[].orientation (rpy) vs ee_pose"
        ),
    )

    ap.add_argument(
        "--wp_topic",
        default="/way_points",
        help="waypoint topic (PoseArray) (default: /way_points)",
    )
    ap.add_argument(
        "--ee_topic",
        default="/me6_robot/ee_pose",
        help="PoseStamped topic (default: /me6_robot/ee_pose)",
    )

    ap.add_argument(
        "--n",
        type=int,
        default=20,
        help="way_points.poses[0..n-1] を描く (default: 20)",
    )
    ap.add_argument(
        "--use_relative_time",
        action="store_true",
        help="時刻を waypoints の先頭=0[s] にする",
    )
    ap.add_argument(
        "--out",
        default="",
        help="保存先ファイル名（例: out.png）。空なら表示のみ",
    )
    ap.add_argument(
        "--title",
        default="",
        help="グラフタイトル（空なら自動）",
    )
    ap.add_argument(
        "--done_topic",
        default="/following_waypoints/done",
        help="done signal topic (std_msgs/Empty) recorded in bag",
    )

    args = ap.parse_args()

    # topics
    wp_topic = args.wp_topic
    ee_topic = args.ee_topic

    # attribute name for ee_pose
    coord_attr = args.ord  # "x" | "y" | "z" | "roll" | "pitch" | "yaw"

    # default title
    if not args.title:
        if coord_attr in ["roll", "pitch", "yaw"]:
            args.title = f"{wp_topic}.poses[0..{args.n-1}].orientation.{coord_attr} vs {ee_topic}.orientation.{coord_attr}"
        else:
            args.title = f"{wp_topic}.poses[0..{args.n-1}].position.{coord_attr} vs {ee_topic}.position.{coord_attr}"

    done_t = None

    with rosbag.Bag(args.bag, "r") as bag:
        topics = [wp_topic, ee_topic, args.done_topic]
        for topic, msg, t in bag.read_messages(topics=topics):
            ts = get_stamp_sec(msg, t)

            if topic == wp_topic:
                ...
            elif topic == ee_topic:
                ...
            elif topic == args.done_topic:
                done_t = ts  # 最初の1回だけでよければ if done_t is None: done_t = ts

    wp_t = []
    wp_mat = []  # shape: (N, n)

    ee_t = []
    ee_v = []

    # --- read bag ---
    with rosbag.Bag(args.bag, "r") as bag:
        topics = [wp_topic, ee_topic]
        for topic, msg, t in bag.read_messages(topics=topics):
            ts = get_stamp_sec(msg, t)

            if topic == wp_topic:
                poses = list(getattr(msg, "poses", []))
                if len(poses) < args.n:
                    continue
                wp_t.append(ts)
                if coord_attr in ["roll", "pitch", "yaw"]:
                    row = []
                    for p in poses[:args.n]:
                        q = p.orientation
                        r, pch, y = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
                        row.append({"roll": r, "pitch": pch, "yaw": y}[coord_attr])
                else:
                    row = [float(getattr(p.position, coord_attr)) for p in poses[:args.n]]
                wp_mat.append(row)

            elif topic == ee_topic:
                try:
                    if coord_attr in ["roll", "pitch", "yaw"]:
                        q = msg.pose.orientation
                        r, p, y = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
                        v = {"roll": r, "pitch": p, "yaw": y}[coord_attr]
                    else:
                        v = getattr(msg.pose.position, coord_attr)
                    ee_t.append(ts)
                    ee_v.append(float(v))
                except Exception:
                    continue

    if len(wp_t) == 0:
        raise SystemExit(f"No data found on {wp_topic} with poses length >= {args.n}.")
    if len(ee_t) == 0:
        raise SystemExit(f"No data found on {ee_topic}.")

    wp_t = np.array(wp_t, dtype=float)
    wp_mat = np.array(wp_mat, dtype=float)  # (N, n)

    ee_t = np.array(ee_t, dtype=float)
    ee_v = np.array(ee_v, dtype=float)

    # --- sort by time ---
    wp_ord = np.argsort(wp_t)
    wp_t = wp_t[wp_ord]
    wp_mat = wp_mat[wp_ord, :]

    ee_ord = np.argsort(ee_t)
    ee_t = ee_t[ee_ord]
    ee_v = ee_v[ee_ord]

    # relative time option (t=0 at first waypoint)
    if args.use_relative_time:
        t0 = wp_t[0]
        wp_t = wp_t - t0
        ee_t = ee_t - t0
        if done_t is not None:
            done_t = done_t - t0

    # --- plot ---
    plt.figure()

    # waypoints 0..n-1
    for i in range(args.n):
        plt.plot(wp_t, wp_mat[:, i], label=f"wp_{coord_attr}[{i}]", alpha=0.6)

    # ee pose (raw)
    plt.plot(ee_t, ee_v, label=f"ee_pose.{coord_attr}", linewidth=2.0)

    # xlim end
    t_end = done_t if done_t is not None else wp_t[-1]
    plt.xlim(0, t_end)

    plt.xlabel("time [s]")
    if coord_attr in ["roll", "pitch", "yaw"]:
        plt.ylabel(f"{coord_attr} [rad]")
    else:
        plt.ylabel(f"{coord_attr} [m]")
    plt.title(args.title)
    plt.grid(True)

    # 凡例は必要なら有効化
    # plt.legend(fontsize="x-small", ncol=3)

    if args.out:
        plt.savefig(args.out, dpi=200, bbox_inches="tight")
        print(f"Saved: {args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
