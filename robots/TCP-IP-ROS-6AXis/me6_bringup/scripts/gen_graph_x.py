#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import numpy as np
import rosbag
import matplotlib.pyplot as plt


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
    ap.add_argument("--wp_topic", default="/way_points_x",
                    help="Float64MultiArray topic (default: /way_points_x)")
    ap.add_argument("--ee_topic", default="/me6_robot/ee_pose",
                    help="PoseStamped topic (default: /me6_robot/ee_pose)")
    ap.add_argument("--n", type=int, default=20,
                    help="way_points_x.data[0..n-1] を描く (default: 20)")
    ap.add_argument("--use_relative_time", action="store_true",
                    help="時刻を先頭=0[s] にする")
    ap.add_argument("--out", default="",
                    help="保存先ファイル名（例: out.png）。空なら表示のみ")
    ap.add_argument("--title", default="way_points_x.data[0..19] vs ee_pose.x",
                    help="グラフタイトル")
    args = ap.parse_args()

    wp_t = []
    wp_mat = []  # shape: (N, n)

    ee_t = []
    ee_x = []

    # --- read bag ---
    with rosbag.Bag(args.bag, "r") as bag:
        topics = [args.wp_topic, args.ee_topic]
        for topic, msg, t in bag.read_messages(topics=topics):
            ts = get_stamp_sec(msg, t)

            if topic == args.wp_topic:
                data = list(getattr(msg, "data", []))
                if len(data) < args.n:
                    continue
                wp_t.append(ts)
                wp_mat.append([float(v) for v in data[:args.n]])

            elif topic == args.ee_topic:
                try:
                    ee_t.append(ts)
                    ee_x.append(float(msg.pose.position.x))
                except Exception:
                    continue

    if len(wp_t) == 0:
        raise SystemExit(f"No data found on {args.wp_topic} with length >= {args.n}.")
    if len(ee_t) == 0:
        raise SystemExit(f"No data found on {args.ee_topic}.")

    wp_t = np.array(wp_t, dtype=float)
    wp_mat = np.array(wp_mat, dtype=float)  # (N, n)

    ee_t = np.array(ee_t, dtype=float)
    ee_x = np.array(ee_x, dtype=float)

    # --- sort by time ---
    wp_ord = np.argsort(wp_t)
    wp_t = wp_t[wp_ord]
    wp_mat = wp_mat[wp_ord, :]

    ee_ord = np.argsort(ee_t)
    ee_t = ee_t[ee_ord]
    ee_x = ee_x[ee_ord]

    # --- relative time option (t=0 at first waypoint) ---
    if args.use_relative_time:
        t0 = wp_t[0]   # ★ waypoints が最初に出た時刻を 0 にする
        wp_t = wp_t - t0
        ee_t = ee_t - t0

    # --- interpolate ee onto wp time axis ---
    ee_x_interp = np.interp(wp_t, ee_t, ee_x)

    # --- plot ---
    plt.figure()

    # waypoints 0..n-1
    for i in range(args.n):
        plt.plot(wp_t, wp_mat[:, i], label=f"wp_x[{i}]", alpha=0.6)

    # ee pose (raw + interp)
    plt.plot(ee_t, ee_x, label="ee_pose.x (raw)", linewidth=2.0)
    plt.plot(wp_t, ee_x_interp, label="ee_pose.x (interp@wp_t)", linestyle="--", linewidth=2.0)

    plt.xlabel("time [s]" + (" (relative)" if args.use_relative_time else ""))
    plt.ylabel("x [m]")
    plt.title(args.title)
    plt.grid(True)

    plt.ylim(0, max(wp_mat[:, 0]) + 0.1)

    # 凡例がデカくなるので小さめ＆複数列
    plt.legend(fontsize="x-small", ncol=3)

    if args.out:
        plt.savefig(args.out, dpi=200, bbox_inches="tight")
        print(f"Saved: {args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
