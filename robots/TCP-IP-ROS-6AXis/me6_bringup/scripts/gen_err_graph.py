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

    ap.add_argument(
        "--target_topic",
        default="/following_waypoints/target_point",
        help="target point topic (PointStamped)",
    )
    ap.add_argument(
        "--ee_topic",
        default="/me6_robot/ee_pose",
        help="ee pose topic (PoseStamped)",
    )
    ap.add_argument(
        "--fk_err_topic",
        default="/following_waypoints/fk_err",
        help="error norm topic (Float64)",
    )
    ap.add_argument(
        "--fk_err_vec_topic",
        default="/following_waypoints/fk_err_vec",
        help="error vector topic (PointStamped)",
    )
    ap.add_argument(
        "--use_relative_time",
        action="store_true",
        help="時刻を先頭=0[s] にする",
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

    if not args.title:
        args.title = "target vs ee_pose and fk error"

    done_t = None

    # --- read bag ---
    t_target = []
    target_xyz = []

    t_ee = []
    ee_xyz = []

    t_err = []
    err_norm = []

    t_errv = []
    err_xyz = []

    with rosbag.Bag(args.bag, "r") as bag:
        topics = [
            args.target_topic,
            args.ee_topic,
            args.fk_err_topic,
            args.fk_err_vec_topic,
            args.done_topic,
        ]
        for topic, msg, t in bag.read_messages(topics=topics):
            ts = get_stamp_sec(msg, t)

            if topic == args.target_topic:
                try:
                    p = msg.point
                    t_target.append(ts)
                    target_xyz.append([float(p.x), float(p.y), float(p.z)])
                except Exception:
                    continue
            elif topic == args.ee_topic:
                try:
                    p = msg.pose.position
                    t_ee.append(ts)
                    ee_xyz.append([float(p.x), float(p.y), float(p.z)])
                except Exception:
                    continue
            elif topic == args.fk_err_topic:
                try:
                    t_err.append(ts)
                    err_norm.append(float(msg.data))
                except Exception:
                    continue
            elif topic == args.fk_err_vec_topic:
                try:
                    p = msg.point
                    t_errv.append(ts)
                    err_xyz.append([float(p.x), float(p.y), float(p.z)])
                except Exception:
                    continue
            elif topic == args.done_topic:
                if done_t is None:
                    done_t = ts

    if len(t_target) == 0:
        raise SystemExit(f"No data found on {args.target_topic}.")
    if len(t_ee) == 0:
        raise SystemExit(f"No data found on {args.ee_topic}.")

    t_target = np.array(t_target, dtype=float)
    target_xyz = np.array(target_xyz, dtype=float)

    t_ee = np.array(t_ee, dtype=float)
    ee_xyz = np.array(ee_xyz, dtype=float)

    t_err = np.array(t_err, dtype=float)
    err_norm = np.array(err_norm, dtype=float)

    t_errv = np.array(t_errv, dtype=float)
    err_xyz = np.array(err_xyz, dtype=float)

    # --- sort by time ---
    ord_t = np.argsort(t_target)
    t_target = t_target[ord_t]
    target_xyz = target_xyz[ord_t, :]

    ord_ee = np.argsort(t_ee)
    t_ee = t_ee[ord_ee]
    ee_xyz = ee_xyz[ord_ee, :]

    if len(t_err) > 0:
        ord_e = np.argsort(t_err)
        t_err = t_err[ord_e]
        err_norm = err_norm[ord_e]

    if len(t_errv) > 0:
        ord_ev = np.argsort(t_errv)
        t_errv = t_errv[ord_ev]
        err_xyz = err_xyz[ord_ev, :]

    # relative time option
    if args.use_relative_time:
        t0 = t_target[0]
        t_target = t_target - t0
        t_ee = t_ee - t0
        if len(t_err) > 0:
            t_err = t_err - t0
        if len(t_errv) > 0:
            t_errv = t_errv - t0
        if done_t is not None:
            done_t = done_t - t0

    t_end = done_t if done_t is not None else t_target[-1]

    # --- plot ---
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(12, 8))

    labels = ["x", "y", "z"]
    colors = ["r", "g", "b"]
    for i, ax in enumerate(axes[:3]):
        ax.plot(t_target, target_xyz[:, i], label=f"target/{labels[i]}", color="k")
        ax.plot(t_ee, ee_xyz[:, i], label=f"ee_pose/{labels[i]}", color=colors[i])
        ax.set_ylabel(f"{labels[i]} [m]")
        ax.grid(True)
        ax.legend(loc="upper right", fontsize="x-small")

    ax_err = axes[3]
    if len(t_err) > 0:
        ax_err.plot(t_err, err_norm, label="fk_err", linewidth=1.5)
    ax_err.set_ylabel("error [m]")
    ax_err.grid(True)
    ax_err.legend(loc="upper right", fontsize="x-small")

    axes[-1].set_xlabel("time [s]")
    axes[-1].set_xlim(0, t_end)
    fig.suptitle(args.title)
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    if args.out:
        plt.savefig(args.out, dpi=200, bbox_inches="tight")
        print(f"Saved: {args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
