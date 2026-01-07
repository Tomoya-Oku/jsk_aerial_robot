#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import rospy
import numpy as np

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def parse_args():
    ap = argparse.ArgumentParser(
        description="Publish one JointTrajectory to set ME6 joints"
    )
    ap.add_argument(
        "joints",
        nargs="+",
        help="joint values (radians by default; use --deg for degrees)",
    )
    ap.add_argument(
        "--deg",
        action="store_true",
        help="interpret joint values in degrees",
    )
    ap.add_argument(
        "--joint_names",
        nargs="+",
        default=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        help="joint name list",
    )
    ap.add_argument(
        "--cmd_topic",
        default="/me6_robot/joint_controller/command",
        help="command topic (JointTrajectory)",
    )
    ap.add_argument(
        "--joint_state_topic",
        default="/me6_robot/joint_states",
        help="joint_states topic (for validation)",
    )
    ap.add_argument(
        "--time",
        type=float,
        default=2.0,
        help="time_from_start for trajectory point [s]",
    )
    ap.add_argument(
        "--wait",
        type=float,
        default=1.0,
        help="seconds to wait after publish",
    )
    return ap.parse_args()


def main():
    args = parse_args()
    rospy.init_node("me6_set_joints_once")

    if len(args.joints) != len(args.joint_names):
        raise SystemExit(
            f"Expected {len(args.joint_names)} joint values, got {len(args.joints)}"
        )

    vals = np.array([float(v) for v in args.joints], dtype=float)
    if args.deg:
        vals = vals * np.pi / 180.0

    # Optional check: wait for joint_states so topics are alive
    try:
        rospy.wait_for_message(args.joint_state_topic, JointState, timeout=2.0)
    except Exception:
        pass

    pub = rospy.Publisher(args.cmd_topic, JointTrajectory, queue_size=1, latch=True)
    rospy.sleep(0.2)

    traj = JointTrajectory()
    traj.joint_names = list(args.joint_names)

    pt = JointTrajectoryPoint()
    pt.positions = vals.tolist()
    pt.time_from_start = rospy.Duration(args.time)
    traj.points = [pt]

    pub.publish(traj)
    rospy.loginfo("Published JointTrajectory to %s", args.cmd_topic)

    if args.wait > 0:
        rospy.sleep(args.wait)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
