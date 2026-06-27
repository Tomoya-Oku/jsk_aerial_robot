#!/usr/bin/env python3

import argparse
import sys
import time

import rospy
from std_msgs.msg import String


VALID_MODES = ("startup", "precision", "wide")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Switch the running Dracomancer teleoperation mode."
    )
    parser.add_argument(
        "mode",
        choices=VALID_MODES,
        help="teleoperation mode to publish",
    )
    parser.add_argument(
        "--topic",
        default="/dracomancer/teleop_mode",
        help="mode switch topic (default: /dracomancer/teleop_mode)",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=2.0,
        help="seconds to wait for subscribers before publishing (default: 2.0)",
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main():
    args = parse_args()
    rospy.init_node("dracomancer_set_teleop_mode", anonymous=True)

    pub = rospy.Publisher(args.topic, String, queue_size=1, latch=True)
    deadline = time.monotonic() + args.wait_timeout
    rate = rospy.Rate(20)
    while pub.get_num_connections() == 0 and time.monotonic() < deadline and not rospy.is_shutdown():
        rate.sleep()

    pub.publish(String(data=args.mode))
    rospy.sleep(0.2)
    rospy.loginfo("published teleop mode '%s' to %s", args.mode, args.topic)


if __name__ == "__main__":
    main()
