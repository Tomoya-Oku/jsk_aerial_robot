#!/usr/bin/env python3
"""Optional Mk-II viewer input for the ground simulation demo."""
import math
import random

import rospy
from sensor_msgs.msg import JointState


class NaturalMotion:
    """Generate gradual paired arm movements from a reproducible random seed."""

    interval = 7.0

    def __init__(self, seed):
        self.rng = random.Random(seed)
        self.previous = self.target = [0.0] * 14
        self.segment = -1

    def keyframe(self):
        positions = []
        lift = self.rng.uniform(-1, 1)
        for _ in range(2):
            reach = self.rng.uniform(-1, 1)
            bend = self.rng.uniform(-1, 1)
            positions.extend((
                .24 * reach,
                .16 * lift + .11 * bend,
                self.rng.uniform(-.18, .18),
                -.40 - .23 * bend + .06 * reach,
                self.rng.uniform(-.28, .28),
                .13 * bend + self.rng.uniform(-.09, .09),
                self.rng.uniform(-.16, .16),
            ))
        return positions

    def position(self, elapsed):
        segment = int(elapsed / self.interval)
        while self.segment < segment:
            self.previous, self.target = self.target, self.keyframe()
            self.segment += 1
        phase = elapsed / self.interval - segment
        # Zero velocity and acceleration at each keyframe.
        blend = phase ** 3 * (10 + phase * (-15 + 6 * phase))
        return [start + blend * (end - start) for start, end in zip(self.previous, self.target)]


if __name__ == "__main__":
    rospy.init_node("shape_input")
    if not rospy.get_param("/use_sim_time", False):
        raise RuntimeError("shape_input requires simulation time")
    pattern = rospy.get_param("~pattern", "sine")
    if pattern not in ("sine", "natural"):
        raise ValueError("shape_input pattern must be sine or natural")
    natural = NaturalMotion(rospy.get_param("~seed", 27))
    publisher = rospy.Publisher("/dracomancer/joint_cmd", JointState, queue_size=1)
    names = ["%s_joint_%d" % (side, i) for side in ("right", "left") for i in range(1, 8)]
    rate = rospy.Rate(40)
    start = None
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if now == rospy.Time(0):
            rate.sleep()
            continue
        if start is None:
            start = now
        # Allow five seconds for Gazebo quick_init_pose before moving the arms.
        elapsed = max(0., (now-start).to_sec()-5.)
        msg = JointState()
        msg.header.stamp, msg.name = now, names
        if pattern == "natural":
            msg.position = natural.position(elapsed)
        else:
            msg.position = [.25*math.sin(.3*elapsed), 0., 0., .4*math.sin(.2*elapsed), 0., 0., 0.] + [0.]*7
        publisher.publish(msg)
        rate.sleep()
