#!/usr/bin/env python3
"""Optional deterministic Mk-II viewer input for the ground simulation demo."""
import math

import rospy
from sensor_msgs.msg import JointState


if __name__ == "__main__":
    rospy.init_node("shape_input")
    if not rospy.get_param("/use_sim_time", False):
        raise RuntimeError("shape_input requires simulation time")
    publisher = rospy.Publisher("/dracomancer_mkii/joint_cmd", JointState, queue_size=1)
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
        # Five seconds for Gazebo quick_init_pose; smooth right shoulder/elbow
        # variations thereafter. Only the viewer's input topic is driven here.
        elapsed = max(0., (now-start).to_sec()-5.)
        msg = JointState()
        msg.header.stamp, msg.name = now, names
        msg.position = [.25*math.sin(.3*elapsed), 0., 0., .4*math.sin(.2*elapsed), 0., 0., 0.] + [0.]*7
        publisher.publish(msg)
        rate.sleep()
