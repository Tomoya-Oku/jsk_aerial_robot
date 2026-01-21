#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Teleoperation node to send Dracomancer joint angles to the Dragon robot.

Instead of reading the servo angles directly from the exoskeleton hardware,
this node subscribes to a JointState topic published by Spinal (the low-level
motor driver).  The JointState topic provides the current angles for each
joint, similar to how the Twin Hammer robot obtains its gimbal angles from the
'joint_states' topic:contentReference[oaicite:2]{index=2}.  By using Spinal as the
source of truth for joint angles we decouple the teleoperation logic from
hardware interfaces.

The node applies optional offsets to the received angles and publishes them
as a JointState message to a configured topic (default '/dragon/joints_ctrl').
All comments are written in English per the user's request.
"""

import rospy
import time
import tf.transformations as tf
from std_msgs.msg import UInt8, String
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord
from geometry_msgs.msg import PoseStamped, WrenchStamped, Vector3Stamped
from sensor_msgs.msg import JointState

class teleoperation():
    """Bridge joint angles from Spinal to Dragon for teleoperation."""
    def __init__(self):
        # Parameters to configure topics and joints
        self.robot_name = rospy.get_param("~robot_name", "dragon")

        # Control mode
        # 0: Joystick
        # 1: Position
        # 2: Velocity
        self.control_mode = rospy.get_param("~control_mode", 0)

        # For 1: position/ 2: velocity control modes
        self.pos_scale = rospy.get_param("~pos_scale", 1.0)
        self.vel_scale = rospy.get_param("~vel_scale", 0.3)
        self.ang_vel_scale = rospy.get_param("~ang_vel_scale", 0.08)

        self.spinal_topic = rospy.get_param("~spinal_joint_topic", "/joint_states")
        self.publish_topic = rospy.get_param("~publish_topic", "/dragon/joints_ctrl")

        # Servo ID list determines the expected number of joints
        servo_ids = rospy.get_param("~servo_ids", [1, 2, 3, 4, 5, 6, 7])

        # Publishers
        self.nav_pub = rospy.Publisher('/'+self.robot_name+'/uav/nav', FlightNav, queue_size=1)
        # self.att_pub = rospy.Publisher('/'+self.robot_name+'/final_target_baselink_rot', DesireCoord, queue_size=1)
        self.att_pub = rospy.Publisher('/'+self.robot_name+'/final_target_baselink_rpy', Vector3Stamped, queue_size=1)
        
        # Subscribers
        self.flight_state_sub = rospy.Subscriber('/'+self.robot_name+'/flight_state', UInt8, self.flight_state_cb)
        self.device_pos_sub = rospy.Subscriber('/dracomancer/mocap/pose', PoseStamped, self.device_pos_cb)
        self.robot_pos_sub = rospy.Subscriber('/'+self.robot_name+'/mocap/pose', PoseStamped, self.robot_pos_cb)
        self.teleop_mode_sub = rospy.Subscriber('/dracomancer/teleop_mode', String, self.teleop_mode_cb)
        
        self.flight_nav = FlightNav()
        self.flight_nav.target = FlightNav.COG
        self.flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
        self.flight_nav.yaw_nav_mode = FlightNav.POS_VEL_MODE
        self.flight_nav.pos_z_nav_mode = FlightNav.POS_VEL_MODE

        """ for new FlightNav """
        self.flight_nav.roll_nav_mode = FlightNav.POS_VEL_MODE
        self.flight_nav.pitch_nav_mode = FlightNav.POS_VEL_MODE
        self.desire_att_nav = DesireCoord()
        self.target_att_nav = Vector3Stamped()
        self.haptics_wrench_msg = WrenchStamped()

        # Optional joint names; fall back to generic names if lengths mismatch
        joint_names = rospy.get_param("~joint_names", None)
        if joint_names and len(joint_names) != len(servo_ids):
            rospy.logwarn("Length of ~joint_names does not match ~servo_ids; using default names.")
            joint_names = None
        self.joint_names = joint_names if joint_names else [
            "joint{}".format(i + 1) for i in range(len(servo_ids))
        ]

        # Optional angle offsets (radians); disabled if lengths mismatch
        offsets = rospy.get_param("~offsets", None)
        if offsets and len(offsets) != len(servo_ids):
            rospy.logwarn("Length of ~offsets does not match ~servo_ids; disabling offsets.")
            offsets = None
        self.offsets = offsets if offsets else [0.0] * len(servo_ids)

        # Storage for latest angles received from Spinal
        self.current_angles = None

        # Subscribe to Spinal's JointState topic
        rospy.Subscriber(self.spinal_topic, JointState, self.spinal_callback)

        # Publisher to Dragon's joint control topic
        self.joint_pub = rospy.Publisher(self.publish_topic, JointState, queue_size=10)

        self.rate = rospy.Rate(rospy.get_param("~publish_rate", 10.0))

    def spinal_callback(self, msg):
        """
        Store the latest joint angles from Spinal. The 'position' field is
        assumed to contain angles in radians.
        """
        self.current_angles = list(msg.position)

    def main(self):
        """Publish joint commands to Dragon at the configured rate."""
        while not rospy.is_shutdown():
            if self.current_angles is None:
                # Wait until Spinal provides joint angles
                self.rate.sleep()
                continue

            # Ensure we have enough angles; pad with zeros if necessary
            angles = self.current_angles
            if len(angles) < len(self.offsets):
                angles = angles + [0.0] * (len(self.offsets) - len(angles))

            # Apply offsets
            send_angles = [a + off for a, off in zip(angles, self.offsets)]

            # Build and publish JointState message
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = self.joint_names
            msg.position = send_angles
            self.joint_pub.publish(msg)

            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("dracomancer_teleoperation")
    Tracker = teleoperation()
    Tracker.main()
