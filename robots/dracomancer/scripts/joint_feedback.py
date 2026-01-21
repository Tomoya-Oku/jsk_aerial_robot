#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Joint feedback node to enforce safe joint angles on the Dracomancer exoskeleton.

This node subscribes to the Dragon's joint states and uses a simple limit
enforcement algorithm to prevent the operator from driving the exoskeleton into
postures that the Dragon cannot achieve.  Instead of reading the current
exoskeleton angles directly from the hardware, the node reads them from Spinal's
JointState topic (e.g. '/joint_states'), which is how the Twin Hammer robot
monitors its gimbal angles:contentReference[oaicite:3]{index=3}:contentReference[oaicite:4]{index=4}.
If a Dragon joint approaches its limit and the operator is moving further in that
direction, the corresponding Dracomancer joint is commanded to hold its current
angle to prevent unsafe motion. Comments are written in English.
"""

import rospy
from sensor_msgs.msg import JointState

try:
    # Import DracomancerInterface from the package if installed
    from dracomancer.dracomancer_interface import DracomancerInterface
except ImportError:
    # Fallback to local file during development
    from dracomancer_interface import DracomancerInterface

class JointLimitEnforcer(object):
    """
    Utility class to clamp joint angles when the Dragon robot reaches its limits.

    Parameters:
        joint_limits: List of [min, max] pairs (radians) for each joint.
        margin: Safety margin near the limit within which to start clamping.
    """
    def __init__(self, joint_limits=None, margin=0.05):
        self.joint_limits = joint_limits
        self.margin = margin

    def enforce(self, current_angles, dragon_angles):
        """
        Given current exoskeleton angles and the Dragon's angles, return a list of
        angles that should be commanded to the exoskeleton.  If a Dragon joint is
        within margin of its limit and moving further toward the limit, hold the
        exoskeleton at its current angle.
        """
        if not self.joint_limits:
            return current_angles

        result = []
        for idx, (cur, drg) in enumerate(zip(current_angles, dragon_angles)):
            if idx >= len(self.joint_limits) or not self.joint_limits[idx]:
                result.append(cur)
                continue
            min_lim, max_lim = self.joint_limits[idx]
            # Clamp when approaching lower limit and moving smaller
            if min_lim is not None and drg <= min_lim + self.margin and cur < drg:
                result.append(cur)
            # Clamp when approaching upper limit and moving larger
            elif max_lim is not None and drg >= max_lim - self.margin and cur > drg:
                result.append(cur)
            else:
                result.append(cur)
        return result

class JointFeedbackNode(object):
    """Node that applies joint limit enforcement and commands Dracomancer servos."""
    def __init__(self):
        rospy.init_node("dracomancer_joint_feedback")

        # Parameters for hardware interface
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baudrate = rospy.get_param("~baudrate", 57600)
        servo_ids = rospy.get_param("~servo_ids", [1, 2, 3, 4, 5, 6, 7])

        # Joint limits (list of [min, max] pairs); optional
        joint_limits = rospy.get_param("~joint_limits", None)
        margin = rospy.get_param("~margin", 0.05)
        if joint_limits and len(joint_limits) != len(servo_ids):
            rospy.logwarn("Length of ~joint_limits does not match ~servo_ids; disabling limits.")
            joint_limits = None

        # Initialize Dracomancer hardware interface for commanding servos
        self.interface = DracomancerInterface(port, baudrate, servo_ids)

        # Limit enforcer instance
        self.enforcer = JointLimitEnforcer(joint_limits, margin)

        # Storage for latest exoskeleton angles from Spinal
        self.current_angles = None

        # Subscribe to Spinal joint angles
        spinal_topic = rospy.get_param("~spinal_joint_topic", "/joint_states")
        rospy.Subscriber(spinal_topic, JointState, self.spinal_callback)

        # Subscribe to Dragon joint angles
        dragon_topic = rospy.get_param("~dragon_joint_topic", "/dragon/joint_states")
        rospy.Subscriber(dragon_topic, JointState, self.dragon_callback)

    def spinal_callback(self, msg):
        """Update current exoskeleton joint angles from Spinal."""
        self.current_angles = list(msg.position)

    def dragon_callback(self, msg):
        """
        Apply limit enforcement and command the exoskeleton whenever Dragon
        publishes new joint angles.
        """
        if not msg.position or self.current_angles is None:
            return

        # Ensure we have as many current angles as Dragon joints
        current = self.current_angles
        if len(current) < len(msg.position):
            current = current + [0.0] * (len(msg.position) - len(current))

        commanded = self.enforcer.enforce(current, msg.position)
        try:
            # Send the commanded angles to the servo hardware
            self.interface.set_joint_angles(commanded)
        except RuntimeError as e:
            rospy.logerr("Failed to set servo angles: {}".format(e))

    def spin(self):
        """Spin until shutdown; callbacks handle the work."""
        rospy.spin()

def main():
    """Entry point for standalone execution."""
    try:
        node = JointFeedbackNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
