#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool, Float32MultiArray, UInt8
from geometry_msgs.msg import Vector3Stamped


class AttitudeControl:
    def __init__(self):
        rospy.init_node("control_orientation")

        self.robot_name = rospy.get_param("~robot_name", "dragon")
        self.device_ns = rospy.get_param("~device_ns", "/dracomancer").rstrip("/")
        self.joy_topic = rospy.get_param("~joy_topic", self.device_ns + "/joystick/calibrated")
        self.output_topic = rospy.get_param("~output_topic", "/" + self.robot_name + "/final_target_baselink_rpy")
        self.mode_topic = rospy.get_param("~mode_topic", self.device_ns + "/teleop_mode")
        self.teleop_mode = self.parse_bool_param(rospy.get_param("~teleop_mode", False), False)
        self.rate_hz = rospy.get_param("~rate", 40.0)

        self.axis_roll = rospy.get_param("~axis_roll", -1)
        self.axis_pitch = rospy.get_param("~axis_pitch", -1)
        self.axis_yaw = rospy.get_param("~axis_yaw", -1)
        self.scale_roll = rospy.get_param("~scale_roll", 0.0)
        self.scale_pitch = rospy.get_param("~scale_pitch", 0.0)
        self.scale_yaw = rospy.get_param("~scale_yaw", 0.0)
        self.deadzone = rospy.get_param("~deadzone", 0.05)
        self.publish_only_when_hovering = rospy.get_param("~publish_only_when_hovering", True)

        self.latest_axes = None
        self.robot_hovering = False
        self.active_prev = False

        self.att_pub = rospy.Publisher(self.output_topic, Vector3Stamped, queue_size=1)
        rospy.Subscriber(self.joy_topic, Float32MultiArray, self.joystick_cb, queue_size=1)
        rospy.Subscriber(self.mode_topic, Bool, self.mode_cb, queue_size=1)
        rospy.Subscriber("/" + self.robot_name + "/flight_state", UInt8, self.robot_flight_state_cb, queue_size=1)

        rospy.loginfo("joy_topic: %s", self.joy_topic)
        rospy.loginfo("attitude output_topic: %s", self.output_topic)
        rospy.loginfo("attitude axes roll/pitch/yaw: %d/%d/%d",
                      self.axis_roll, self.axis_pitch, self.axis_yaw)

    def joystick_cb(self, msg):
        self.latest_axes = list(msg.data)

    @staticmethod
    def parse_bool_param(value, default=False):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        text = str(value).strip().lower()
        if text == "true":
            return True
        if text == "false":
            return False
        rospy.logwarn("unknown teleop_mode bool '%s', use %s", value, default)
        return default

    def mode_cb(self, msg):
        self.teleop_mode = bool(msg.data)
        self.latest_axes = None

    def robot_flight_state_cb(self, msg):
        self.robot_hovering = int(msg.data) >= 4

    def axis_value(self, axes, index, scale):
        if index < 0 or index >= len(axes):
            return 0.0
        value = float(axes[index])
        if abs(value) < self.deadzone:
            return 0.0
        return scale * value

    def make_attitude_msg(self):
        axes = self.latest_axes or []
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.vector.x = self.axis_value(axes, self.axis_roll, self.scale_roll)
        msg.vector.y = self.axis_value(axes, self.axis_pitch, self.scale_pitch)
        msg.vector.z = self.axis_value(axes, self.axis_yaw, self.scale_yaw)
        return msg

    def publish_zero(self):
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        self.att_pub.publish(msg)

    def main(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            active = self.teleop_mode
            if self.publish_only_when_hovering:
                active = active and self.robot_hovering

            if active:
                self.att_pub.publish(self.make_attitude_msg())
            elif self.active_prev:
                self.publish_zero()

            self.active_prev = active
            rate.sleep()


if __name__ == "__main__":
    try:
        node = AttitudeControl()
        node.main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))
