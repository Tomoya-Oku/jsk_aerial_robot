#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rosgraph

from std_msgs.msg import Float32MultiArray
from aerial_robot_msgs.msg import FlightNav


class JoystickTranslationCommand:
    def __init__(self):
        rospy.init_node("joystick_translation_command")

        self.robot_ns = rospy.get_param("~robot_ns", "")
        self.joy_topic = rospy.get_param("~joy_topic", "dracomancer/joystick/calib")

        # 最大速度 [m/s]
        self.xy_vel = rospy.get_param("~xy_vel", 0.2)
        self.z_vel = rospy.get_param("~z_vel", 0.15)

        # 軸割り当て
        self.axis_x = rospy.get_param("~axis_x", 0)   # forward/back
        self.axis_y = rospy.get_param("~axis_y", 1)   # left/right
        self.axis_z = rospy.get_param("~axis_z", 2)   # up/down, なければ未使用

        # 符号反転
        self.invert_x = rospy.get_param("~invert_x", False)
        self.invert_y = rospy.get_param("~invert_y", False)
        self.invert_z = rospy.get_param("~invert_z", False)

        # デッドゾーン
        self.deadzone = rospy.get_param("~deadzone", 0.05)

        # publish 周波数
        self.publish_rate = rospy.get_param("~publish_rate", 30.0)

        self.latest_axes = None

        if not self.robot_ns:
            self.robot_ns = self.detect_robot_ns()

        if not self.robot_ns:
            raise RuntimeError("robot_ns could not be determined. Please set ~robot_ns.")

        self.nav_pub = rospy.Publisher(self.robot_ns + "/uav/nav", FlightNav, queue_size=1)
        self.joy_sub = rospy.Subscriber(self.joy_topic, Float32MultiArray, self.joy_cb, queue_size=1)

        rospy.loginfo("robot_ns: %s", self.robot_ns)
        rospy.loginfo("joy_topic: %s", self.joy_topic)
        rospy.loginfo("publishing to: %s/uav/nav", self.robot_ns)

    def detect_robot_ns(self):
        master = rosgraph.Master('/rostopic')
        try:
            _, subs, _ = master.getSystemState()
        except Exception as e:
            rospy.logwarn("Failed to communicate with ROS master: %s", str(e))
            return ""

        teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
        if len(teleop_topics) == 1:
            return teleop_topics[0].split('/teleop')[0]

        return ""

    def apply_deadzone(self, x):
        return 0.0 if abs(x) < self.deadzone else x

    def get_axis(self, data, idx, invert=False):
        if idx < 0 or idx >= len(data):
            return 0.0
        v = float(data[idx])
        if invert:
            v = -v
        return self.apply_deadzone(v)

    def joy_cb(self, msg):
        self.latest_axes = list(msg.data)

    def make_nav_msg(self):
        nav_msg = FlightNav()
        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG

        nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
        nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE

        if self.latest_axes is None:
            nav_msg.target_vel_x = 0.0
            nav_msg.target_vel_y = 0.0
            nav_msg.target_vel_z = 0.0
            return nav_msg

        x_cmd = self.get_axis(self.latest_axes, self.axis_x, self.invert_x)
        y_cmd = self.get_axis(self.latest_axes, self.axis_y, self.invert_y)
        z_cmd = self.get_axis(self.latest_axes, self.axis_z, self.invert_z)

        nav_msg.target_vel_x = x_cmd * self.xy_vel
        nav_msg.target_vel_y = y_cmd * self.xy_vel
        nav_msg.target_vel_z = z_cmd * self.z_vel

        return nav_msg

    def run(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            nav_msg = self.make_nav_msg()
            self.nav_pub.publish(nav_msg)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = JoystickTranslationCommand()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))