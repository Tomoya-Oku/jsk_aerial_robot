#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState


class RvizJointCtrlWrapper(object):
    def __init__(self):
        rospy.init_node("rviz_joint_ctrl_wrapper")

        self.rate = rospy.get_param("~rate", 40.0)
        self.joint_ctrl_topic = rospy.get_param("~joint_ctrl_topic", "joints_ctrl")
        self.joint_states_topic = rospy.get_param("~joint_states_topic", "joint_states")

        self.joint_names = self._load_servo_names("joints")
        self.gimbal_names = self._load_servo_names("gimbals")
        self.joint_positions = self._load_initial_joint_positions()

        self.joint_states_pub = rospy.Publisher(self.joint_states_topic, JointState, queue_size=1)
        self.joint_ctrl_sub = rospy.Subscriber(self.joint_ctrl_topic, JointState, self.joint_ctrl_cb, queue_size=1)

        rospy.loginfo("rviz_joint_ctrl_wrapper: %s -> %s", self.joint_ctrl_topic, self.joint_states_topic)
        rospy.loginfo("rviz_joint_ctrl_wrapper: joints=%s, gimbals=%s", self.joint_names, self.gimbal_names)

    def _load_servo_names(self, group_name):
        group_param = rospy.get_param("servo_controller/{}".format(group_name), {})
        controllers = []
        for key, value in group_param.items():
            if not key.startswith("controller"):
                continue
            if not isinstance(value, dict) or "name" not in value:
                continue
            try:
                index = int(key[len("controller"):])
            except ValueError:
                continue
            controllers.append((index, value["name"]))
        return [name for _, name in sorted(controllers)]

    def _load_initial_joint_positions(self):
        zeros = rospy.get_param("zeros", {})
        return {name: float(zeros.get(name, 0.0)) for name in self.joint_names}

    def joint_ctrl_cb(self, msg):
        if len(msg.name) == 0:
            if len(msg.position) != len(self.joint_names):
                rospy.logerr_throttle(
                    1.0,
                    "rviz_joint_ctrl_wrapper: unnamed joints_ctrl has %d positions, expected %d",
                    len(msg.position),
                    len(self.joint_names),
                )
                return
            for name, position in zip(self.joint_names, msg.position):
                self.joint_positions[name] = position
            return

        if len(msg.name) != len(msg.position):
            rospy.logerr_throttle(
                1.0,
                "rviz_joint_ctrl_wrapper: joints_ctrl name/position size mismatch [%d vs %d]",
                len(msg.name),
                len(msg.position),
            )
            return

        for name, position in zip(msg.name, msg.position):
            if name in self.joint_positions:
                self.joint_positions[name] = position

    def make_joint_states(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names + self.gimbal_names
        msg.position = [self.joint_positions[name] for name in self.joint_names]
        msg.position.extend([0.0] * len(self.gimbal_names))
        return msg

    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.joint_states_pub.publish(self.make_joint_states())
            rate.sleep()


if __name__ == "__main__":
    try:
        RvizJointCtrlWrapper().spin()
    except rospy.ROSInterruptException:
        pass
