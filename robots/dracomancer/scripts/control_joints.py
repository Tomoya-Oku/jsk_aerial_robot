#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from spinal.msg import ServoStates


class ServoStateToDragonJointsCtrl:
    def __init__(self):
        rospy.init_node("servo_state_to_dragon_joints_ctrl")

        self.servo_topic = rospy.get_param("~servo_topic", "dracomancer/servo/states")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/dragon/joints_ctrl")
        self.rate_hz = rospy.get_param("~rate", 30.0)

        # servo ID 0..5 -> DRAGON joint index
        self.id_to_joint_index = rospy.get_param("~id_to_joint_index", [0, 1, 2, 3, 4, 5])

        # 符号・スケール
        self.signs = rospy.get_param("~signs", [1, 1, 1, 1, 1, 1])
        self.angle_scale = rospy.get_param("~angle_scale", 0.01)

        # DRAGON初期姿勢
        # 参考スクリプトの reset 姿勢
        self.dragon_init_pose = rospy.get_param(
            "~dragon_init_pose",
            [0.0, 1.56, 0.0, 1.56, 0.0, 1.56]
        )

        # サーボ起動時の角度を初期値として使う
        self.capture_initial_on_first_msg = rospy.get_param("~capture_initial_on_first_msg", True)

        self.joint_limit = rospy.get_param("~joint_limit", 1.56)

        self.latest_servo_pos = {}
        self.initial_servo_pos = {}
        self.initialized = False

        self.pub = rospy.Publisher(self.cmd_topic, JointState, queue_size=10)
        self.sub = rospy.Subscriber(self.servo_topic, ServoStates, self.servo_cb, queue_size=10)

        rospy.loginfo("servo_topic: %s", self.servo_topic)
        rospy.loginfo("cmd_topic: %s", self.cmd_topic)
        rospy.loginfo("dragon_init_pose: %s", self.dragon_init_pose)

    def clamp(self, x):
        return max(-self.joint_limit, min(self.joint_limit, x))

    def servo_cb(self, msg):
        current = {}

        for s in msg.servos:
            sid = int(s.index)
            if 0 <= sid <= 5:
                current[sid] = float(s.angle)

        self.latest_servo_pos = current

        if self.capture_initial_on_first_msg and not self.initialized:
            # 最初に取得できた角度を基準値にする
            for sid in range(6):
                if sid in current:
                    self.initial_servo_pos[sid] = current[sid]
            if len(self.initial_servo_pos) == 6:
                self.initialized = True
                rospy.loginfo("Captured initial servo angles: %s", self.initial_servo_pos)

    def make_joint_msg(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()

        target = list(self.dragon_init_pose)

        if not self.initialized:
            msg.position = target
            return msg

        for servo_id in range(6):
            if servo_id not in self.latest_servo_pos:
                continue
            if servo_id not in self.initial_servo_pos:
                continue

            joint_index = self.id_to_joint_index[servo_id]
            if not (0 <= joint_index < len(target)):
                continue

            delta = self.latest_servo_pos[servo_id] - self.initial_servo_pos[servo_id]
            delta = self.signs[servo_id] * delta * self.angle_scale

            target[joint_index] = self.clamp(self.dragon_init_pose[joint_index] + delta)

        msg.position = target
        return msg

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.pub.publish(self.make_joint_msg())
            rate.sleep()


if __name__ == "__main__":
    try:
        node = ServoStateToDragonJointsCtrl()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))