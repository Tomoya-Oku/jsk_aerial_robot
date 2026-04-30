#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from spinal.msg import ServoStates


class ControlJoints:
    def __init__(self):
        rospy.init_node("control_joints")

        # Parameters
        self.robot_name = rospy.get_param("~robot_name", "dragon")
        self.frame = rospy.get_param("~frame", "local")
        self.rate_hz = rospy.get_param("~rate", 40.0)

        # DRAGONの尾頭のどちらを手首側にするか
        self.reverse_head = rospy.get_param("~reverse_head", False)

        # 関節角制限
        self.device_joint_limit = rospy.get_param("~joint_limit", np.pi / 2)

        # サーボ起動時の角度を初期値として使う
        self.capture_initial_on_first_msg = rospy.get_param(
            "~capture_initial_on_first_msg",
            False
        )

        # servo角度からjoint角度への変換係数
        self.angle_scale = rospy.get_param("~angle_scale", 0.01)

        # --------------------------------------------------
        # servo ID -> DRAGON joint index
        #
        # target[0]: 第1関節 pitch
        # target[1]: 第1関節 yaw
        # target[2]: 第2関節 pitch
        # target[3]: 第2関節 yaw
        # target[4]: 第3関節 pitch
        # target[5]: 第3関節 yaw
        # --------------------------------------------------
        self.servo_to_joint_index = {
            1: 0,
            2: 1,
            4: 2,
            6: 4,
            7: 5,
        }

        # 符号
        self.signs = {
            1: 1.0,
            2: 1.0,
            4: 1.0,
            6: 1.0,
            7: 1.0,
        }

        # DRAGON初期姿勢
        self.dragon_init_pose = rospy.get_param(
            "~dragon_init_pose",
            [0.0, 1.56, 0.0, 1.56, 0.0, 1.56]
        )

        self.latest_servo_pos = {}

        if self.capture_initial_on_first_msg:
            self.initial_servo_pos = {}
            self.initialized = False
        else:
            # captureしない場合は0基準
            self.initial_servo_pos = {
                sid: 0.0 for sid in self.servo_to_joint_index.keys()
            }
            self.initialized = True

        # Publisher
        self.joints_ctrl_pub = rospy.Publisher(
            "/dragon/joints_ctrl",
            JointState,
            queue_size=10
        )

        # Subscriber
        self.servo_states_sub = rospy.Subscriber(
            "/servo/states",
            ServoStates,
            self.servo_cb,
            queue_size=10
        )

        rospy.loginfo("servo_to_joint_index: %s", self.servo_to_joint_index)
        rospy.loginfo("capture_initial_on_first_msg: %s", self.capture_initial_on_first_msg)

    def clamp(self, x):
        return max(-self.device_joint_limit, min(self.device_joint_limit, x))

    def servo_cb(self, msg):
        current = {}

        for s in msg.servos:
            sid = int(s.index)

            # 必要なservo IDだけ読む
            if sid in self.servo_to_joint_index:
                current[sid] = float(s.angle)

        self.latest_servo_pos = current

        if self.capture_initial_on_first_msg and not self.initialized:
            for sid in self.servo_to_joint_index.keys():
                if sid in current:
                    self.initial_servo_pos[sid] = current[sid]

            if len(self.initial_servo_pos) == len(self.servo_to_joint_index):
                self.initialized = True
                rospy.loginfo(
                    "Captured initial servo angles: %s",
                    self.initial_servo_pos
                )

    def make_joint_msg(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()

        target = list(self.dragon_init_pose)

        if not self.initialized:
            msg.position = target
            return msg

        for servo_id, joint_index in self.servo_to_joint_index.items():
            if servo_id not in self.latest_servo_pos:
                continue

            if servo_id not in self.initial_servo_pos:
                continue

            delta = self.latest_servo_pos[servo_id] - self.initial_servo_pos[servo_id]
            delta = self.signs[servo_id] * delta * self.angle_scale

            target[joint_index] = self.clamp(
                self.dragon_init_pose[joint_index] + delta
            )

        msg.position = target
        return msg

    def main(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            self.joints_ctrl_pub.publish(self.make_joint_msg())
            rate.sleep()


if __name__ == "__main__":
    try:
        node = ControlJoints()
        node.main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))