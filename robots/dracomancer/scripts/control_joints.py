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
        # servo ID -> DRAGON gimbal index
        #
        # target[0]: gimbal1_roll
        # target[1]: gimbal1_pitch
        # target[2]: gimbal2_roll
        # target[3]: gimbal2_pitch
        # target[4]: gimbal3_roll
        # target[5]: gimbal3_pitch
        # --------------------------------------------------
        self.servo_to_gimbal_index = {
            1: 0,
            2: 1,
            4: 2,
            6: 3,
            7: 4,
        }
        
        self.gimbal_names = [
            "gimbal1_roll",
            "gimbal1_pitch",
            "gimbal2_roll",
            "gimbal2_pitch",
            "gimbal3_roll",
            "gimbal3_pitch"
        ]

        # 符号
        self.signs = {
            1: 1.0,
            2: 1.0,
            4: 1.0,
            6: 1.0,
            7: 1.0,
        }

        # ギンバル初期姿勢
        self.gimbal_init_pose = rospy.get_param(
            "~gimbal_init_pose",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )

        self.latest_servo_pos = {}

        if self.capture_initial_on_first_msg:
            self.initial_servo_pos = {}
            self.initialized = False
        else:
            # captureしない場合は0基準
            self.initial_servo_pos = {
                sid: 0.0 for sid in self.servo_to_gimbal_index.keys()
            }
            self.initialized = True

        # Publisher
        self.gimbals_ctrl_pub = rospy.Publisher(
            "/dragon/gimbals_ctrl",
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

        rospy.loginfo("servo_to_gimbal_index: %s", self.servo_to_gimbal_index)
        rospy.loginfo("capture_initial_on_first_msg: %s", self.capture_initial_on_first_msg)

    def clamp(self, x):
        return max(-self.device_joint_limit, min(self.device_joint_limit, x))

    def servo_cb(self, msg):
        current = {}

        for s in msg.servos:
            sid = int(s.index)

            # 必要なservo IDだけ読む
            if sid in self.servo_to_gimbal_index:
                current[sid] = float(s.angle)

        self.latest_servo_pos = current

        if self.capture_initial_on_first_msg and not self.initialized:
            for sid in self.servo_to_gimbal_index.keys():
                if sid in current:
                    self.initial_servo_pos[sid] = current[sid]

            if len(self.initial_servo_pos) == len(self.servo_to_gimbal_index):
                self.initialized = True
                rospy.loginfo(
                    "Captured initial servo angles: %s",
                    self.initial_servo_pos
                )

    def make_gimbal_msg(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.gimbal_names

        target = list(self.gimbal_init_pose)

        if not self.initialized:
            msg.position = target
            return msg

        for servo_id, gimbal_index in self.servo_to_gimbal_index.items():
            if servo_id not in self.latest_servo_pos:
                continue

            if servo_id not in self.initial_servo_pos:
                continue

            delta = self.latest_servo_pos[servo_id] - self.initial_servo_pos[servo_id]
            delta = self.signs[servo_id] * delta * self.angle_scale

            target[gimbal_index] = self.clamp(
                self.gimbal_init_pose[gimbal_index] + delta
            )

        msg.position = target
        return msg

    def main(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            self.gimbals_ctrl_pub.publish(self.make_gimbal_msg())
            rate.sleep()


if __name__ == "__main__":
    try:
        node = ControlJoints()
        node.main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))