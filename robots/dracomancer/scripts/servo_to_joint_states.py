#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from sensor_msgs.msg import JointState
from spinal.msg import ServoStates


class ControlJoints:
    def __init__(self):
        rospy.init_node("control_joints")

        # Parameters
        self.rate_hz = rospy.get_param("~rate", 40.0)

        # Topic names
        self.servo_topic = rospy.get_param("~servo_topic", "/servo/states")
        self.joint_states_topic = rospy.get_param("~joint_states_topic", "/dracomancer/joint_states")

        # 初回受信値を基準にするか
        self.capture_initial_on_first_msg = rospy.get_param(
            "~capture_initial_on_first_msg", False
        )

        # サーボID -> Joint名
        # 0始まりではなく、質問文どおり ID: 1,2,...,7 を使う
        self.id_to_joint_name = rospy.get_param("~id_to_joint_name", {
            0: "shoulder_abduction_adduction_joint",
            1: "shoulder_flexion_extension_joint",
            2: "upper_arm_external_internal_rotation_joint",
            3: "elbow_flexion_extension_joint",
            4: "wrist_supination_joint",
            5: "wrist_flexion_extension_joint",
            6: "wrist_abduction_adduction_joint",
        })

        # 各関節の符号補正
        self.signs = rospy.get_param("~signs", {
            0: 1.0,
            1: 1.0,
            2: 1.0,
            3: 1.0,
            4: 1.0,
            5: 1.0,
            6: 1.0,
        })

        # 生値 -> rad 変換設定
        # Dynamixel系の 0~4096, 中央2048 を想定
        self.center_tick = rospy.get_param("~center_tick", 2048.0)
        self.ticks_per_rev = rospy.get_param("~ticks_per_rev", 4096.0)

        # 各関節のオフセット [rad]
        # capture_initial_on_first_msg=False のとき有効
        self.offsets = rospy.get_param("~offsets", {
            0: 0.0,
            1: 0.0,
            2: 0.0,
            3: 0.0,
            4: 0.0,
            5: 0.0,
            6: 0.0,
        })

        # Joint limit [rad]
        self.joint_limit = rospy.get_param("~joint_limit", math.pi / 2.0)

        # 内部状態
        self.latest_servo_pos = {}
        self.initial_servo_pos = {}
        self.initialized = False

        # joint_states の順番
        self.ordered_ids = [0, 1, 2, 3, 4, 5, 6]

        # Publisher
        self.joint_states_pub = rospy.Publisher(
            self.joint_states_topic, JointState, queue_size=10
        )

        # Subscriber
        self.servo_states_sub = rospy.Subscriber(
            self.servo_topic, ServoStates, self.servo_cb, queue_size=10
        )

        rospy.loginfo("servo_topic: %s", self.servo_topic)
        rospy.loginfo("joint_states_topic: %s", self.joint_states_topic)
        rospy.loginfo("capture_initial_on_first_msg: %s", self.capture_initial_on_first_msg)

    def clamp(self, x):
        return max(-self.joint_limit, min(self.joint_limit, x))

    def tick_to_rad_from_center(self, tick, servo_id):
        sign = float(self.signs.get(servo_id, 1.0))
        return sign * (float(tick) - self.center_tick) * 2.0 * math.pi / self.ticks_per_rev

    def tick_delta_to_rad(self, delta_tick, servo_id):
        sign = float(self.signs.get(servo_id, 1.0))
        return sign * float(delta_tick) * 2.0 * math.pi / self.ticks_per_rev

    def servo_cb(self, msg):
        current = {}

        # ここはあなたの環境の /servo/states に合わせている
        for s in msg.servos:
            sid = int(s.index)
            if sid in self.ordered_ids:
                current[sid] = float(s.angle)

        self.latest_servo_pos = current

        if self.capture_initial_on_first_msg and not self.initialized:
            for sid in self.ordered_ids:
                if sid in current:
                    self.initial_servo_pos[sid] = current[sid]

            if len(self.initial_servo_pos) == len(self.ordered_ids):
                self.initialized = True
                rospy.loginfo("Captured initial servo angles: %s", self.initial_servo_pos)

    def make_joint_msg(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [self.id_to_joint_name[sid] for sid in self.ordered_ids]

        positions = []

        for sid in self.ordered_ids:
            # 値未受信なら 0 扱い
            if sid not in self.latest_servo_pos:
                positions.append(0.0)
                continue

            raw = self.latest_servo_pos[sid]

            if self.capture_initial_on_first_msg:
                # 初回姿勢を0 rad基準にする
                if not self.initialized or sid not in self.initial_servo_pos:
                    pos = 0.0
                else:
                    delta_tick = raw - self.initial_servo_pos[sid]
                    pos = self.tick_delta_to_rad(delta_tick, sid)
            else:
                # 2048中心で絶対角として扱う
                pos = self.tick_to_rad_from_center(raw, sid)
                pos += float(self.offsets.get(sid, 0.0))

            pos = self.clamp(pos)
            positions.append(pos)

        msg.position = positions
        return msg

    def main(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.joint_states_pub.publish(self.make_joint_msg())
            rate.sleep()


if __name__ == "__main__":
    try:
        node = ControlJoints()
        node.main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))