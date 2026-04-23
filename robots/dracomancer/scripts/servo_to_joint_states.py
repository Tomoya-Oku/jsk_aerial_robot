#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from sensor_msgs.msg import JointState
from spinal.msg import ServoStates   # 環境に合わせて確認

JOINT_MAP = {
    1: "shoulder_abduction_adduction_joint",
    2: "shoulder_flexion_extension_joint",
    3: "upper_arm_external_internal_rotation_joint",
    4: "elbow_flexion_extension_joint",
    5: "wrist_supination_joint",
    6: "wrist_flexion_extension_joint",
    7: "wrist_abduction_adduction_joint",
}

# 方向が逆ならここを -1 にする
SIGN_MAP = {
    1: 1.0,
    2: 1.0,
    3: 1.0,
    4: 1.0,
    5: 1.0,
    6: 1.0,
    7: 1.0,
}

CENTER_TICK = 2048.0
TICKS_PER_REV = 4096.0

latest_pos = {i: 0.0 for i in JOINT_MAP.keys()}

def tick_to_rad(tick, servo_id):
    return SIGN_MAP[servo_id] * (tick - CENTER_TICK) * 2.0 * math.pi / TICKS_PER_REV

def cb(msg):
    global latest_pos

    # msg.servos / msg.states など、実際のフィールド名に合わせて修正
    for s in msg.servos:
        sid = int(s.index)      # ここも環境に応じて s.id の可能性あり
        raw = float(s.angle)    # ここも環境に応じて s.position / s.value の可能性あり

        if sid in JOINT_MAP:
            latest_pos[sid] = tick_to_rad(raw, sid)

def main():
    rospy.init_node("servo_states_to_joint_states")

    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    rospy.Subscriber("/servo/states", ServoStates, cb, queue_size=10)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = [JOINT_MAP[i] for i in range(1, 8)]
        js.position = [latest_pos[i] for i in range(1, 8)]
        pub.publish(js)
        rate.sleep()

if __name__ == "__main__":
    main()