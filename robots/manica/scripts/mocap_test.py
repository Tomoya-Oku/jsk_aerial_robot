#!/usr/bin/env python

import rospy
import argparse
import numpy as np
from geometry_msgs.msg import PoseStamped, WrenchStamped, Vector3Stamped
from scipy.spatial.transform import Rotation as R
from math import dist

params_path = "../config/params.yaml"

# YAMLファイル読み込み (すべての大元)
def load_params(params_path):
    with open(params_path, 'r') as file:
        params = params.safe_load(file)
    return params

# 引数抽出
def parse_args():
    argv = rospy.myargv(argv=None)  # ROS固有引数を除去
    parser = argparse.ArgumentParser(description="Teleop glue node with app args + ROS params")
    parser.add_argument("--user", choices=["oku"], default="oku", help="Profile name to select user's body size")

    args = parser.parse_args(argv[1:])
    return args

class Mocap_Test():
    """
    モーキャプのテスト
    """
    def __init__(self, user: str):
        # Parameters
        self.user_name = user
        self.params = load_params()
        self.upperarm_length = self.params['user'][self.user_name]['upperarm_length']
        self.forearm_length = self.params['user'][self.user_name]['forearm_length']
        self.rate = self.params['rospy']['rate']

        # Publisher
        self.pub_shoulder_att = rospy.Publisher('/manica/calculation/shoulder_att', Vector3Stamped, queue_size=1)
        self.pub_shoulder_att_unwraped = rospy.Publisher('/manica/calculation/shoulder_att_unwraped', Vector3Stamped, queue_size=1)
        self.pub_shoulder_att_prev = rospy.Publisher('/manica/calculation/shoulder_att_prev', Vector3Stamped, queue_size=1)

        self.pub_wrist_att = rospy.Publisher('/manica/calculation/wrist_att', Vector3Stamped, queue_size=1)
        self.pub_wrist_att_unwraped = rospy.Publisher('/manica/calculation/wrist_att_unwraped', Vector3Stamped, queue_size=1)
        self.pub_wrist_att_prev = rospy.Publisher('/manica/calculation/wrist_att_prev', Vector3Stamped, queue_size=1)

        self.pub_shoulder_wrist_dist = rospy.Publisher('/manica/calculation/shoulder_wrist_dist', float, queue_size=1) # Distance between shoulder and wrist
        self.pub_elbow_angle = rospy.Publisher('/manica/calculation/elbow_angle', float, queue_size=1) # Angle of elbow calculated from positions of shoulder and wrist [deg]

        # Subscriber
        self.sub_shoulder_pose = rospy.Subscriber('/manica/mocap/shoulder/pose', PoseStamped, self.shoulder_pos_cb)
        self.sub_wrist_pose = rospy.Subscriber('/manica/mocap/wrist/pose', PoseStamped, self.wrist_pos_cb)

        # Variables
        self.shoulder_pos = [None]*3
        self.shoulder_att = [None]*3
        self.shoulder_att_unwrapped = [0.0]*3
        self.shoulder_att_prev = [0.0]*3
        
        self.wrist_pos = [None]*3
        self.wrist_att = [None]*3
        self.wrist_att_unwrapped = [0.0]*3
        self.wrist_att_prev = [0.0]*3

        self.dist_shoulder_wrist = 0.0
        self.elbow_angle = 0.0

    def shoulder_pos_cb(self, msg):
        self.shoulder_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        rot = R.from_quat(q)
        self.shoulder_att = rot.as_euler('xyz')
        for i in range(3):
            current_angle = self.shoulder_att[i]
            delta_angle = (current_angle - self.shoulder_att_prev[i] + np.pi) % (2 * np.pi) - np.pi
            self.shoulder_att_unwrapped[i] += delta_angle
            self.shoulder_att_prev[i] = current_angle

    def wrist_pos_cb(self, msg):
        self.wrist_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        rot = R.from_quat(q)
        self.wrist_att = rot.as_euler('xyz')
        for i in range(3):
            current_angle = self.wrist_att[i]
            delta_angle = (current_angle - self.wrist_att_prev[i] + np.pi) % (2 * np.pi) - np.pi
            self.wrist_att_unwrapped[i] += delta_angle
            self.wrist_att_prev[i] = current_angle

    # モーキャプで得られる手首と肩の距離を計算
    def calc_dist_shoulder_wrist(self):
        return dist(self.shoulder_pos, self.wrist_pos)

    # モーキャプで得られる手首と肩の位置から肘の角度を算出
    def calc_angle_from_wrist_and_shoulder_pos(self):
        if self.upperarm_length == 0.0 or self.forearm_length == 0.0:
            return None
        else:
            return np.arccos((self.upperarm_length**2 + self.forearm_length**2 - self.dist_shoulder_wrist**2) / (2 * self.upperarm_length * self.forearm_length))

    def main(self):
        r = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            # Calculation from Mocap
            self.dist_shoulder_wrist = self.calc_dist_shoulder_wrist()
            self.elbow_angle = self.calc_angle_from_wrist_and_shoulder_pos()

            # Publish
            self.pub_shoulder_att.publish(self.shoulder_att)
            self.pub_shoulder_att_unwraped.publish(self.shoulder_att_unwraped)
            self.pub_shoulder_att_prev.publish(self.shoulder_att_prev)

            self.pub_wrist_att.publish(self.wrist_att)
            self.pub_wrist_att_unwraped.publish(self.wrist_att_unwraped)
            self.pub_wrist_att_prev.publish(self.wrist_att_prev)

            self.pub_shoulder_wrist_dist.publish(self.shoulder_wrist_dist)
            self.pub_elbow_angle.publish(self.elbow_angle)

            r.sleep()

if __name__ == "__main__":
    rospy.init_node("mocap_test_node")

    args = parse_args()
    user_name = args.user

    Tracker = Mocap_Test(user=user_name)
    Tracker.main()