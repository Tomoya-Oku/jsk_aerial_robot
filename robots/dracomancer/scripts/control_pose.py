#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rosgraph
from std_msgs.msg import Float32MultiArray, UInt8
from geometry_msgs.msg import PoseStamped, WrenchStamped, Vector3Stamped
from aerial_robot_msgs.msg import FlightNav


class ControlPose:
    def __init__(self):
        rospy.init_node("control_pose")

        self.robot_name = rospy.get_param("~robot_name", "dragon")
        self.frame = rospy.get_param("~frame", "local") 
        self.rate_hz = rospy.get_param("~rate", 40.0)

        # Msgs
        self.flight_nav = FlightNav()
        self.flight_nav.control_frame = FlightNav.WORLD_FRAME
        self.flight_nav.target = FlightNav.COG
        self.flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
        self.flight_nav.pos_z_nav_mode = FlightNav.POS_VEL_MODE
        self.flight_nav.yaw_nav_mode = FlightNav.POS_VEL_MODE
        self.flight_nav.roll_nav_mode = FlightNav.POS_VEL_MODE
        self.flight_nav.pitch_nav_mode = FlightNav.POS_VEL_MODE
        self.target_att_nav = Vector3Stamped()

        # Robot's Pose Limitation
        self.pos_limit = rospy.get_param("~pos_limit", True)
        self.att_limit = rospy.get_param("~att_limit", False)
        ## Position in Room-0066
        self.LIMIT_X = [-1.3, 2.0]
        self.LIMIT_Y = [-1.6, 1.6]
        self.LIMIT_Z = [0.3, 1.2]
        ## Attitude (not used for now)
        # self.LIMIT_R = [-0.5, 0.5]  # roll
        # self.LIMIT_P = [-0.5, 0.5]  # pitch
        # self.LIMIT_Y = [-1.0, 1.0]  # yaw

        # Joystick
        ## Scaling for Joystick (negative means inverted)
        self.scale_x = rospy.get_param("~scale_x", 1.0)
        self.scale_y = rospy.get_param("~scale_y", 1.0)
        self.scale_z = rospy.get_param("~scale_z", 1.0)
        ## Deadzone for Joystick
        self.deadzone = rospy.get_param("~deadzone", 0.05)
        ## Buffer for latest joystick axes
        self.latest_axes = None

        # Publishers
        self.nav_pub = rospy.Publisher('/'+self.robot_name + "/uav/nav", FlightNav, queue_size=1)
        self.att_pub = rospy.Publisher('/'+self.robot_name+'/final_target_baselink_rpy', Vector3Stamped, queue_size=1)

        # Subscribers
        self.robot_flight_state_sub = rospy.Subscriber('/'+self.robot_name+'/flight_state', UInt8, self.robot_flight_state_cb)
        self.robot_pos_sub = rospy.Subscriber('/'+self.robot_name+'/mocap/pose', PoseStamped, self.robot_pos_cb)
        self.js_calibrated_sub = rospy.Subscriber('/dracomancer/joystick/calibrated', Float32MultiArray, self.joystick_calib_cb, queue_size=1)

        # Logger
        rospy.loginfo("robot_name: %s", self.robot_name)

    def get_axis(self, data, idx):
        if idx < 0 or idx >= len(data):
            return 0.0
        v = float(data[idx])
        if idx == 0:
            v *= self.scale_x
        elif idx == 1:
            v *= self.scale_y
        elif idx == 2:
            v *= self.scale_z
        return 0.0 if abs(v) < self.deadzone else v

    def joystick_calib_cb(self, msg):
        self.latest_axes = list(msg.data)

    def make_nav_msg(self):
        if self.latest_axes is None:
            self.flight_nav.target_vel_x = 0.0
            self.flight_nav.target_vel_y = 0.0
            self.flight_nav.target_vel_z = 0.0
            return self.flight_nav

        x_cmd = self.get_axis(self.latest_axes, self.axis_x)
        y_cmd = self.get_axis(self.latest_axes, self.axis_y)
        z_cmd = self.get_axis(self.latest_axes, self.axis_z)

        self.flight_nav.target_vel_x = x_cmd * self.xy_vel
        self.flight_nav.target_vel_y = y_cmd * self.xy_vel
        self.flight_nav.target_vel_z = z_cmd * self.z_vel

    def limit_pose(self, pos, att):
        # Limit position
        if self.pos_limit:
            pos[0] = max(self.LIMIT_X[0], min(pos[0], self.LIMIT_X[1]))
            pos[1] = max(self.LIMIT_Y[0], min(pos[1], self.LIMIT_Y[1]))
            pos[2] = max(self.LIMIT_Z[0], min(pos[2], self.LIMIT_Z[1]))

        # Limit attitude
        if self.att_limit:
            att[0] = max(self.LIMIT_R[0], min(att[0], self.LIMIT_R[1]))
            att[1] = max(self.LIMIT_P[0], min(att[1], self.LIMIT_P[1]))
            att[2] = max(self.LIMIT_Y[0], min(att[2], self.LIMIT_Y[1]))

        return pos, att

    def main(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            target_pos = [0.0]*3
            target_att = [0.0]*3

            if self.device_init_pos is None or not self.robot_hovering:
                self.device_initialize_flag = False
            if self.robot_init_pos is None or not self.robot_hovering:
                self.robot_initialize_flag = False

            if self.device_initialize_flag and self.robot_initialize_flag:
                # limitation of position and attitude
                target_pos, target_att = self.limit_pose(target_pos, target_att)

            if self.robot_hovering and not self.robot_landing:
                if not self.wait_flag:
                    rospy.sleep(3.0)
                    self.wait_flag = True
                
                self.make_nav_msg()
                self.nav_pub.publish(self.flight_nav)

            rate.sleep()

if __name__ == "__main__":
    try:
        node = ControlPose()
        node.main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))