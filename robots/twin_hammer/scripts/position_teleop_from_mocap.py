#!/usr/bin/env python

import rospy
import time
import tf.transformations as tf
from std_msgs.msg import UInt8
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord
from geometry_msgs.msg import PoseStamped

class position_teleop():

  def __init__(self):

    self.real_machine = rospy.get_param("~real_machine", False)
    self.robot_name = rospy.get_param("~robot_name", "quadrotor")
    self.pos_scale = rospy.get_param("~pos_scale", 1.1)

    self.nav_pub = rospy.Publisher('/'+self.robot_name+'/uav/nav', FlightNav, queue_size=1)
    self.att_pub = rospy.Publisher('/'+self.robot_name+'/final_target_baselink_rot', DesireCoord, queue_size=1)
    self.flight_state_sub = rospy.Subscriber('/'+self.robot_name+'/flight_state', UInt8, self.flight_state_cb)
    self.device_pos_sub = rospy.Subscriber('/device/mocap/pose', PoseStamped, self.device_pos_cb)
    self.robot_pos_sub = rospy.Subscriber('/'+self.robot_name+'/mocap/pose', PoseStamped, self.robot_pos_cb)
    self.flight_nav = FlightNav()
    self.flight_nav.target = FlightNav.COG
    self.flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.pos_z_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.yaw_nav_mode = FlightNav.POS_VEL_MODE
    self.desire_att_nav = DesireCoord()

    self.hovering = False
    self.landing = False
    self.device_pos = None
    self.device_att = None
    self.robot_pos = None
    self.robot_att = None
    self.device_init_pos = None
    self.robot_init_pos = None

    time.sleep(0.5)

  def flight_state_cb(self,msg):
    if msg.data == 5:
      self.hovering = True
    if msg.data == 4:
      self.landing = True

  def device_pos_cb(self,msg):
    self.device_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    if self.device_init_pos == None and self.hovering:
      self.device_init_pos = self.device_pos
    device_orientation_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    self.device_att = tf.euler_from_quaternion(device_orientation_q)

  def robot_pos_cb(self,msg):
    self.robot_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    if self.robot_init_pos == None and self.hovering:
      self.robot_init_pos = self.robot_pos
    robot_orientation_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    self.robot_att = tf.euler_from_quaternion(robot_orientation_q)

  def main(self):
    r = rospy.Rate(40)
    while not rospy.is_shutdown():

      if self.hovering and self.device_pos!=None and self.device_init_pos!=None and self.robot_init_pos!=None:
        target_pos = [0.0,0.0,0.0]
        for i in range(3):
          target_pos[i] = (self.device_pos[i] - self.device_init_pos[i] + self.robot_init_pos[i]) * self.pos_scale
        self.flight_nav.target_pos_x = target_pos[0]
        self.flight_nav.target_pos_y = target_pos[1]
        self.flight_nav.target_pos_z = target_pos[2]
        self.flight_nav.target_yaw = self.device_att[2]
        self.desire_att_nav.roll = self.device_att[0]
        self.desire_att_nav.pitch = self.device_att[1]

      if self.hovering and not self.landing:
        self.nav_pub.publish(self.flight_nav)
        self.att_pub.publish(self.desire_att_nav)

      r.sleep()

if __name__ == "__main__":
  rospy.init_node("position_teleop_form_mocap")
  Tracker = position_teleop()
  Tracker.main()
