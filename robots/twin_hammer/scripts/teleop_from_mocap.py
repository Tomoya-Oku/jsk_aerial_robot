#!/usr/bin/env python

import rospy
import time
import tf.transformations as tf
from std_msgs.msg import UInt8, String
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord
from geometry_msgs.msg import PoseStamped, WrenchStamped
from geometry_msgs.msg import Vector3Stamped

class teleop_from_mocap():

  def __init__(self):

    self.robot_name = rospy.get_param("~robot_name", "quadrotor")
    self.control_mode = rospy.get_param("~control_mode", "pos") # "pos" or "vel"
    self.pos_scale = rospy.get_param("~pos_scale", 1.0)
    self.vel_scale = rospy.get_param("~vel_scale", 0.2)
    self.ang_vel_scale = rospy.get_param("~ang_vel_scale", 0.08)
    self.feedback_force_scale = rospy.get_param("~feedback_force_scale", 10.0)
    self.feedback_torque_scale = rospy.get_param("~feedback_torque_scale", 1.0)

    self.nav_pub = rospy.Publisher('/'+self.robot_name+'/uav/nav', FlightNav, queue_size=1)
    # self.att_pub = rospy.Publisher('/'+self.robot_name+'/final_target_baselink_rot', DesireCoord, queue_size=1)
    self.att_pub = rospy.Publisher('/'+self.robot_name+'/final_target_baselink_rpy', Vector3Stamped, queue_size=1)
    self.feedback_pub = rospy.Publisher('/twin_hammer/haptics_wrench', WrenchStamped, queue_size=1)
    self.flight_state_sub = rospy.Subscriber('/'+self.robot_name+'/flight_state', UInt8, self.flight_state_cb)
    self.device_pos_sub = rospy.Subscriber('/twin_hammer/mocap/pose', PoseStamped, self.device_pos_cb)
    self.robot_pos_sub = rospy.Subscriber('/'+self.robot_name+'/mocap/pose', PoseStamped, self.robot_pos_cb)
    self.teleop_mode_sub = rospy.Subscriber('/twin_hammer/teleop_mode', String, self.teleop_mode_cb)
    self.flight_nav = FlightNav()
    self.flight_nav.target = FlightNav.COG
    self.flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.yaw_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.pos_z_nav_mode = FlightNav.POS_VEL_MODE
    # for new FlightNav
    self.flight_nav.roll_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.pitch_nav_mode = FlightNav.POS_VEL_MODE
    self.desire_att_nav = DesireCoord()
    self.target_att_nav = Vector3Stamped()
    self.haptics_wrench_msg = WrenchStamped()

    self.hovering = False
    self.landing = False
    self.device_pos = None
    self.device_att = None
    self.robot_pos = None
    self.robot_att = None
    self.device_init_pos = None
    self.device_init_att = None
    self.robot_init_pos = None
    self.robot_init_att = None
    self.initialize_flag = False
    self.wait_flag = True

  def flight_state_cb(self,msg):
    if msg.data == 5:
      self.hovering = True
    if msg.data == 4:
      self.landing = True

  def device_pos_cb(self,msg):
    self.device_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    device_orientation_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    self.device_att = tf.euler_from_quaternion(device_orientation_q)
    if self.device_init_pos == None and self.hovering:
      self.device_init_pos = self.device_pos
      self.device_init_att = self.device_att
    if self.initialize_flag == False:
      self.device_init_pos = self.device_pos
      self.device_init_att = self.device_att
      self.initialize_flag = True

  def robot_pos_cb(self,msg):
    self.robot_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    robot_orientation_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    self.robot_att = tf.euler_from_quaternion(robot_orientation_q)
    if self.robot_init_pos == None and self.hovering:
      self.robot_init_pos = self.robot_pos
      self.robot_init_att = self.robot_att
    if self.initialize_flag == False:
      self.robot_init_pos = self.robot_pos
      self.robot_init_att = self.robot_att
      self.initialize_flag = True

  def teleop_mode_cb(self,msg):
    self.initialize_flag = False
    self.wait_flag = True
    if msg.data == "pos":
      self.control_mode = "pos"
    if msg.data == "vel":
      self.control_mode = "vel"   

  def main(self):
    target_pos = [0.0,0.0,0.0]
    target_att = [0.0,0.0,0.0]
    target_vel = [0.0,0.0,0.0]
    target_ang_vel = [0.0,0.0,0.0]
    feedback_wrench = [0.0,0.0,0.0,0.0,0.0,0.0]

    r = rospy.Rate(40)
    while not rospy.is_shutdown():

      if self.hovering and self.device_pos!=None and self.device_init_pos!=None and self.robot_init_pos!=None:
        for i in range(3):
          target_pos[i] = (self.device_pos[i] - self.device_init_pos[i] + self.robot_init_pos[i]) * self.pos_scale
          target_att[i] = self.device_att[i]
          # target_vel[i] = (self.device_pos[i] - self.device_init_pos[i]) * self.vel_scale
          target_vel[i] = self.robot_pos[i] + (self.device_pos[i] - self.device_init_pos[i]) * self.vel_scale
          # target_ang_vel[i] = (self.device_att[i] - self.device_init_att[i]) * self.ang_vel_scale
          target_ang_vel[i] = self.robot_att[i] + (self.device_att[i] - self.device_init_att[i]) * self.ang_vel_scale
          feedback_wrench[i] = - (self.device_pos[i] - self.device_init_pos[i]) * self.feedback_force_scale
          feedback_wrench[i+3] = (self.device_att[i] - self.device_init_att[i]) * self.feedback_torque_scale
          # feedback_wrench[i+3] = - self.device_att[i]
        
        """ limitation of z and att for safety """
        if self.robot_pos[2] > 1.0:
          target_pos[2] = 1.0
          target_vel[2] = 0.0
        if self.robot_pos[2] < 0.3:
          target_pos[2] = 0.3
          target_vel[2] = 0.0
        limit_angle = 0.2
        for i in range(2):  # except yaw
          if target_att[i] > limit_angle:
            target_att[i] = limit_angle
          if target_att[i] < -limit_angle:
            target_att[i] = -limit_angle
          if target_ang_vel[i] > limit_angle:
            target_ang_vel[i] = limit_angle
          if target_ang_vel[i] < -limit_angle:
            target_ang_vel[i] = -limit_angle

        if self.control_mode == "pos":
          self.flight_nav.target_pos_x = target_pos[0]
          self.flight_nav.target_pos_y = target_pos[1]
          self.flight_nav.target_pos_z = target_pos[2]
          self.flight_nav.target_yaw = target_att[2]
          # for new FlightNav
          self.flight_nav.target_roll = target_att[0]
          self.flight_nav.target_pitch = target_att[1]
          self.desire_att_nav.roll = target_att[0]
          self.desire_att_nav.pitch = target_att[1]
          self.target_att_nav.vector.x = target_att[0]
          self.target_att_nav.vector.y = target_att[1]
        if self.control_mode == "vel":
          self.flight_nav.target_pos_x = target_vel[0]
          self.flight_nav.target_pos_y = target_vel[1]
          # self.flight_nav.target_pos_z = target_vel[2]
          self.flight_nav.target_pos_z = target_pos[2]
          self.flight_nav.target_yaw = target_ang_vel[2]
          # self.flight_nav.target_yaw = target_att[2]
          # for new FlightNav
          self.flight_nav.target_roll = target_att[0]
          self.flight_nav.target_pitch = target_att[1]
          self.desire_att_nav.roll = target_att[0]
          self.desire_att_nav.pitch = target_att[1]
          self.target_att_nav.vector.x = target_att[0]
          self.target_att_nav.vector.y = target_att[1]
          
          self.haptics_wrench_msg.wrench.force.x = feedback_wrench[0]
          self.haptics_wrench_msg.wrench.force.y = feedback_wrench[1]
          self.haptics_wrench_msg.wrench.force.z = feedback_wrench[2]
          self.haptics_wrench_msg.wrench.torque.x = feedback_wrench[3]
          self.haptics_wrench_msg.wrench.torque.y = feedback_wrench[4]
          self.haptics_wrench_msg.wrench.torque.z = feedback_wrench[5]

      if self.hovering and not self.landing:
        if self.wait_flag:
          rospy.sleep(3.0)
          self.wait_flag = False
        self.nav_pub.publish(self.flight_nav)
        # self.att_pub.publish(self.desire_att_nav)
        self.att_pub.publish(self.target_att_nav)
        if self.control_mode == "vel":
          self.feedback_pub.publish(self.haptics_wrench_msg)

      r.sleep()

if __name__ == "__main__":
  rospy.init_node("teleop_form_mocap")
  Tracker = teleop_from_mocap()
  Tracker.main()
