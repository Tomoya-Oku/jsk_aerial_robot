#!/usr/bin/env python

import rospy
import time
import math
import tf.transformations as tf
from std_msgs.msg import Int8
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord
from geometry_msgs.msg import PoseStamped, WrenchStamped

def exponential(x, base, k_exp):
  return pow(x,base) * k_exp

def logarithm(x, base, k_log):
  return math.log(x,base) * k_log

class force_feedback_from_robot():

  def __init__(self):

    self.real_machine = rospy.get_param("~real_machine", False)
    self.robot_name = rospy.get_param("~robot_name", "quadrotor")
    self.pos_scale = rospy.get_param("~pos_scale", 1.1)

    # self.haptics_switch_pub = rospy.Publisher('/twin_hammer/haptics_switch', Int8, queue_size=1)
    self.haptics_wrench_pub = rospy.Publisher('/twin_hammer/haptics_wrench', WrenchStamped, queue_size=1)
    self.robot_wrench_sub = rospy.Subscriber('/filtered_ft_sensor', WrenchStamped, self.robot_wrench_cb)

    self.haptics_wrench_msg = WrenchStamped()
    self.robot_wrench = []

    self.k_p = 1.0
    self.exp_base = 1.3
    self.log_base = 1.3
    self.k_exp = 1.0
    self.k_log = 1.1
    self.range_log = math.e
    self.a_log = self.k_log / (math.e * math.log(self.log_base))
    # time.sleep(0.5)

  def robot_wrench_cb(self,msg):
    self.robot_wrench = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]

  def main(self):
    r = rospy.Rate(40)
    while not rospy.is_shutdown():
      haptics_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

      for i in range(len(self.robot_wrench)):
        wrench_i = self.robot_wrench[i]

        # propotional conversion
        haptics_wrench[i] = wrench_i * self.k_p

        # exponential conversion
        if wrench_i>=0.0:
          haptics_wrench[i] = exponential(wrench_i, self.exp_base, self.k_exp)
        else:
          haptics_wrench[i] = -exponential(-wrench_i, self.exp_base, self.k_exp)

        # log conversion
        if wrench_i>self.range_log:
          haptics_wrench[i] = logarithm(wrench_i, self.log_base, self.k_log)
        elif wrench_i<-self.range_log:
          haptics_wrench[i] = -logarithm(-wrench_i, self.log_base, self.k_log)
        else:
          haptics_wrench[i] = wrench_i * self.a_log

        self.haptics_wrench_msg.wrench.force.x = haptics_wrench[0]
        self.haptics_wrench_msg.wrench.force.y = haptics_wrench[1]
        self.haptics_wrench_msg.wrench.force.z = haptics_wrench[2]
        self.haptics_wrench_msg.wrench.torque.x = haptics_wrench[3]
        self.haptics_wrench_msg.wrench.torque.y = haptics_wrench[4]
        self.haptics_wrench_msg.wrench.torque.z = haptics_wrench[5]
        self.haptics_wrench_pub.publish(self.haptics_wrench_msg)

      r.sleep()

if __name__ == "__main__":
  rospy.init_node("force_feedback_from_robot")
  Tracker = force_feedback_from_robot()
  Tracker.main()
