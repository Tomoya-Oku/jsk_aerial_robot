#!/usr/bin/env python
import sys
import time
import rospy
import math
from geometry_msgs.msg import Vector3Stamped, Vector3
from sensor_msgs.msg import JointState

class MocapDragon():
  
  def __init__(self):
    self.att_control_pub = rospy.Publisher("/dragon/final_target_baselink_rpy", Vector3Stamped, queue_size=1)
    self.desire_att = Vector3Stamped(vector = Vector3(0,0,0))
    self.desire_att.header.stamp = ros.time.now()


while not rospy.is_shutdown():
  if self.desire_att.vector.x == -1.57:
     desire_att.vector.x = 0
  else:
    desire_att.vector.x = -1.57
  att_control_pub.publish(desire_att)

  time.sleep(10)

if __name__ == "__main__":
  rospy.init_node("mocap_dragon")
  mocap_dragon = 