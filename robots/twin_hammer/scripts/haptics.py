import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped

class Haptics():

    def __init__(self):
        self.robot_wrench_pub = rospy.Publisher('/twin_hammer/robot_wrench', list[float], queue_size=1)
        self.robot_wrench_sub = rospy.Subscriber('/cfs/data', WrenchStamped, self.robot_wrench_cb)

    def robot_wrench_cb(self, msg):
        fx = msg.wrench.force.z
        fy = msg.wrench.force.x
        fz = msg.wrench.force.y
        tx = msg.wrench.torque.z
        ty = msg.wrench.torque.x
        tz = msg.wrench.torque.y
        wrench_local = [fx, fy, fz, tx, ty, tz]
        delay_param = 0.05

        filtered_robot_wrench_local = [0.0]*6
        for i in range(6):
            filtered_robot_wrench_local[i] = (1 - delay_param) * filtered_robot_wrench_local[i] + delay_param * wrench_local[i]
        
        wrench_world = np.dot(self.Ad_R_robot, self.filtered_robot_wrench_local)
        
        if self.frame == "local":
            robot_wrench = filtered_robot_wrench_local
        else:
            robot_wrench = wrench_world

        self.robot_wrench_pub.publish(robot_wrench)
            
if __name__ == "__main__":
  rospy.init_node("haptics")
  Tracker = Haptics()
  Tracker.main()