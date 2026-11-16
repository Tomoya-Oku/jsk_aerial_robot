#!/usr/bin/env python3
"""Exercise the viewer on an isolated ROS master, with no hardware backend."""
import json
import time
import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

rospy.init_node('view_test')
rospy.wait_for_service('/dracomancer/control/right_joint_1/torque_mode', timeout=30)
mode = rospy.ServiceProxy('/dracomancer/control/right_joint_1/torque_mode', SetBool)
assert not mode(True).success
assert not rospy.get_param('/use_sim_time', False)
data = []
sub = rospy.Subscriber('/dracomancer/joint_states', JointState, data.append)
pub = rospy.Publisher('/dracomancer/joint_cmd', JointState, queue_size=1)


def until(check):
    end = time.monotonic()+5
    while time.monotonic() < end:
        if check():
            return
        time.sleep(0.02)
    raise AssertionError('Viewer response timeout')


until(lambda: pub.get_num_connections() > 0 and data)
pub.publish(JointState(name=['right_joint_1'], position=[0.4]))
until(lambda: data[-1].position[data[-1].name.index('right_joint_1')] == 0.4)
assert len(data[-1].name) == 14
pub.publish(JointState(name=['right_joint_1'], position=[float('nan')]))
time.sleep(0.15)
assert data[-1].position[data[-1].name.index('right_joint_1')] == 0.4
print(json.dumps({'viewer': True, 'joints': 14, 'angles': True,
                  'invalid_angles_rejected': True, 'torque_refused': True}))
