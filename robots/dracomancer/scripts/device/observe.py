#!/usr/bin/env python3
"""Read-only, uncalibrated RViz preview of one seven-axis arm."""
import math
import time

import rospy
from sensor_msgs.msg import JointState
from spinal.msg import ServoStates
from spinal.srv import GetBoardInfo
from std_msgs.msg import String


class Observer:
    def __init__(self):
        if rospy.get_param('/use_sim_time', False):
            raise ValueError('Hardware preview requires real ROS time')
        self.names = rospy.get_param('joints')
        self.arm = rospy.get_param('~arm', 'right')
        default_ids = list(range(11, 18)) if self.arm == 'left' else list(range(1, 8))
        self.ids = rospy.get_param('~servo_ids', default_ids)
        if (self.arm not in ('right', 'left') or not isinstance(self.ids, list)
                or len(self.ids) != 7 or any(type(i) is not int or not 0 <= i <= 252 for i in self.ids)
                or len(set(self.ids)) != 7):
            raise ValueError('arm must be right/left and servo_ids must contain seven unique bus IDs')
        self.observed = [self.arm+'_joint_'+str(i) for i in range(1, 8)]
        if any(name not in self.names for name in self.observed):
            raise ValueError('Observed joints must exist in the model')
        dual = rospy.get_param('~dual_spinal', False)
        prefix = rospy.get_param('~spinal_ns', '/dracomancer' if dual else '/').rstrip('/')
        if dual:
            prefix += '/'+self.arm
        service = prefix+'/get_board_info'
        rospy.wait_for_service(service, timeout=10)
        boards = rospy.ServiceProxy(service, GetBoardInfo)().boards
        if len(boards) != 1 or boards[0].slave_id != 0:
            raise ValueError('Preview requires one direct-UART board')
        servos = boards[0].servos
        bus_ids = [servo.id for servo in servos]
        if len(set(bus_ids)) != len(bus_ids) or any(i not in bus_ids for i in self.ids):
            raise ValueError('Requested servo IDs are missing or duplicated on the bus')
        # Scan indexes are discovered, never inferred from physical IDs.
        self.indexes = [bus_ids.index(i) for i in self.ids]
        for index in self.indexes:
            servo = servos[index]
            if servo.external_encoder_flag or servo.joint_resolution != servo.servo_resolution:
                raise ValueError('Preview assumes direct shaft drive without an external encoder')
        self.zero = None
        self.last_stamp = rospy.Time(0)
        self.last_received = time.monotonic()
        # Publish display data only; no measured_joint_states or actuator commands.
        self.publisher = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.status = rospy.Publisher('preview/status', String, queue_size=1, latch=True)
        self.status.publish(String(data='UNCALIBRATED: waiting for the initial arm pose'))
        self.subscriber = rospy.Subscriber(prefix+'/servo/states', ServoStates, self.states, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.watchdog)
        rospy.logwarn('UNCALIBRATED read-only preview: %s IDs %s, startup pose = zero, signs = +1',
                      self.arm, self.ids)

    def states(self, msg):
        age = (rospy.Time.now()-msg.stamp).to_sec()
        indexes = [servo.index for servo in msg.servos]
        if (msg.stamp.to_sec() <= self.last_stamp.to_sec() or not -0.1 <= age <= 0.5
                or len(set(indexes)) != len(indexes)):
            return
        by_index = {servo.index: servo for servo in msg.servos}
        if any(index not in by_index for index in self.indexes):
            return
        ticks = [by_index[index].angle for index in self.indexes]
        if self.zero is None:
            self.zero = ticks
            self.previous = ticks
            self.delta = [0]*7
        # Spinal publishes a signed 16-bit position. Unwrap successive samples so
        # a count rollover does not create a full-scale jump in the preview.
        for i, tick in enumerate(ticks):
            self.delta[i] += (tick-self.previous[i]+32768) % 65536-32768
        self.previous = ticks
        angles = dict(zip(self.observed, [tick*2*math.pi/4096 for tick in self.delta]))
        state = JointState()
        state.header.stamp = msg.stamp
        state.name = self.names
        state.position = [angles.get(name, 0.0) for name in self.names]
        self.publisher.publish(state)
        self.last_stamp = msg.stamp
        self.last_received = time.monotonic()
        errors = {servo_id: by_index[index].error for servo_id, index in zip(self.ids, self.indexes)
                  if by_index[index].error}
        label = 'UNCALIBRATED: startup pose = zero; unobserved arm = model zero'
        if errors:
            label += '; servo errors '+str(errors)
            rospy.logwarn_throttle(3, 'Preview servo errors (bus ID: code): %s', errors)
        self.status.publish(String(data=label))

    def watchdog(self, _event):
        if time.monotonic()-self.last_received > 0.5:
            self.status.publish(String(data='UNCALIBRATED: stale servo data; display frozen'))
            rospy.logwarn_throttle(3, 'No fresh complete servo sample; preview is frozen')


if __name__ == '__main__':
    rospy.init_node('observe')
    try:
        observer = Observer()
        rospy.spin()
    except (ValueError, rospy.ROSException, rospy.ServiceException) as error:
        rospy.logfatal(str(error))
        raise SystemExit(1)
