#!/usr/bin/env python3
"""Read-only, uncalibrated RViz preview of the seven-axis arms."""
import math
import threading
import time

import rospy
from sensor_msgs.msg import JointState
from spinal.msg import ServoStates
from spinal.srv import GetBoardInfo
from std_msgs.msg import String


ARM_IDS = {'right': list(range(1, 8)), 'left': list(range(11, 18))}
TIMEOUT = 0.5


class ArmState:
    def __init__(self, name, ids, prefix, suffix, model_joints):
        self.name = name
        self.ids = ids
        if len(set(ids)) != 7:
            raise ValueError(f'{name}: servo_ids must contain seven unique bus IDs')
        self.joints = [f'{name}_joint_{i}' for i in range(1, 8)]
        if any(joint not in model_joints for joint in self.joints):
            raise ValueError(f'{name}: observed joints must exist in the model')
        service = f'{prefix}/get_board_info{suffix}'
        rospy.wait_for_service(service, timeout=10)
        boards = rospy.ServiceProxy(service, GetBoardInfo)().boards
        if len(boards) != 1 or boards[0].slave_id != 0:
            raise ValueError(f'{name}: preview requires one direct-UART board')
        servos = boards[0].servos
        bus_ids = [servo.id for servo in servos]
        if len(set(bus_ids)) != len(bus_ids) or any(i not in bus_ids for i in ids):
            raise ValueError(f'{name}: requested servo IDs {ids} do not match detected IDs {bus_ids}')
        # Each UART has its own scan order, including overlapping indexes.
        self.indexes = [bus_ids.index(i) for i in ids]
        for index in self.indexes:
            servo = servos[index]
            if servo.external_encoder_flag or servo.joint_resolution != servo.servo_resolution:
                raise ValueError(f'{name}: preview requires direct shaft drive without an external encoder')
        self.topic = f'{prefix}/servo/states{suffix}'
        self.previous = None
        self.delta = [0]*7
        self.last_stamp = rospy.Time(0)
        self.last_received = time.monotonic()
        self.errors = {}

    def update(self, msg):
        age = (rospy.Time.now()-msg.stamp).to_sec()
        indexes = [servo.index for servo in msg.servos]
        if (msg.stamp <= self.last_stamp or not -0.1 <= age <= TIMEOUT
                or len(set(indexes)) != len(indexes)):
            return False
        by_index = {servo.index: servo for servo in msg.servos}
        if any(index not in by_index for index in self.indexes):
            return False
        ticks = [by_index[index].angle for index in self.indexes]
        if self.previous is None:
            self.previous = ticks
        # Spinal publishes a signed 16-bit position. Unwrap successive samples so
        # a count rollover does not create a full-scale jump in the preview.
        for i, tick in enumerate(ticks):
            self.delta[i] += (tick-self.previous[i]+32768) % 65536-32768
        self.previous = ticks
        self.last_stamp = msg.stamp
        self.last_received = time.monotonic()
        self.errors = {servo_id: by_index[index].error for servo_id, index in zip(self.ids, self.indexes)
                       if by_index[index].error}
        return True


class Observer:
    def __init__(self):
        if rospy.get_param('/use_sim_time', False):
            raise ValueError('Hardware preview requires real ROS time')
        self.names = rospy.get_param('joints')
        dual = rospy.get_param('~dual_spinal', False)
        selection = rospy.get_param('~arm', 'both' if dual else 'right')
        if selection not in ('both', *ARM_IDS):
            raise ValueError('arm must be both, right, or left')
        if selection == 'both' and not dual:
            raise ValueError('arm:=both requires dual_spinal:=true')
        names = tuple(ARM_IDS) if selection == 'both' else (selection,)
        default_ids = [servo_id for name in names for servo_id in ARM_IDS[name]]
        ids = rospy.get_param('~servo_ids', default_ids)
        if (not isinstance(ids, list) or len(ids) != len(default_ids)
                or any(type(i) is not int or not 0 <= i <= 252 for i in ids)):
            raise ValueError(f'servo_ids must contain {len(default_ids)} physical bus IDs')
        prefix = rospy.get_param('~spinal_ns', '/dracomancer' if dual else '/').rstrip('/')
        self.arms = [ArmState(name, ids[i*7:(i+1)*7], prefix, f'/{name}' if dual else '', self.names)
                     for i, name in enumerate(names)]
        self.lock = threading.Lock()
        # Publish display data only; no measured_joint_states or actuator commands.
        self.publisher = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.status = rospy.Publisher('preview/status', String, queue_size=1, latch=True)
        self.publish_status()
        self.subscribers = [rospy.Subscriber(arm.topic, ServoStates, self.states, callback_args=arm, queue_size=1)
                            for arm in self.arms]
        self.timer = rospy.Timer(rospy.Duration(TIMEOUT), self.watchdog)
        for arm in self.arms:
            rospy.logwarn('UNCALIBRATED read-only preview: %s IDs %s, startup pose = zero, signs = +1',
                          arm.name, arm.ids)

    def states(self, msg, arm):
        with self.lock:
            if not arm.update(msg):
                return
            # Each arm establishes its own zero and keeps its last valid pose.
            angles = {joint: tick*2*math.pi/4096 for observed in self.arms
                      for joint, tick in zip(observed.joints, observed.delta)}
            state = JointState()
            state.header.stamp = max(observed.last_stamp for observed in self.arms)
            state.name = self.names
            state.position = [angles.get(name, 0.0) for name in self.names]
            self.publisher.publish(state)
            self.publish_status()

    def publish_status(self):
        now = time.monotonic()
        labels = []
        for arm in self.arms:
            if now-arm.last_received > TIMEOUT:
                label = 'stale servo data, display frozen'
            elif arm.previous is None:
                label = 'waiting for the initial arm pose'
            else:
                label = 'startup pose = zero'
            if arm.errors:
                label += f', servo errors {arm.errors}'
            labels.append(f'{arm.name}: {label}')
        self.status.publish(String(data='UNCALIBRATED: '+'; '.join(labels)))

    def watchdog(self, _event):
        with self.lock:
            self.publish_status()
            now = time.monotonic()
            stale = [arm.name for arm in self.arms if now-arm.last_received > TIMEOUT]
            if stale:
                rospy.logwarn_throttle(3, 'No fresh complete servo sample for %s; preview is frozen for that arm',
                                       ', '.join(stale))
            errors = {arm.name: arm.errors for arm in self.arms if arm.errors}
            if errors:
                rospy.logwarn_throttle(3, 'Preview servo errors (bus ID: code): %s', errors)


if __name__ == '__main__':
    rospy.init_node('observe')
    try:
        observer = Observer()
        rospy.spin()
    except (ValueError, rospy.ROSException, rospy.ServiceException) as error:
        rospy.logfatal(str(error))
        raise SystemExit(1)
