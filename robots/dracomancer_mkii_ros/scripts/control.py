#!/usr/bin/env python3
"""ROS1 joint_cmd bridge, with exclusive position/effort controllers in Gazebo."""
import json
import math
import threading
import time

import rospy
from controller_manager_msgs.srv import LoadController, SwitchController, ListControllers
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


class Control:
    def __init__(self):
        self.names = rospy.get_param('joints')
        self.backend = rospy.get_param('~backend', 'viewer')
        if self.backend not in ('viewer', 'gazebo'):
            raise ValueError('backend must be viewer or gazebo')
        self.limit = float(rospy.get_param('max_torque', 2.0))
        self.timeout = float(rospy.get_param('command_timeout', 0.35))
        if not (math.isfinite(self.limit) and self.limit > 0 and 0.05 <= self.timeout <= 2):
            raise ValueError('Invalid torque limit or timeout')
        self.lock = threading.RLock()
        self.position = dict.fromkeys(self.names, 0.0)
        self.mode = dict.fromkeys(self.names, False)
        self.last_state = dict.fromkeys(self.names, 0.0)
        self.last_cmd = dict.fromkeys(self.names, 0.0)
        self.release = dict.fromkeys(self.names, 0.0)
        self.applied = dict.fromkeys(self.names, 0.0)
        self.alive = True
        self.status_pub = rospy.Publisher('control/status', String, queue_size=1, latch=True)
        if self.backend == 'gazebo':
            self.position_pub = {n: rospy.Publisher(n+'_position/command', Float64, queue_size=1) for n in self.names}
            self.effort_pub = {n: rospy.Publisher(n+'_effort/command', Float64, queue_size=1) for n in self.names}
            self.initialize_controllers()
            self.state_sub = rospy.Subscriber('joint_states', JointState, self.states, queue_size=1)
        else:
            self.state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.angle_sub = rospy.Subscriber('joint_cmd', JointState, self.angles, queue_size=1)
        self.torque_sub = rospy.Subscriber('torque_cmd', JointState, self.torques, queue_size=1)
        self.services = [rospy.Service('control/'+n+'/torque_mode', SetBool,
                         lambda request, name=n: self.change_mode(name, request.data)) for n in self.names]
        self.stop_service = rospy.Service('control/stop', Trigger, self.stop)
        rospy.on_shutdown(self.shutdown)
        self.thread = threading.Thread(target=self.watchdog, daemon=True)
        self.thread.start()

    def initialize_controllers(self):
        rospy.wait_for_service('controller_manager/load_controller', timeout=90)
        load = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
        listing = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
        self.switch = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
        # Refuse to attach to a pre-existing controller owner.
        existing = {c.name for c in listing().controller}
        controllers = ['joint_state_controller'] + [n+s for n in self.names for s in ('_position', '_effort')]
        if existing.intersection(controllers):
            raise RuntimeError('Controllers already loaded; stop the previous control node first')
        for name in controllers:
            if not load(name).ok:
                raise RuntimeError('Failed to load '+name)
        start = ['joint_state_controller'] + [n+'_position' for n in self.names]
        if not self.switch(start, [], 2, False, 3.0).ok:
            raise RuntimeError('Could not start position controllers')

    @staticmethod
    def valid(msg, field):
        values = getattr(msg, field)
        return (len(msg.name) == len(values) and len(set(msg.name)) == len(msg.name)
                and all(math.isfinite(v) for v in values))

    def states(self, msg):
        if not self.valid(msg, 'position'):
            return
        with self.lock:
            for name, value in zip(msg.name, msg.position):
                if name in self.position:
                    self.position[name] = value
                    self.last_state[name] = time.monotonic()

    def angles(self, msg):
        if not self.valid(msg, 'position') or any(n not in self.position for n in msg.name):
            rospy.logwarn_throttle(2, 'Rejected malformed angle command')
            return
        with self.lock:
            for name, value in zip(msg.name, msg.position):
                if not self.mode[name] and time.monotonic() >= self.release[name]:
                    if self.backend == 'viewer':
                        self.position[name] = value
                    else:
                        self.position_pub[name].publish(value)

    def torques(self, msg):
        # Reject the complete packet rather than applying a partial malformed command.
        if (not self.valid(msg, 'effort') or any(n not in self.position for n in msg.name)
                or any(abs(t) > self.limit for t in msg.effort)):
            rospy.logwarn_throttle(2, 'Rejected torque command: names, finite values, or Nm limit')
            self.stop(None)
            return
        with self.lock:
            now = time.monotonic()
            for name, value in zip(msg.name, msg.effort):
                if self.backend == 'gazebo' and self.mode[name] and now-self.last_state[name] < self.timeout:
                    self.effort_pub[name].publish(value)
                    self.last_cmd[name] = now
                    self.applied[name] = value

    def change_mode(self, name, enabled):
        with self.lock:
            if self.backend != 'gazebo':
                return SetBoolResponse(False, 'RViz viewer has no torque physics')
            if time.monotonic()-self.last_state[name] > self.timeout:
                return SetBoolResponse(False, 'Joint feedback is stale or missing')
            if self.mode[name] == enabled:
                return SetBoolResponse(True, 'Already in requested mode')
            self.effort_pub[name].publish(0.0)
            start = name+('_effort' if enabled else '_position')
            stop = name+('_position' if enabled else '_effort')
            try:
                response = self.switch([start], [stop], 2, False, 2.0)
            except rospy.ServiceException as error:
                return SetBoolResponse(False, str(error))
            if not response.ok:
                return SetBoolResponse(False, 'Strict controller switch failed')
            self.mode[name] = enabled
            self.applied[name] = 0.0
            self.last_cmd[name] = time.monotonic()
            if not enabled:
                # Let the GUI adopt measured q before its periodic position stream resumes.
                self.position_pub[name].publish(self.position[name])
                self.release[name] = time.monotonic()+0.4
            self.publish_status()
            return SetBoolResponse(True, 'torque' if enabled else 'position')

    def stop(self, _request):
        with self.lock:
            if self.backend == 'gazebo':
                for name in self.names:
                    self.effort_pub[name].publish(0.0)
                    self.applied[name] = 0.0
        return TriggerResponse(True, 'Zero torque sent; effort mode remains free under gravity')

    def publish_status(self):
        self.status_pub.publish(json.dumps({'backend': self.backend, 'torque_mode': self.mode,
                                           'position': self.position, 'commanded_torque': self.applied,
                                           'max_torque': self.limit}))

    def watchdog(self):
        # Wall clock remains effective when Gazebo /clock is paused.
        while self.alive and not rospy.is_shutdown():
            with self.lock:
                now = time.monotonic()
                if self.backend == 'gazebo':
                    for name in self.names:
                        if self.mode[name] and (now-self.last_cmd[name] > self.timeout or
                                                now-self.last_state[name] > self.timeout):
                            self.effort_pub[name].publish(0.0)
                            self.applied[name] = 0.0
                else:
                    msg = JointState()
                    msg.header.stamp = rospy.Time.now()
                    msg.name = self.names
                    msg.position = [self.position[n] for n in self.names]
                    self.state_pub.publish(msg)
                self.publish_status()
            time.sleep(0.025)

    def shutdown(self):
        self.alive = False
        self.stop(None)


if __name__ == '__main__':
    rospy.init_node('control')
    try:
        node = Control()
        rospy.spin()
    except (rospy.ROSException, RuntimeError, ValueError) as error:
        rospy.logfatal(str(error))
        raise SystemExit(1)
