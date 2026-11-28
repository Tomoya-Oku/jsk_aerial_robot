#!/usr/bin/env python3
"""Run the viewer/Gazebo backend, demo input, or wall setup."""
import argparse
import json
import math
import random
import threading
import time
import xml.etree.ElementTree as ET

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
        self.measured_pub = rospy.Publisher('measured_joint_states', JointState, queue_size=1)
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
            # Forward the simulator's measurement timestamp, never receipt time.
            self.measured_pub.publish(msg)

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
                    self.measured_pub.publish(msg)
                self.publish_status()
            time.sleep(0.025)

    def shutdown(self):
        self.alive = False
        self.stop(None)


class NaturalMotion:
    """Generate gradual paired arm movements from a reproducible random seed."""

    interval = 7.0

    def __init__(self, seed):
        self.rng = random.Random(seed)
        self.previous = self.target = [0.0] * 14
        self.segment = -1

    def keyframe(self):
        positions = []
        lift = self.rng.uniform(-1, 1)
        for _ in range(2):
            reach = self.rng.uniform(-1, 1)
            bend = self.rng.uniform(-1, 1)
            positions.extend((
                .24 * reach,
                .16 * lift + .11 * bend,
                self.rng.uniform(-.18, .18),
                -.40 - .23 * bend + .06 * reach,
                self.rng.uniform(-.28, .28),
                .13 * bend + self.rng.uniform(-.09, .09),
                self.rng.uniform(-.16, .16),
            ))
        return positions

    def position(self, elapsed):
        segment = int(elapsed / self.interval)
        while self.segment < segment:
            self.previous, self.target = self.target, self.keyframe()
            self.segment += 1
        phase = elapsed / self.interval - segment
        # Zero velocity and acceleration at each keyframe.
        blend = phase ** 3 * (10 + phase * (-15 + 6 * phase))
        return [start + blend * (end - start) for start, end in zip(self.previous, self.target)]


def publish_demo_motion():
    rospy.init_node("shape_input")
    if not rospy.get_param("/use_sim_time", False):
        raise RuntimeError("shape_input requires simulation time")
    pattern = rospy.get_param("~pattern", "sine")
    if pattern not in ("sine", "natural"):
        raise ValueError("shape_input pattern must be sine or natural")
    natural = NaturalMotion(rospy.get_param("~seed", 27))
    publisher = rospy.Publisher("/dracomancer/joint_cmd", JointState, queue_size=1)
    names = ["%s_joint_%d" % (side, i) for side in ("right", "left") for i in range(1, 8)]
    rate = rospy.Rate(40)
    start = None
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if now == rospy.Time(0):
            rate.sleep()
            continue
        if start is None:
            start = now
        # Allow five seconds for Gazebo quick_init_pose before moving the arms.
        elapsed = max(0., (now-start).to_sec()-5.)
        msg = JointState()
        msg.header.stamp, msg.name = now, names
        if pattern == "natural":
            msg.position = natural.position(elapsed)
        else:
            msg.position = [.25*math.sin(.3*elapsed), 0., 0., .4*math.sin(.2*elapsed), 0., 0., 0.] + [0.]*7
        publisher.publish(msg)
        rate.sleep()


def wall_sdf(position, size, friction):
    if (len(position) != 3 or len(size) != 3 or
            not all(math.isfinite(float(v)) for v in list(position) + list(size)) or
            any(float(v) <= 0 for v in size) or not math.isfinite(friction) or friction < 0):
        raise ValueError("invalid wall geometry/friction")
    root = ET.Element("sdf", version="1.6")
    model = ET.SubElement(root, "model", name="contact_wall")
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = " ".join(map(str, position)) + " 0 0 0"
    link = ET.SubElement(model, "link", name="wall")
    for tag in ("collision", "visual"):
        item = ET.SubElement(link, tag, name=tag)
        geom = ET.SubElement(item, "geometry")
        ET.SubElement(ET.SubElement(geom, "box"), "size").text = " ".join(map(str, size))
        if tag == "collision":
            ode = ET.SubElement(ET.SubElement(ET.SubElement(item, "surface"), "friction"), "ode")
            for key in ("mu", "mu2"):
                ET.SubElement(ode, key).text = str(friction)
    sensor = ET.SubElement(link, "sensor", name="wall_contacts", type="contact")
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(ET.SubElement(sensor, "contact"), "collision").text = "collision"
    plugin = ET.SubElement(sensor, "plugin", name="wall_contact", filename="libwall_contact.so")
    ET.SubElement(plugin, "robot").text = "dragon"
    ET.SubElement(plugin, "link").text = "link1"
    return ET.tostring(root, encoding="unicode")


def spawn_wall():
    from gazebo_msgs.srv import SpawnModel
    from geometry_msgs.msg import Pose

    rospy.init_node("contact_wall")
    settings = rospy.get_param("/dracomancer/contact/wall")
    xml = wall_sdf(settings["position"], settings["size"], float(settings["friction"]))
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    initial = Pose()
    initial.orientation.w = 1.0
    response = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)(
        "contact_wall", xml, "", initial, "world")
    if not response.success:
        raise RuntimeError(response.status_message)


def run_backend():
    rospy.init_node('control')
    try:
        node = Control()
        rospy.spin()
    except (rospy.ROSException, RuntimeError, ValueError) as error:
        rospy.logfatal(str(error))
        raise SystemExit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", nargs="?", choices=("backend", "motion", "wall"), default="backend")
    args = parser.parse_args(rospy.myargv()[1:])
    {"backend": run_backend, "motion": publish_demo_motion, "wall": spawn_wall}[args.mode]()
