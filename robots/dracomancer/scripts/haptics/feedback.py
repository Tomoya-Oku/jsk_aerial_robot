#!/usr/bin/env python3
"""Opt-in haptic feedback with calibrated torque caps and freshness checks."""
import json
import time

import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from control.contacts import ContactInputs, ContactModel
from control.output_lock import claim_output
from control.shape import DRAGON_JOINTS, DragonChain, JointMapper, UrdfModel, make_mapper


def _reflect(mapping, robot_jacobian, wrench, gain):
    return float(gain) * mapping.T.dot(robot_jacobian.T.dot(wrench))


class ContactFeedback:
    """Apply tau_h = M.T * J_r.T * wrench.

    ``robot_jacobian`` maps robot joint velocity to the wrench-frame twist and
    ``mapping_jacobian`` maps human joint velocity to robot joint velocity.
    Keeping both matrices as inputs lets the fixed-link and global-shape
    mappings share this haptic implementation.
    """

    def __init__(self, robot_joint_count, human_joint_count, gain=1.0):
        self.robot_joint_count = int(robot_joint_count)
        self.human_joint_count = int(human_joint_count)
        self.gain = float(gain)
        if self.robot_joint_count <= 0 or self.human_joint_count <= 0 or not np.isfinite(self.gain) or self.gain < 0:
            raise ValueError("joint counts must be positive and contact gain nonnegative")

    def compute(self, wrench, robot_jacobian, mapping_jacobian):
        wrench = np.asarray(wrench, dtype=float)
        robot_jacobian = np.asarray(robot_jacobian, dtype=float)
        mapping_jacobian = np.asarray(mapping_jacobian, dtype=float)

        expected_robot = (6, self.robot_joint_count)
        expected_mapping = (self.robot_joint_count, self.human_joint_count)
        if wrench.shape != (6,):
            raise ValueError("contact wrench must have 6 elements")
        if robot_jacobian.shape != expected_robot:
            raise ValueError("robot Jacobian must have shape %s" % (expected_robot,))
        if mapping_jacobian.shape != expected_mapping:
            raise ValueError("mapping Jacobian must have shape %s" % (expected_mapping,))
        if not (np.all(np.isfinite(wrench)) and
                np.all(np.isfinite(robot_jacobian)) and
                np.all(np.isfinite(mapping_jacobian))):
            raise ValueError("contact feedback input contains non-finite values")

        return _reflect(mapping_jacobian, robot_jacobian, wrench, self.gain)


class SafetyFeedback:
    """Return rejected robot-shape motion to the operator as resistance."""

    def __init__(self, robot_stiffness, human_damping, gain=1.0):
        self.robot_stiffness = np.asarray(robot_stiffness, dtype=float)
        self.human_damping = np.asarray(human_damping, dtype=float)
        self.gain = float(gain)
        # Nonnegative gains are required by the thesis virtual potential.
        if (self.robot_stiffness.ndim != 1 or self.human_damping.ndim != 1
                or not self.robot_stiffness.size or not self.human_damping.size
                or not all(np.all(np.isfinite(v)) for v in (self.robot_stiffness, self.human_damping))
                or np.any(self.robot_stiffness < 0) or np.any(self.human_damping < 0)
                or not np.isfinite(self.gain) or self.gain < 0):
            raise ValueError("stiffness, damping, and safety gain must be finite and nonnegative")

    @property
    def robot_joint_count(self):
        return self.robot_stiffness.size

    @property
    def human_joint_count(self):
        return self.human_damping.size

    def compute(self, shape_error, mapping_jacobian, human_velocity):
        shape_error = np.asarray(shape_error, dtype=float)
        mapping_jacobian = np.asarray(mapping_jacobian, dtype=float)
        human_velocity = np.asarray(human_velocity, dtype=float)

        expected_mapping = (self.robot_joint_count, self.human_joint_count)
        if shape_error.shape != (self.robot_joint_count,):
            raise ValueError("shape error size does not match robot joints")
        if mapping_jacobian.shape != expected_mapping:
            raise ValueError("mapping Jacobian must have shape %s" % (expected_mapping,))
        if human_velocity.shape != (self.human_joint_count,):
            raise ValueError("human velocity size does not match human joints")
        if not (np.all(np.isfinite(shape_error)) and
                np.all(np.isfinite(mapping_jacobian)) and
                np.all(np.isfinite(human_velocity))):
            raise ValueError("safety feedback input contains non-finite values")

        # shape_error = q_ref - q_cmd. The minus sign makes the cue resist
        # motion toward the rejected shape rather than push the operator into it.
        robot_resistance = -self.robot_stiffness * shape_error
        spring = mapping_jacobian.T.dot(robot_resistance)
        damping = -self.human_damping * human_velocity
        return self.gain * spring + damping


def combine_torque(contact, safety, external, limits, deadband=0.0):
    contact = np.asarray(contact, dtype=float)
    safety = np.asarray(safety, dtype=float)
    external = np.asarray(external, dtype=float)
    limits = np.abs(np.asarray(limits, dtype=float))
    if not (contact.shape == safety.shape == external.shape == limits.shape):
        raise ValueError("haptic torque vectors must have the same shape")
    if not (np.all(np.isfinite(contact)) and np.all(np.isfinite(safety)) and
            np.all(np.isfinite(external)) and np.all(np.isfinite(limits))):
        raise ValueError("haptic torque vectors must contain finite values")

    total = contact + safety + external
    total[np.abs(total) < abs(float(deadband))] = 0.0
    return np.clip(total, -limits, limits)


def rate_limit(current, previous, limits_per_second, dt):
    current = np.asarray(current, dtype=float)
    previous = np.asarray(previous, dtype=float)
    limits = np.abs(np.asarray(limits_per_second, dtype=float))
    if not (current.shape == previous.shape == limits.shape):
        raise ValueError("torque rate-limit vectors must have the same shape")
    if not (np.all(np.isfinite(current)) and np.all(np.isfinite(previous)) and
            np.all(np.isfinite(limits))):
        raise ValueError("torque rate-limit vectors must contain finite values")
    if dt <= 0.0:
        return previous.copy()
    step = limits * float(dt)
    return previous + np.clip(current - previous, -step, step)


class Feedback:
    def __init__(self):
        rospy.init_node('shape_feedback')
        self.device = rospy.get_param('~device_ns', '/dracomancer').rstrip('/')
        self.robot = rospy.get_param('~robot_name', 'dragon')
        self.config = rospy.get_param('~shape')
        self.mode = rospy.get_param('~mapping_mode', 'shape')
        human = UrdfModel(rospy.get_param(self.device+'/robot_description'))
        xml = rospy.get_param('/'+self.robot+'/robot_description')
        dragon = DragonChain(UrdfModel(xml), self.config['dragon_tail'])
        rest = rospy.get_param('~startup_pose', [0., np.pi/2]*3)
        self.mapper = make_mapper(
            human, dragon, self.config, rest, self.mode,
            simulation=rospy.get_param('/use_sim_time', False))
        self.contact_model = ContactModel(xml)
        self.names = self.mapper.human.names
        n = len(self.names)
        cfg = rospy.get_param('~haptics')
        self.contact = ContactFeedback(6,n,cfg['contact_gain'])
        self.safety = SafetyFeedback(cfg['stiffness'],[cfg['damping']]*n,cfg['safety_gain'])
        self.limit = np.full(n,float(cfg['max_torque']))
        self.slew = np.full(n,float(cfg['max_torque_rate']))
        if not np.all(np.isfinite([*self.limit,*self.slew])) or np.any(self.limit<=0) or np.any(self.slew<=0):
            raise ValueError('positive finite haptic torque/rate limits required')
        self.anchor = rospy.get_param('~anchor_mode', 'none')
        if self.anchor not in ('none','tail','mid'):
            raise ValueError('anchor_mode must be none, tail, or mid')
        self.data = None
        self.received = 0.
        self.contacts=ContactInputs(rospy.get_param('~contacts',[{}]),self.robot)
        self.hardware = rospy.get_param('~hardware', False)
        self.hardware_limits = None
        if self.hardware:
            rospy.Subscriber(self.device+'/control/status', String, self.hardware_status, queue_size=1)
        self.previous = np.zeros(n)
        self.input_previous = None
        self.last_tick = time.monotonic()
        topic = self.device+'/torque_cmd'
        self.owner = claim_output(topic,topic)
        self.publisher = rospy.Publisher(topic,JointState,queue_size=1)
        rospy.Subscriber(self.device+'/shape/diagnostics',String,self.state,queue_size=1)
        for i,spec in enumerate(self.contacts.specs):
            rospy.Subscriber(spec['topic'],WrenchStamped,lambda msg,index=i:self.contact_state(msg,index),queue_size=1)
        rospy.on_shutdown(self.stop)

    def hardware_status(self, msg):
        try:
            status = json.loads(msg.data)
            limits = {
                name: float(status['max_torque_by_joint'][name])
                for name in status['measured_joints'] if status['available'][name]
            } if status['backend'] == 'spinal_direct' else {}
            if any(not np.isfinite(limit) or limit <= 0 for limit in limits.values()):
                raise ValueError('invalid actuator torque limit')
            self.hardware_limits = (time.monotonic(), limits)
        except (KeyError, TypeError, ValueError):
            self.hardware_limits = None

    def state(self,msg):
        try:
            self.data = json.loads(msg.data)
            self.received = time.monotonic()
        except (ValueError,TypeError):
            self.data = None

    def contact_state(self,msg,index=0):
        self.contacts.update(index,msg,time.monotonic())

    def torque(self):
        data = self.data
        now = rospy.Time.now().to_sec()
        if (not data or not data.get('active') or not data.get('ok')
                or time.monotonic()-self.received>.25
                or not 0 <= now-data['stamp'] <= .25
                or not 0 <= now-data['input_stamp'] <= .5):
            raise ValueError('haptic nominal packet unavailable')
        human = data['human']
        q = np.asarray(data['q_ref'])
        mapping = self.mapper.mapping_jacobian(human,data['q_previous'],q)
        velocity = np.zeros(len(self.names))
        current = self.mapper.human.values(human) if isinstance(self.mapper,JointMapper) else np.array([human[n] for n in self.names])
        if self.input_previous is not None:
            stamp, previous = self.input_previous
            dt = data['input_stamp']-stamp
            if dt>0: velocity = (current-previous)/dt
        self.input_previous = (data['input_stamp'],current)
        positions = dict(data['robot'],**dict(zip(DRAGON_JOINTS,q)))
        for i,angle in enumerate(data.get('evaluation_gimbals',[])):
            positions[f'gimbal{i//2+1}_{"roll" if i%2==0 else "pitch"}']=angle
        anchor = None
        if self.anchor != 'none':
            anchor = {'link':'inter_joint2','xyz':[0.,0.,0.]} if self.anchor=='mid' else {'link':'link4','xyz':self.config['dragon_tail']}
        raw=self.safety.compute(q-np.asarray(data['q_command']),mapping,velocity)
        if self.contact.gain>0:
            if not self.contact_model.required_joints.issubset(data['robot']):
                raise ValueError('measured contact-frame geometry unavailable')
            for link,point,wrench,local in self.contacts.values(now,time.monotonic()):
                wrench=np.asarray(wrench)
                if local:wrench=self.contact_model.wrench_in_fc(data['robot'],wrench,link)
                jacobian=self.contact_model.jacobian(positions,DRAGON_JOINTS,point,anchor,link)
                raw+=self.contact.compute(wrench,jacobian,mapping)
        # Dual power transformation for q_H = scale*(q_CAD-zero).
        if isinstance(self.mapper,JointMapper): raw *= self.mapper.human.scale
        return combine_torque(raw,np.zeros_like(raw),np.zeros_like(raw),self.limit)

    def stop(self):
        if self.hardware:
            # The hardware adapter stops the session when this stream expires.
            return
        msg = JointState()
        msg.name, msg.effort = self.names, [0.]*len(self.names)
        msg.header.stamp = rospy.Time.now()
        self.publisher.publish(msg)

    def main(self):
        rate = rospy.Rate(10.)
        while not rospy.is_shutdown():
            start = time.monotonic()
            hardware_limits = self.hardware_limits
            try:
                if self.hardware and (not hardware_limits or start-hardware_limits[0] >= .15):
                    raise ValueError('fresh calibrated actuator status required')
                torque = self.torque()
                if time.monotonic()-start>.1: raise ValueError('haptic computation over budget')
                torque = rate_limit(torque,self.previous,self.slew,min(start-self.last_tick,.1))
            except (ValueError,KeyError,np.linalg.LinAlgError) as error:
                rospy.logwarn_throttle(3.,'haptics zero: %s',str(error))
                torque = np.zeros(len(self.names))
                self.input_previous = None
                if self.hardware:
                    self.previous, self.last_tick = torque, start
                    rate.sleep()
                    continue
            names = self.names
            if self.hardware:
                # Apply both the haptic limit and the live actuator cap, and omit unmapped joints.
                limits = hardware_limits[1]
                caps = np.array([limits.get(name, 0.) for name in self.names])
                torque = np.clip(torque, -caps, caps)
                names = [name for name in self.names if name in limits]
            self.previous, self.last_tick = torque,start
            msg = JointState()
            msg.header.stamp, msg.name = rospy.Time.now(), names
            msg.effort = torque[caps > 0].tolist() if self.hardware else torque.tolist()
            if names:
                self.publisher.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    Feedback().main()
