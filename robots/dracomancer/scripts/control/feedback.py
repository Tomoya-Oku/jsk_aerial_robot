#!/usr/bin/env python3
"""Opt-in thesis haptics. Publishes Nm only; never arms a servo."""
import json
from pathlib import Path
import sys
import time

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String

helper_dir = Path(__file__).resolve().parent
if not (helper_dir/'shape_mapping.py').is_file():
    import rospkg
    helper_dir = Path(rospkg.RosPack().get_path('dracomancer'))/'scripts/control'
sys.path.insert(0,str(helper_dir))
sys.path.insert(0, str(helper_dir.parent/'haptics'))
from contact_feedback import ContactFeedback
from safety_feedback import SafetyFeedback, combine_torque, rate_limit
from shape_mapping import UrdfModel, HumanChain, BimanualHumanChain, DragonChain, ShapeMapper, DRAGON_JOINTS
from joint_mapping import JointMapper
from contact_model import ContactModel
from contacts import ContactInputs
from dracomancer.output_lock import claim_output


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
        dual = self.mode.startswith('dual_')
        if self.mode in ('joint', 'dual_joint'):
            if self.config.get('joint_mapping', {}).get('calibrated') is not True and not rospy.get_param('/use_sim_time', False):
                raise ValueError('anatomical calibration required for joint feedback')
            self.mapper = JointMapper(human,dragon,self.config,rest,dual)
        else:
            chain = BimanualHumanChain(human,self.config) if dual else HumanChain(human,self.config)
            self.mapper = ShapeMapper(chain,dragon,self.config,rest)
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
        msg = JointState()
        msg.name, msg.effort = self.names, [0.]*len(self.names)
        msg.header.stamp = rospy.Time.now()
        self.publisher.publish(msg)

    def main(self):
        rate = rospy.Rate(10.)
        while not rospy.is_shutdown():
            start = time.monotonic()
            try:
                torque = self.torque()
                if time.monotonic()-start>.1: raise ValueError('haptic computation over budget')
                torque = rate_limit(torque,self.previous,self.slew,min(start-self.last_tick,.1))
            except (ValueError,KeyError,np.linalg.LinAlgError) as error:
                rospy.logwarn_throttle(3.,'haptics zero: %s',str(error))
                torque = np.zeros(len(self.names))
                self.input_previous = None
            self.previous, self.last_tick = torque,start
            msg = JointState()
            msg.header.stamp, msg.name, msg.effort = rospy.Time.now(),self.names,torque.tolist()
            self.publisher.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    Feedback().main()
