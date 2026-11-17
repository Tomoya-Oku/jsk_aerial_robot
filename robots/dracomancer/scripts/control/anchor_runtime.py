#!/usr/bin/env python3
"""Single navigation owner for tail/midpoint world-position compensation."""
import json
import time
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from contact_model import ContactModel
from contact_math import anchor_position
from shape_mapping import DRAGON_JOINTS
from navigation import quaternion_matrix


class AnchorRuntime:
    def init_anchor(self):
        self.anchor_mode = rospy.get_param('~anchor_mode','none')
        if self.anchor_mode not in ('none','tail','mid'):
            raise ValueError('anchor_mode must be none, tail, or mid')
        self.anchor_data = self.anchor_odom = None
        self.anchor_base_odom=None
        self.base_received=0.
        self.anchor_received = self.odom_received = 0.
        self.anchor_reference = None
        self.anchor_last = None
        self.anchor_model = None
        self.anchor_tail = rospy.get_param('~dragon_tail',[.474,0,0])
        self.allow_joystick = bool(rospy.get_param('~allow_joystick',True))
        if self.anchor_mode != 'none':
            rospy.Subscriber(self.device_ns+'/shape/diagnostics',String,self.anchor_state,queue_size=1)
        rospy.Subscriber('/'+self.robot_name+'/uav/cog/odom',Odometry,self.anchor_pose,queue_size=1)
        if self.anchor_mode!='none':
            rospy.Subscriber('/'+self.robot_name+'/uav/baselink/odom',Odometry,self.anchor_base_pose,queue_size=1)

    def anchor_state(self,msg):
        try:
            self.anchor_data = json.loads(msg.data)
            self.anchor_received = time.monotonic()
        except ValueError:
            self.anchor_data = None

    def anchor_pose(self,msg):
        self.anchor_odom, self.odom_received = msg,time.monotonic()

    def anchor_base_pose(self,msg):
        self.anchor_base_odom,self.base_received=msg,time.monotonic()

    def anchor_navigation(self,nav):
        if self.anchor_mode == 'none': return True
        data, odom = self.anchor_data,self.anchor_odom
        base=self.anchor_base_odom
        now = rospy.Time.now().to_sec()
        if (not data or odom is None or base is None or not data.get('active')
                or time.monotonic()-self.anchor_received>.25 or time.monotonic()-self.odom_received>.25
                or time.monotonic()-self.base_received>.25 or not 0<=now-base.header.stamp.to_sec()<=.25
                or not 0 <= now-data['stamp'] <= .25
                or not 0 <= now-odom.header.stamp.to_sec() <= .25):
            self.anchor_last = None
            return False
        try:
            pose = odom.pose.pose
            quat=base.pose.pose.orientation
            rotation=quaternion_matrix([quat.x,quat.y,quat.z,quat.w])
            actual = np.array([pose.position.x,pose.position.y,pose.position.z])
            if not np.all(np.isfinite(actual)): raise ValueError('invalid CoG position')
            if self.anchor_model is None:
                self.anchor_model = ContactModel(rospy.get_param('/'+self.robot_name+'/robot_description'))
            link,point = ('inter_joint2',[0,0,0]) if self.anchor_mode=='mid' else ('link4',self.anchor_tail)
            positions = dict(data['robot'])
            if not self.anchor_model.required_joints.issubset(positions): raise ValueError('incomplete robot state')
            frames=self.anchor_model.transforms(positions)
            base_link=next((parent for _,kind,parent,child,_,_ in self.anchor_model.joints if child=='fc' and kind=='fixed'),None)
            if base_link is None:raise ValueError('a fixed FC mounting frame is required')
            # Use the physical baselink orientation and its fixed URDF FC
            # transform. A planning model's virtual CoG axes can differ from
            # the estimator CoG axes; multiplying those would rotate twice.
            rotation=rotation @ frames[base_link][:3,:3].T @ frames['fc'][:3,:3]
            if self.anchor_reference is None:
                relative = self.anchor_model.point_pose(positions,link,point)[:3,3]
                self.anchor_reference = actual + rotation @ relative
            dt = 0. if self.anchor_last is None else max(0.,min(now-self.anchor_last,1./self.rate_hz))
            velocity = np.array([nav.target_vel_x,nav.target_vel_y,nav.target_vel_z])
            reference = self.anchor_reference + velocity*dt
            positions.update(zip(DRAGON_JOINTS,data['q_command']))
            for i,angle in enumerate(data.get('evaluation_gimbals',[])):
                positions[f'gimbal{i//2+1}_{"roll" if i%2==0 else "pitch"}']=angle
            relative = self.anchor_model.point_pose(positions,link,point)[:3,3]
            command = anchor_position(reference,rotation,relative)
            if self.pos_limit and any(not low <= v <= high for v,(low,high) in
                                     zip(command,[self.LIMIT_X,self.LIMIT_Y,self.LIMIT_Z])):
                self.anchor_last = now
                return False
            self.anchor_reference,self.anchor_last = reference,now
            nav.target = nav.COG
            nav.pos_xy_nav_mode = nav.pos_z_nav_mode = nav.POS_MODE
            nav.target_pos_x,nav.target_pos_y,nav.target_pos_z = command.tolist()
            nav.target_vel_x = nav.target_vel_y = nav.target_vel_z = 0.
            return True
        except (ValueError,KeyError,np.linalg.LinAlgError) as error:
            rospy.logwarn_throttle(3.,'anchor command held: %s',str(error))
            self.anchor_last = None
            return False
