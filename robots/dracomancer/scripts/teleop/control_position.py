#!/usr/bin/env python3
"""DRAGON position and reference-point compensation."""
import json
import math
import time

import numpy as np
import rospy
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped, QuaternionStamped
from nav_msgs.msg import Odometry
from spinal.msg import Imu as SpinalImu
from std_msgs.msg import Bool, Empty, Float32MultiArray, String, UInt8

from control.contacts import ContactModel, anchor_position
from control.pose import matrix_rpy, quat_from_euler, quaternion_matrix
from control.shape import DRAGON_JOINTS, rpy_matrix, vector


# --------------------------------------------------------------------------
#  Quaternion / rotation helpers (numpy, [x, y, z, w] convention)
# --------------------------------------------------------------------------
def quat_normalize(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return np.array(q) / n


def quat_mult(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_inverse(q):
    # unit quaternion -> conjugate
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_rotate_vec(q, v):
    # rotate vector v by quaternion q
    qv = np.array([v[0], v[1], v[2], 0.0])
    return quat_mult(quat_mult(q, qv), quat_inverse(q))[:3]


def rot_z(a, v):
    c, s = math.cos(a), math.sin(a)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])


def rot_y(a, v):
    c, s = math.cos(a), math.sin(a)
    return np.array([c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2]])


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ViewControl:
    def __init__(self, operator_position, linear, maximum):
        self.operator = vector(operator_position, 3)
        self.linear = vector(linear, 2)
        self.maximum = self.linear.max() if maximum is None else float(maximum)
        if np.any(self.linear <= 0) or not np.isfinite(self.maximum) or self.maximum <= 0:
            raise ValueError('positive translation scales and speed limit required')
        self.previous_right = None

    def basis(self,position,tilt):
        z=self.operator-vector(position,3)
        if np.linalg.norm(z)<1e-6:raise ValueError('operator and robot positions coincide')
        z/=np.linalg.norm(z)
        right=np.cross([0.,0.,1.],z)
        if np.linalg.norm(right)<1e-3:
            seed=self.previous_right if self.previous_right is not None else np.array([1.,0.,0.])
            right=seed-z*np.dot(z,seed)
        if np.linalg.norm(right)<1e-9:raise ValueError('view basis unavailable')
        right/=np.linalg.norm(right)
        if self.previous_right is not None and np.dot(right,self.previous_right)<0:
            right=-right
        self.previous_right=right.copy()
        axes=np.column_stack([right,np.cross(z,right),z])
        tilt=np.asarray(tilt,dtype=float)
        if (tilt.shape!=(3,3) or not np.all(np.isfinite(tilt))
                or not np.allclose(tilt.T@tilt,np.eye(3),atol=1e-7) or np.linalg.det(tilt)<0):
            raise ValueError('invalid tilt rotation')
        return axes @ tilt

    def velocity(self, basis, axes):
        velocity = basis[:, :2] @ (self.linear * np.clip(axes, -1., 1.))
        speed = np.linalg.norm(velocity)
        if speed > self.maximum:
            velocity *= self.maximum / speed
        return velocity


class ControlPose:
    """Convert Dracomancer joystick (+ optional operator IMU heading) into a
    DRAGON FlightNav velocity command.

    direction_mode:
        none      : joystick axes are used directly as world-frame velocity
                    (legacy behaviour).
        yaw       : rotate the horizontal command by the operator heading
                    (rotation about world vertical) relative to a captured
                    neutral pose.
        yaw_pitch : additionally apply the operator pitch (lean) so that
                    leaning tilts the motion direction.  Roll is ignored.
        full      : apply the full relative 3D rotation of the operator body.
    """

    def __init__(self):
        rospy.init_node("control_position")

        self.robot_name = rospy.get_param("~robot_name", "dragon")
        self.device_ns = rospy.get_param("~device_ns", "/dracomancer").rstrip("/")
        self.joy_topic = rospy.get_param("~joy_topic", self.device_ns + "/right/joystick/calibrated")
        self.imu_topic = rospy.get_param("~imu_topic", "/dracomancer/imu")
        self.rate_hz = rospy.get_param("~rate", 40.0)
        self.wait_after_hover = rospy.get_param("~wait_after_hover", 3.0)
        self.joy_timeout = float(rospy.get_param("~joy_timeout", 0.0))
        self.last_joy_time = None

        self.enabled = self.parse_bool_param(rospy.get_param("~enabled", False), False)
        self.mode_topic = rospy.get_param("~mode_topic", self.device_ns + "/enabled")

        # ----- navigation target: COG or BASELINK -----
        nav_target = str(rospy.get_param("~nav_target", "cog")).lower()
        self.nav_target = FlightNav.BASELINK if nav_target == "baselink" else FlightNav.COG

        # ----- IMU based relative direction control -----
        # one of: none / yaw / yaw_pitch / full
        self.direction_mode = str(rospy.get_param("~direction_mode", "none")).lower()
        if self.direction_mode not in ("none", "yaw", "yaw_pitch", "full"):
            rospy.logwarn("unknown direction_mode '%s', fall back to 'none'", self.direction_mode)
            self.direction_mode = "none"
        # IMU mounting correction (operator-body <- IMU board), [rad]
        mount_rpy = rospy.get_param("~imu_mount_rpy", [0.0, -math.pi / 2.0, 0.0])
        self.q_mount = quat_from_euler(float(mount_rpy[0]), float(mount_rpy[1]), float(mount_rpy[2]))
        # recapture neutral heading when the robot starts hovering
        self.recapture_on_hover = rospy.get_param("~recapture_on_hover", True)
        self.imu_timeout = rospy.get_param("~imu_timeout", 0.5)

        # Msgs
        self.flight_nav = FlightNav()
        self.flight_nav.control_frame = FlightNav.WORLD_FRAME
        self.flight_nav.target = self.nav_target
        self.flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
        self.flight_nav.pos_z_nav_mode = FlightNav.POS_VEL_MODE
        self.flight_nav.yaw_nav_mode = FlightNav.NO_NAVIGATION
        self.flight_nav.roll_nav_mode = FlightNav.NO_NAVIGATION
        self.flight_nav.pitch_nav_mode = FlightNav.NO_NAVIGATION
        if rospy.get_param("~velocity_only", False):
            # VEL_MODE integrates velocity into the existing hover target in
            # FlightNavigator. POS_VEL_MODE would overwrite position with zero.
            self.flight_nav.pos_xy_nav_mode = FlightNav.VEL_MODE
            self.flight_nav.pos_z_nav_mode = FlightNav.VEL_MODE

        # Robot position limits
        self.pos_limit = rospy.get_param("~pos_limit", True)
        ## Position in Room-0066
        self.LIMIT_X = [-1.3, 2.0]
        self.LIMIT_Y = [-1.6, 1.6]
        self.LIMIT_Z = [0.3, 1.2]

        # Joystick
        ## Scaling for Joystick (negative means inverted)
        self.scale_x = rospy.get_param("~scale_x", 1.0)
        self.scale_y = rospy.get_param("~scale_y", 1.0)
        self.scale_z = rospy.get_param("~scale_z", 1.0)
        self.axis_x = rospy.get_param("~axis_x", 0)
        self.axis_y = rospy.get_param("~axis_y", 1)
        self.axis_z = rospy.get_param("~axis_z", 2)
        self.xy_vel = rospy.get_param("~xy_vel", 0.3)
        self.z_vel = rospy.get_param("~z_vel", 0.2)
        ## Deadzone for Joystick
        self.deadzone = rospy.get_param("~deadzone", 0.05)
        ## Buffer for latest joystick axes
        self.latest_axes = None
        self.warned_missing_z_axis = False
        self.robot_pose = None
        self.robot_hovering = False
        self.robot_landing = False
        self.wait_flag = False
        self.pose_active_prev = False

        # IMU state
        self.q_wo = None                # operator-body orientation in world
        self.last_imu_stamp = rospy.Time(0)
        self.neutral_q = None           # captured neutral operator orientation
        self.neutral_yaw = 0.0
        self.neutral_pitch = 0.0
        self.want_recapture = True      # capture neutral on first IMU msg
        self.prev_hovering = False

        # Publishers
        self.nav_pub = rospy.Publisher('/'+self.robot_name + "/uav/nav", FlightNav, queue_size=1)

        # Subscribers
        self.robot_flight_state_sub = rospy.Subscriber('/'+self.robot_name+'/flight_state', UInt8, self.robot_flight_state_cb)
        self.robot_pos_sub = rospy.Subscriber('/'+self.robot_name+'/mocap/pose', PoseStamped, self.robot_pos_cb)
        self.js_calibrated_sub = rospy.Subscriber(self.joy_topic, Float32MultiArray, self.joystick_calib_cb, queue_size=1)
        self.imu_sub = rospy.Subscriber(self.imu_topic, SpinalImu, self.imu_cb, queue_size=1)
        self.recapture_sub = rospy.Subscriber("~recapture_neutral", Empty, self.recapture_cb, queue_size=1)
        self.mode_sub = rospy.Subscriber(self.mode_topic, Bool, self.enabled_cb, queue_size=1)
        self.init_anchor()
        self.navigation_mode = rospy.get_param('~navigation_mode', 'legacy')
        if self.navigation_mode not in ('view', 'legacy'):
            raise ValueError('navigation_mode must be view or legacy')
        if self.navigation_mode == 'view':
            self.navigation_config = rospy.get_param('~navigation', {})
            operator = self.navigation_config.get('operator_position', [])
            self.view = self.make_view(operator) if operator else None
            self.operator_received = None
            self.view_tilt_alignment = rpy_matrix(
                self.navigation_config.get('tilt_alignment', [0., 0., 0.]))
            self.view_require_imu = self.navigation_config.get('require_imu', True)
            self.view_pub = rospy.Publisher(self.device_ns + '/view_frame',
                                            QuaternionStamped, queue_size=1)
            rospy.Subscriber(self.device_ns + '/operator_position', PoseStamped,
                             self.operator_position, queue_size=1)

        # Logger
        rospy.loginfo("robot_name: %s", self.robot_name)
        rospy.loginfo("enabled: %s, mode_topic: %s", self.enabled, self.mode_topic)
        rospy.loginfo("navigation_mode: %s", self.navigation_mode)
        rospy.loginfo("joy_topic: %s", self.joy_topic)
        rospy.loginfo("imu_topic: %s", self.imu_topic)
        rospy.loginfo("direction_mode: %s, nav_target: %s",
                      self.direction_mode, "baselink" if self.nav_target == FlightNav.BASELINK else "cog")
        rospy.loginfo("imu_mount_rpy: %s, recapture_on_hover: %s", mount_rpy, self.recapture_on_hover)

    @staticmethod
    def parse_bool_param(value, default=False):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        text = str(value).strip().lower()
        if text == "true":
            return True
        if text == "false":
            return False
        rospy.logwarn("invalid enabled value '%s', use %s", value, default)
        return default

    def enabled_cb(self, msg):
        enabled = bool(msg.data)
        if enabled != self.enabled:
            rospy.loginfo("enabled: %s -> %s", self.enabled, enabled)
            self.enabled = enabled
            if enabled:
                self.want_recapture = True
            else:
                self.publish_zero_nav()
            self.latest_axes = None

    def robot_flight_state_cb(self, msg):
        # Only HOVER permits joystick motion; landing must not accept motion.
        self.robot_hovering = int(msg.data) == 5
        self.robot_landing = int(msg.data) == 6
        if not self.robot_hovering:
            self.wait_flag = False
            self.anchor_reference = self.anchor_last = None
        # recapture the neutral heading at the moment hovering starts
        if self.recapture_on_hover and self.robot_hovering and not self.prev_hovering:
            self.want_recapture = True
        self.prev_hovering = self.robot_hovering

    def robot_pos_cb(self, msg):
        self.robot_pose = msg

    def recapture_cb(self, msg):
        self.want_recapture = True
        rospy.loginfo("neutral heading recapture requested")

    def imu_cb(self, msg):
        if len(msg.quaternion)!=4 or not np.all(np.isfinite(msg.quaternion)):return
        q = quat_normalize([msg.quaternion[0], msg.quaternion[1],
                            msg.quaternion[2], msg.quaternion[3]])
        # operator-body orientation in world = q_spinal (board->world) * q_mount
        self.q_wo = quat_mult(q, self.q_mount)
        self.last_imu_stamp = rospy.Time.now()
        if self.want_recapture:
            self.capture_neutral()
            self.want_recapture = False

    def imu_ready(self):
        if self.q_wo is None or self.neutral_q is None:
            return False
        return 0 <= (rospy.Time.now() - self.last_imu_stamp).to_sec() <= self.imu_timeout

    def make_view(self, operator):
        return ViewControl(operator, self.navigation_config.get('linear_scale', [.3, .3]),
                           self.navigation_config.get('max_linear_speed'))

    def operator_position(self, msg):
        if msg.header.frame_id.lstrip('/') != 'world' or msg.header.stamp == rospy.Time(0):
            return
        if not 0 <= (rospy.Time.now() - msg.header.stamp).to_sec() <= .5:
            return
        p = msg.pose.position
        value = np.array([p.x, p.y, p.z])
        if np.all(np.isfinite(value)):
            if self.view is None:
                self.view = self.make_view(value)
            else:
                self.view.operator = value
            self.operator_received = time.monotonic()

    def view_nav(self):
        nav = self.flight_nav
        nav.target_vel_x = nav.target_vel_y = nav.target_vel_z = 0.
        if not self.allow_joystick:
            return nav

        odom = self.anchor_odom
        now = rospy.Time.now()
        received = time.monotonic()
        if self.view is None:
            rospy.logwarn_throttle(3., 'view input held: set operator_position or provide a world-frame measurement')
            return nav
        if (odom is None or received - self.odom_received > .25
                or not 0 <= (now - odom.header.stamp).to_sec() <= .25
                or (self.operator_received is not None and received - self.operator_received > .5)):
            return nav

        tilt = np.eye(3)
        if self.imu_ready():
            relative = quaternion_matrix(quat_mult(quat_inverse(self.neutral_q), self.q_wo))
            tilt = self.view_tilt_alignment @ relative @ self.view_tilt_alignment.T
        elif self.view_require_imu:
            rospy.logwarn_throttle(3., 'view input held: fresh calibrated body IMU required')
            return nav

        p = odom.pose.pose.position
        try:
            basis = self.view.basis([p.x, p.y, p.z], tilt)
        except ValueError as error:
            rospy.logwarn_throttle(3., 'view input held: %s', str(error))
            return nav

        # Share the same world-frame axes with attitude, even without right-stick input.
        frame = QuaternionStamped()
        frame.header.stamp, frame.header.frame_id = now, 'world'
        q = quat_from_euler(*matrix_rpy(basis))
        frame.quaternion.x, frame.quaternion.y, frame.quaternion.z, frame.quaternion.w = q
        self.view_pub.publish(frame)

        if (self.latest_axes is not None and self.last_joy_time is not None
                and received - self.last_joy_time <= self.joy_timeout):
            axes = [self.get_axis(self.latest_axes, self.axis_x),
                    self.get_axis(self.latest_axes, self.axis_y)]
            velocity = self.view.velocity(basis, axes)
            if self.pos_limit:
                velocity = [self.limit_velocity_axis(value, speed, bounds)
                            for value, speed, bounds in
                            zip([p.x, p.y, p.z], velocity, [self.LIMIT_X, self.LIMIT_Y, self.LIMIT_Z])]
            nav.target_vel_x, nav.target_vel_y, nav.target_vel_z = velocity
        if self.anchor_mode == 'none':
            nav.pos_xy_nav_mode = nav.pos_z_nav_mode = FlightNav.VEL_MODE
        return nav

    def operator_yaw_pitch(self, q_wo):
        # operator forward axis (body x) expressed in world
        fwd = quat_rotate_vec(q_wo, [1.0, 0.0, 0.0])
        yaw = math.atan2(fwd[1], fwd[0])
        pitch = math.atan2(-fwd[2], math.hypot(fwd[0], fwd[1]))
        return yaw, pitch

    def capture_neutral(self):
        if self.q_wo is None:
            return
        self.neutral_q = np.array(self.q_wo)
        self.neutral_yaw, self.neutral_pitch = self.operator_yaw_pitch(self.q_wo)
        rospy.loginfo("captured neutral heading: yaw=%.3f pitch=%.3f",
                      self.neutral_yaw, self.neutral_pitch)

    def get_axis(self, data, idx):
        if idx < 0 or idx >= len(data):
            if idx == self.axis_z and not self.warned_missing_z_axis:
                rospy.logwarn("joystick z axis index %d is not available; z command is fixed to zero", idx)
                self.warned_missing_z_axis = True
            return 0.0
        v = float(data[idx])
        if not math.isfinite(v):
            return 0.0
        if idx == 0:
            v *= self.scale_x
        elif idx == 1:
            v *= self.scale_y
        elif idx == 2:
            v *= self.scale_z
        return 0.0 if abs(v) < self.deadzone else v

    def joystick_calib_cb(self, msg):
        self.latest_axes = list(msg.data)
        self.last_joy_time = time.monotonic()

    def rotate_command(self, body_vec):
        """Rotate the joystick body command into the world frame using the
        operator orientation, according to direction_mode."""
        if self.direction_mode == "none":
            return body_vec
        if not self.imu_ready():
            return np.zeros(3)

        if self.direction_mode == "yaw":
            yaw, _ = self.operator_yaw_pitch(self.q_wo)
            return rot_z(wrap_pi(yaw - self.neutral_yaw), body_vec)

        if self.direction_mode == "yaw_pitch":
            yaw, pitch = self.operator_yaw_pitch(self.q_wo)
            v = rot_y(pitch - self.neutral_pitch, body_vec)
            return rot_z(wrap_pi(yaw - self.neutral_yaw), v)

        # full: relative orientation in world since neutral
        q_rel = quat_mult(self.q_wo, quat_inverse(self.neutral_q))
        return quat_rotate_vec(q_rel, body_vec)

    def make_nav_msg(self):
        if self.navigation_mode=='view':return self.view_nav()
        if (not self.allow_joystick or self.latest_axes is None or
                (self.joy_timeout > 0 and (self.last_joy_time is None or
                 time.monotonic() - self.last_joy_time > self.joy_timeout))):
            self.flight_nav.target_vel_x = 0.0
            self.flight_nav.target_vel_y = 0.0
            self.flight_nav.target_vel_z = 0.0
            return self.flight_nav

        x_cmd = self.get_axis(self.latest_axes, self.axis_x)
        y_cmd = self.get_axis(self.latest_axes, self.axis_y)
        z_cmd = self.get_axis(self.latest_axes, self.axis_z)

        # body-frame velocity command from joystick
        body_vec = np.array([x_cmd * self.xy_vel,
                             y_cmd * self.xy_vel,
                             z_cmd * self.z_vel])

        world_vec = self.rotate_command(body_vec)
        world_vec = self.limit_velocity_by_position(world_vec)

        self.flight_nav.target_vel_x = float(world_vec[0])
        self.flight_nav.target_vel_y = float(world_vec[1])
        self.flight_nav.target_vel_z = float(world_vec[2])
        return self.flight_nav

    def publish_zero_nav(self):
        self.flight_nav.target_vel_x = 0.0
        self.flight_nav.target_vel_y = 0.0
        self.flight_nav.target_vel_z = 0.0
        self.nav_pub.publish(self.flight_nav)

    def limit_velocity_axis(self, pos, vel, bounds):
        if pos <= bounds[0] and vel < 0.0:
            return 0.0
        if pos >= bounds[1] and vel > 0.0:
            return 0.0
        return vel

    def limit_velocity_by_position(self, vel):
        if not self.pos_limit or self.robot_pose is None:
            return vel

        pos = self.robot_pose.pose.position
        limited = np.array(vel)
        limited[0] = self.limit_velocity_axis(pos.x, limited[0], self.LIMIT_X)
        limited[1] = self.limit_velocity_axis(pos.y, limited[1], self.LIMIT_Y)
        limited[2] = self.limit_velocity_axis(pos.z, limited[2], self.LIMIT_Z)
        return limited

    def main(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            pose_active = self.enabled and self.robot_hovering and not self.robot_landing
            if pose_active:
                if not self.wait_flag:
                    rospy.sleep(self.wait_after_hover)
                    if not self.enabled or not self.robot_hovering or self.robot_landing:
                        continue
                    self.wait_flag = True

                self.make_nav_msg()
                if self.anchor_navigation(self.flight_nav):
                    self.nav_pub.publish(self.flight_nav)
            else:
                self.anchor_reference = self.anchor_last = None
                if self.pose_active_prev:
                    self.publish_zero_nav()
                self.latest_axes = None
                if not self.enabled:
                    self.wait_flag = False
            self.pose_active_prev = pose_active

            rate.sleep()

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


if __name__ == "__main__":
    try:
        node = ControlPose()
        node.main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))
