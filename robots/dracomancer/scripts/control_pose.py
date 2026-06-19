#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, UInt8, Empty, String
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import Imu as SpinalImu


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


def quat_from_euler(roll, pitch, yaw):
    # intrinsic Rz(yaw) * Ry(pitch) * Rx(roll)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def rot_z(a, v):
    c, s = math.cos(a), math.sin(a)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])


def rot_y(a, v):
    c, s = math.cos(a), math.sin(a)
    return np.array([c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2]])


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


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
        rospy.init_node("control_pose")

        self.robot_name = rospy.get_param("~robot_name", "dragon")
        self.device_ns = rospy.get_param("~device_ns", "/dracomancer").rstrip("/")
        self.joy_topic = rospy.get_param("~joy_topic", "/dracomancer/joystick/calibrated")
        self.imu_topic = rospy.get_param("~imu_topic", "/dracomancer/imu")
        self.rate_hz = rospy.get_param("~rate", 40.0)
        self.wait_after_hover = rospy.get_param("~wait_after_hover", 3.0)

        self.teleop_mode = str(rospy.get_param("~teleop_mode", "startup")).lower()
        self.mode_topic = rospy.get_param("~mode_topic", self.device_ns + "/teleop_mode")
        self.valid_modes = ("startup", "precision", "wide")
        if self.teleop_mode not in self.valid_modes:
            rospy.logwarn("unknown teleop_mode '%s', fall back to 'startup'", self.teleop_mode)
            self.teleop_mode = "startup"

        # ----- navigation target: COG or BASELINK -----
        nav_target = str(rospy.get_param("~nav_target", "cog")).lower()
        self.nav_target = FlightNav.BASELINK if nav_target == "baselink" else FlightNav.COG

        # ----- IMU based relative direction control -----
        # one of: none / yaw / yaw_pitch / full
        self.direction_mode = str(rospy.get_param("~direction_mode", "yaw")).lower()
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
        self.flight_nav.yaw_nav_mode = FlightNav.POS_VEL_MODE
        self.flight_nav.roll_nav_mode = FlightNav.POS_VEL_MODE
        self.flight_nav.pitch_nav_mode = FlightNav.POS_VEL_MODE
        self.target_att_nav = Vector3Stamped()

        # Robot's Pose Limitation
        self.pos_limit = rospy.get_param("~pos_limit", True)
        self.att_limit = rospy.get_param("~att_limit", False)
        ## Position in Room-0066
        self.LIMIT_X = [-1.3, 2.0]
        self.LIMIT_Y = [-1.6, 1.6]
        self.LIMIT_Z = [0.3, 1.2]
        ## Attitude (not used for now)
        # self.LIMIT_R = [-0.5, 0.5]  # roll
        # self.LIMIT_P = [-0.5, 0.5]  # pitch
        # self.LIMIT_Y = [-1.0, 1.0]  # yaw

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
        self.att_pub = rospy.Publisher('/'+self.robot_name+'/final_target_baselink_rpy', Vector3Stamped, queue_size=1)

        # Subscribers
        self.robot_flight_state_sub = rospy.Subscriber('/'+self.robot_name+'/flight_state', UInt8, self.robot_flight_state_cb)
        self.robot_pos_sub = rospy.Subscriber('/'+self.robot_name+'/mocap/pose', PoseStamped, self.robot_pos_cb)
        self.js_calibrated_sub = rospy.Subscriber(self.joy_topic, Float32MultiArray, self.joystick_calib_cb, queue_size=1)
        self.imu_sub = rospy.Subscriber(self.imu_topic, SpinalImu, self.imu_cb, queue_size=1)
        self.recapture_sub = rospy.Subscriber("~recapture_neutral", Empty, self.recapture_cb, queue_size=1)
        self.mode_sub = rospy.Subscriber(self.mode_topic, String, self.mode_cb, queue_size=1)

        # Logger
        rospy.loginfo("robot_name: %s", self.robot_name)
        rospy.loginfo("teleop_mode: %s, mode_topic: %s", self.teleop_mode, self.mode_topic)
        rospy.loginfo("joy_topic: %s", self.joy_topic)
        rospy.loginfo("imu_topic: %s", self.imu_topic)
        rospy.loginfo("direction_mode: %s, nav_target: %s",
                      self.direction_mode, "baselink" if self.nav_target == FlightNav.BASELINK else "cog")
        rospy.loginfo("imu_mount_rpy: %s, recapture_on_hover: %s", mount_rpy, self.recapture_on_hover)

    def mode_cb(self, msg):
        mode = str(msg.data).strip().lower()
        if mode not in self.valid_modes:
            rospy.logwarn("ignore unknown teleop mode '%s'", mode)
            return
        if mode != self.teleop_mode:
            old_mode = self.teleop_mode
            rospy.loginfo("teleop mode: %s -> %s", self.teleop_mode, mode)
            self.teleop_mode = mode
            if mode == "wide":
                self.want_recapture = True
            elif old_mode == "wide":
                self.publish_zero_nav()
            self.latest_axes = None

    def robot_flight_state_cb(self, msg):
        # aerial_robot commonly uses HOVER_STATE=5 and LAND_STATE=6, but keep
        # this node permissive because enum values differ between branches.
        self.robot_hovering = int(msg.data) >= 4
        self.robot_landing = int(msg.data) == 6
        if not self.robot_hovering:
            self.wait_flag = False
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
        return (rospy.Time.now() - self.last_imu_stamp).to_sec() <= self.imu_timeout

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
        if idx == 0:
            v *= self.scale_x
        elif idx == 1:
            v *= self.scale_y
        elif idx == 2:
            v *= self.scale_z
        return 0.0 if abs(v) < self.deadzone else v

    def joystick_calib_cb(self, msg):
        self.latest_axes = list(msg.data)

    def rotate_command(self, body_vec):
        """Rotate the joystick body command into the world frame using the
        operator orientation, according to direction_mode."""
        if self.direction_mode == "none" or not self.imu_ready():
            return body_vec

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
        if self.latest_axes is None:
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

    def limit_pose(self, pos, att):
        # Limit position
        if self.pos_limit:
            pos[0] = max(self.LIMIT_X[0], min(pos[0], self.LIMIT_X[1]))
            pos[1] = max(self.LIMIT_Y[0], min(pos[1], self.LIMIT_Y[1]))
            pos[2] = max(self.LIMIT_Z[0], min(pos[2], self.LIMIT_Z[1]))

        # Limit attitude
        if self.att_limit:
            att[0] = max(self.LIMIT_R[0], min(att[0], self.LIMIT_R[1]))
            att[1] = max(self.LIMIT_P[0], min(att[1], self.LIMIT_P[1]))
            att[2] = max(self.LIMIT_Y[0], min(att[2], self.LIMIT_Y[1]))

        return pos, att

    def main(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            pose_active = self.teleop_mode == "wide" and self.robot_hovering and not self.robot_landing
            if pose_active:
                if not self.wait_flag:
                    rospy.sleep(self.wait_after_hover)
                    self.wait_flag = True

                self.make_nav_msg()
                self.nav_pub.publish(self.flight_nav)
            else:
                if self.pose_active_prev:
                    self.publish_zero_nav()
                self.latest_axes = None
                if self.teleop_mode != "wide":
                    self.wait_flag = False
            self.pose_active_prev = pose_active

            rate.sleep()

if __name__ == "__main__":
    try:
        node = ControlPose()
        node.main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))
