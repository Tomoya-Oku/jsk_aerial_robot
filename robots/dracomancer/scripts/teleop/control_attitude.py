#!/usr/bin/env python3
"""Integrate left joystick input into a DRAGON CoG attitude target."""

import time
from threading import Lock

import numpy as np
import rospy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray, UInt8

from control.pose import matrix_rpy, quat_from_euler, quaternion_matrix
from control.shape import rotation, rpy_matrix, vector


class ControlAttitude:
    def __init__(self):
        rospy.init_node('control_attitude')
        robot = '/' + rospy.get_param('~robot_name', 'dragon')
        device = rospy.get_param('~device_ns', '/dracomancer').rstrip('/')
        self.enabled = rospy.get_param('~enabled', False)
        self.view_mode = rospy.get_param('~view_mode', False)
        self.rate_hz = float(rospy.get_param('~rate', 40.))
        self.joy_timeout = float(rospy.get_param('~joy_timeout', .1))
        self.deadzone = float(rospy.get_param('~deadzone', .05))
        self.wait_after_hover = float(rospy.get_param('~wait_after_hover', 3.))
        cfg = rospy.get_param('~navigation', {})
        self.scale = vector(cfg.get('angular_scale', [.3, .3]), 2)
        self.maximum = float(cfg.get('max_angular_speed', self.scale.max()))
        limits = np.asarray(cfg.get('attitude_limits', [[-.5, .5], [-.5, .5], [-np.pi, np.pi]]),
                            dtype=float)
        if (limits.shape != (3, 2) or not np.all(np.isfinite(limits))
                or np.any(limits[:, 0] >= limits[:, 1])):
            raise ValueError('invalid attitude limits')
        if (not np.isfinite(self.maximum) or self.maximum <= 0 or np.any(self.scale <= 0)
                or not 0 <= self.deadzone < 1):
            raise ValueError('invalid joystick attitude scales or deadzone')
        self.low, self.high = limits.T
        self.hovering = False
        self.ready_at = None
        self.joy = self.odom = self.view_frame = None
        self.last_step = self.target = None
        self.state_lock = Lock()

        self.publisher = rospy.Publisher(robot + '/target_rotation_motion', Odometry, queue_size=1)
        rospy.Subscriber(robot + '/flight_state', UInt8, self.flight_state, queue_size=1)
        rospy.Subscriber(robot + '/uav/cog/odom', Odometry, self.cog_odom, queue_size=1)
        rospy.Subscriber(device + '/left/joystick/calibrated', Float32MultiArray,
                         self.joystick, queue_size=1)
        rospy.Subscriber(device + '/enabled', Bool, self.set_enabled, queue_size=1)
        if self.view_mode:
            rospy.Subscriber(device + '/view_frame', QuaternionStamped, self.view_axes, queue_size=1)

    def set_enabled(self, msg):
        enabled = bool(msg.data)
        with self.state_lock:
            if enabled != self.enabled:
                self.enabled = enabled
                self.ready_at = None

    def flight_state(self, msg):
        hovering = msg.data == 5
        with self.state_lock:
            if hovering != self.hovering:
                self.hovering = hovering
                self.ready_at = None

    def joystick(self, msg):
        try:
            axes = np.clip(vector(msg.data, 2), -1., 1.)
        except ValueError:
            self.joy = None
            return
        axes[np.abs(axes) < self.deadzone] = 0.
        self.joy = axes, time.monotonic()

    def cog_odom(self, msg):
        self.odom = msg, time.monotonic()

    def view_axes(self, msg):
        if msg.header.frame_id == 'world':
            self.view_frame = msg, time.monotonic()

    @staticmethod
    def fresh(sample, now, received, timeout):
        if sample is None:
            return False
        msg, receipt = sample
        return (msg.header.stamp != rospy.Time(0) and received - receipt <= timeout
                and 0 <= (now - msg.header.stamp).to_sec() <= timeout)

    @staticmethod
    def orientation(q):
        return quaternion_matrix([q.x, q.y, q.z, q.w])

    def command(self, now, received):
        if not self.enabled or not self.hovering:
            self.ready_at = self.last_step = self.target = None
            return None
        if self.ready_at is None:
            self.ready_at = received + self.wait_after_hover
            self.last_step = self.target = None
        if received < self.ready_at:
            return None

        odom, joy, view_frame = self.odom, self.joy, self.view_frame
        if (not self.fresh(odom, now, received, .25) or joy is None
                or received - joy[1] > self.joy_timeout
                or (self.view_mode and not self.fresh(view_frame, now, received, self.joy_timeout))):
            self.last_step = None
            return None

        dt = 0. if self.last_step is None else max(0., min((now - self.last_step).to_sec(), 1. / self.rate_hz))
        self.last_step = now
        x, y = self.scale * joy[0]
        if not dt or (x == 0. and y == 0.):
            return None
        # A horizontal stick motion rotates about the input plane's up axis.
        basis = self.orientation(view_frame[0].quaternion) if self.view_mode else np.eye(3)
        omega = basis @ [-y, x, 0.]
        angle = min(np.linalg.norm(omega), self.maximum) * dt
        if self.target is None:
            self.target = self.orientation(odom[0].pose.pose.orientation)
        previous = matrix_rpy(self.target)
        angles = matrix_rpy(rotation(omega, angle) @ self.target)
        # A measured attitude outside the limits may move toward them without a jump.
        angles = np.clip(angles, np.minimum(self.low, previous), np.maximum(self.high, previous))
        self.target = rpy_matrix(angles)

        msg = Odometry()
        msg.header.stamp, msg.header.frame_id = now, 'cog'
        q = msg.pose.pose.orientation
        q.x, q.y, q.z, q.w = quat_from_euler(*angles)
        return msg

    def main(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            # Serialize publishing with enable and flight-state transitions.
            with self.state_lock:
                try:
                    msg = self.command(rospy.Time.now(), time.monotonic())
                except ValueError as error:
                    rospy.logwarn_throttle(3., 'attitude input held: %s', error)
                    msg = None
                if msg is not None:
                    self.publisher.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        ControlAttitude().main()
    except rospy.ROSInterruptException:
        pass
