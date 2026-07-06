#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Target-shape shadow visualizer for the shape-target reaching task.

Renders the trial's target shape q_star as a semi-transparent link chain
(MarkerArray) in RViz so the operator can overlay the current DRAGON shape on
it. The shadow is attached to <robot>/link4 by default: both the shadow and
the E_s metric live in the link4-root frame (link4 is the shoulder-
corresponding link held fixed in the world by control_joint_angle.py's
link4-anchor), so what the operator sees matches what is scored and what
actually stays still during real operation. Colors follow the task status:

  gray   : idle / no trial          cyan  : running
  yellow : within tolerance (holding) green : success   red : timeout/abort
"""

import json

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

import dragon_fk

COLORS = {
    "idle": (0.6, 0.6, 0.6),
    "running": (0.1, 0.7, 1.0),
    "holding": (1.0, 0.9, 0.1),
    "success": (0.2, 1.0, 0.2),
    "fail": (1.0, 0.2, 0.2),
}


class ShadowVisualizer:
    def __init__(self):
        rospy.init_node("shadow_visualizer")

        self.device_ns = rospy.get_param("~device_ns", "/dracomancer").rstrip("/")
        base = self.device_ns + "/shape_task"
        self.robot_ns = rospy.get_param("~robot_ns", "dragon")
        self.frame = rospy.get_param("~shadow_frame", self.robot_ns + "/link4")
        self.alpha = float(rospy.get_param("~shadow_alpha", 0.5))
        self.link_radius = float(rospy.get_param("~link_radius", 0.08))
        self.use_shape_error = bool(rospy.get_param("~use_shape_error", False))
        self.link_length = float(rospy.get_param("~link_length", dragon_fk.DEFAULT_LINK_LENGTH))
        self.inter_joint_x_offset = float(rospy.get_param(
            "~inter_joint_x_offset", dragon_fk.DEFAULT_INTER_JOINT_X_OFFSET))
        self.rate_hz = float(rospy.get_param("~rate", 5.0))
        self.joint_names = rospy.get_param("~dragon_joint_names", list(dragon_fk.JOINT_ORDER))

        self.q_star = None
        self.target_name = ""
        self.status = {}

        self.marker_pub = rospy.Publisher(base + "/shadow", MarkerArray, queue_size=1)
        rospy.Subscriber(base + "/target", JointState, self.target_cb, queue_size=1)
        rospy.Subscriber(base + "/status", String, self.status_cb, queue_size=1)

        rospy.loginfo("shadow visualizer: frame=%s, topic=%s/shadow", self.frame, base)
        rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 0.5)), self.publish)

    def target_cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        vals = [pos.get(n) for n in self.joint_names]
        if any(v is None for v in vals):
            rospy.logwarn("shadow: target message misses some joints; ignored")
            return
        self.q_star = [float(v) for v in vals]
        self.target_name = msg.header.frame_id

    def status_cb(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            pass

    def color_key(self):
        state = self.status.get("state", "idle")
        if state == "running":
            if self.status.get("hold", 0.0) > 0.0 or self.status.get("in_tolerance"):
                return "holding"
            return "running"
        if state in ("idle", "countdown"):
            # Right after a trial, briefly reflect the outcome via last_success.
            last = self.status.get("last_success")
            if last is True:
                return "success"
            if last is False:
                return "fail"
            return "idle"
        if state == "finished":
            return "success" if self.status.get("last_success") else "fail"
        return "idle"

    def publish(self, _event):
        if self.q_star is None:
            return
        pts = dragon_fk.fk_points(self.q_star, self.link_length, self.inter_joint_x_offset)
        r, g, b = COLORS[self.color_key()]
        color = ColorRGBA(r, g, b, self.alpha)
        stamp = rospy.Time.now()

        def base_marker(mid, mtype):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.frame
            m.ns = "shape_task_shadow"
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.color = color
            m.lifetime = rospy.Duration(2.0 / max(self.rate_hz, 0.5))
            return m

        links = base_marker(0, Marker.LINE_STRIP)
        links.scale.x = self.link_radius
        links.points = [Point(*p) for p in pts]

        joints = base_marker(1, Marker.SPHERE_LIST)
        joints.scale.x = joints.scale.y = joints.scale.z = self.link_radius * 1.5
        joints.points = [Point(*p) for p in pts]

        text = base_marker(2, Marker.TEXT_VIEW_FACING)
        text.scale.z = 0.15
        text.color = ColorRGBA(r, g, b, 1.0)
        tip = pts[-1]
        text.pose.position.x, text.pose.position.y = tip[0], tip[1]
        text.pose.position.z = tip[2] + 0.3
        e_q = self.status.get("E_q")
        label = self.target_name
        if isinstance(e_q, (int, float)):
            label += "\nE_q=%.3f rad" % e_q
        e_s = self.status.get("E_s")
        if self.use_shape_error and isinstance(e_s, (int, float)):
            label += "  E_s=%.3f" % e_s
        text.text = label

        self.marker_pub.publish(MarkerArray(markers=[links, joints, text]))


if __name__ == "__main__":
    try:
        ShadowVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
