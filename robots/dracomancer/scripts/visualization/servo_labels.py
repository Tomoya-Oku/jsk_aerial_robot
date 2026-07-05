#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Publish RViz text markers for Dracomancer servo angles.

The displayed angles are computed directly from servo ticks, using the same
center-tick convention as convert_servo_to_joint_states.py.
"""

import math

import rospy
from geometry_msgs.msg import Point
from spinal.msg import ServoStates
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class ServoLabels:
    def __init__(self):
        rospy.init_node("servo_labels")

        self.device_ns = rospy.get_param("~device_ns", "/dracomancer").rstrip("/")
        self.servo_topic = rospy.get_param("~servo_topic", self.device_ns + "/servo/states")
        self.marker_topic = rospy.get_param("~marker_topic", self.device_ns + "/servo_angle_markers")
        self.frame_id = rospy.get_param("~frame_id", self.device_ns.lstrip("/") + "/base_link")
        self.status_rate_hz = float(rospy.get_param("~status_rate", 1.0))
        self.max_update_rate = float(rospy.get_param("~max_update_rate", 30.0))
        self.stale_timeout = float(rospy.get_param("~stale_timeout", 1.0))

        self.center_tick = float(rospy.get_param("~center_tick", 2048.0))
        self.ticks_per_rev = float(rospy.get_param("~ticks_per_rev", 4096.0))
        self.font_size = float(rospy.get_param("~font_size", 0.08))
        self.position = rospy.get_param("~position", [-0.6, -0.45, 0.7])
        self.show_tick = bool(rospy.get_param("~show_tick", False))

        # ID order and labels mirror convert_servo_to_joint_states.py.
        self.ordered_ids = [int(x) for x in rospy.get_param("~ordered_ids", [0, 1, 2, 3, 4, 5, 6])]
        self.id_to_label = {
            0: "shoulder_abd",
            1: "shoulder_flex",
            2: "upper_arm_roll",
            3: "elbow_flex",
            4: "wrist_roll",
            5: "wrist_flex",
            6: "wrist_abd",
        }
        self.id_to_label.update({int(k): str(v) for k, v in rospy.get_param("~id_to_label", {}).items()})

        self.latest_ticks = {}
        self.last_stamp = None
        self.last_publish_stamp = rospy.Time(0)

        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)
        rospy.Subscriber(self.servo_topic, ServoStates, self.servo_cb, queue_size=1)

        rospy.loginfo("servo_labels: servo_topic=%s marker_topic=%s frame=%s",
                      self.servo_topic, self.marker_topic, self.frame_id)
        rospy.Timer(rospy.Duration(1.0 / max(self.status_rate_hz, 0.5)), self.publish_status)

    def servo_cb(self, msg):
        ticks = {}
        for servo in msg.servos:
            sid = int(servo.index)
            if sid in self.ordered_ids:
                ticks[sid] = float(servo.angle)
        self.latest_ticks = ticks
        self.last_stamp = rospy.Time.now()
        self.publish_if_due()

    def tick_to_rad(self, tick):
        return (float(tick) - self.center_tick) * 2.0 * math.pi / self.ticks_per_rev

    def marker_text(self):
        if self.last_stamp is None:
            return "servo angles\nwaiting for %s" % self.servo_topic

        lines = ["servo angles (center tick %.0f)" % self.center_tick]
        age = (rospy.Time.now() - self.last_stamp).to_sec()
        if age > self.stale_timeout:
            lines.append("stale: %.1fs" % age)

        for sid in self.ordered_ids:
            label = self.id_to_label.get(sid, "servo_%d" % sid)
            if sid not in self.latest_ticks:
                lines.append("ID%d %-15s --" % (sid, label))
                continue
            tick = self.latest_ticks[sid]
            rad = self.tick_to_rad(tick)
            deg = math.degrees(rad)
            suffix = " (%+.0f tick)" % (tick - self.center_tick) if self.show_tick else ""
            lines.append("ID%d %-15s %+6.1f deg / %+6.3f rad%s" % (
                sid, label, deg, rad, suffix))
        return "\n".join(lines)

    def make_marker(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = "servo_angle_labels"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position = Point(float(self.position[0]), float(self.position[1]), float(self.position[2]))
        marker.scale.z = self.font_size
        marker.color = ColorRGBA(0.95, 0.95, 0.95, 1.0)
        marker.lifetime = rospy.Duration(0.0)
        marker.text = self.marker_text()
        return marker

    def publish_marker(self):
        self.marker_pub.publish(MarkerArray(markers=[self.make_marker()]))
        self.last_publish_stamp = rospy.Time.now()

    def publish_if_due(self):
        now = rospy.Time.now()
        if self.max_update_rate > 0.0:
            min_period = 1.0 / self.max_update_rate
            if (now - self.last_publish_stamp).to_sec() < min_period:
                return
        self.publish_marker()

    def publish_status(self, _event):
        # Keep the waiting/stale status visible even when servo input is absent.
        if self.last_stamp is None:
            self.publish_marker()
            return
        age = (rospy.Time.now() - self.last_stamp).to_sec()
        if age > self.stale_timeout:
            self.publish_marker()


if __name__ == "__main__":
    try:
        ServoLabels()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
