#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Publish RViz text markers for Dracomancer servo angles and joint switch ratio.

The displayed servo angles are computed directly from servo ticks, using the
same center-tick convention as convert_servo_to_joint_states.py. The joint
switch block mirrors control_joint_angle.py's roll-based pitch/yaw allocation
(joint1=wrist, joint2=elbow), republished on switch_diag_topic.

Timing note: this node runs on the GUI PC while its input topics are
published from the Khadas PC / control_joint_angle.py. Two pitfalls are
handled explicitly here:
- rospy never re-establishes a subscriber TCP link once it drops (e.g. a
  Wi-Fi hiccup between the two PCs), so a watchdog re-subscribes when a
  stream goes silent.
- /use_sim_time may be true on the shared master (DRAGON Gazebo publishes
  /clock), so all rate/staleness logic uses the wall clock instead of ROS
  time, which would freeze or scale with the simulation.
"""

import math
import time

import rospy
from geometry_msgs.msg import Point
from spinal.msg import ServoStates
from std_msgs.msg import ColorRGBA, Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class TopicWatch:
    """Subscribes to a topic and re-subscribes if silent for too long.

    rospy only connects a subscriber on a master publisherUpdate, so a link
    that dies silently (e.g. Wi-Fi drop) is never repaired without this.
    """

    def __init__(self, log_prefix, topic, msg_type, callback, reconnect_timeout):
        self.log_prefix = log_prefix
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback
        self.reconnect_timeout = reconnect_timeout
        self.last_msg_time = None
        self.last_subscribe_time = 0.0
        self.reconnect_announced = False
        self.sub = None
        self.subscribe()

    def subscribe(self):
        if self.sub is not None:
            self.sub.unregister()
        self.sub = rospy.Subscriber(self.topic, self.msg_type, self._on_msg, queue_size=1)
        self.last_subscribe_time = time.monotonic()

    def _on_msg(self, msg):
        self.last_msg_time = time.monotonic()
        self.reconnect_announced = False
        self.callback(msg)

    def age(self):
        if self.last_msg_time is None:
            return None
        return time.monotonic() - self.last_msg_time

    def watchdog(self):
        age = self.age()
        if age is not None and age <= self.reconnect_timeout:
            return
        if (time.monotonic() - self.last_subscribe_time) <= self.reconnect_timeout:
            return
        if age is not None and not self.reconnect_announced:
            rospy.logwarn("%s: no data on %s for %.1fs, re-subscribing",
                          self.log_prefix, self.topic, age)
            self.reconnect_announced = True
        self.subscribe()


class ServoLabels:
    def __init__(self):
        rospy.init_node("servo_labels")

        self.device_ns = rospy.get_param("~device_ns", "/dracomancer").rstrip("/")
        self.servo_topic = rospy.get_param("~servo_topic", self.device_ns + "/servo/states")
        self.switch_diag_topic = rospy.get_param(
            "~switch_diag_topic", self.device_ns + "/joint_map/switch_ratio")
        self.marker_topic = rospy.get_param("~marker_topic", self.device_ns + "/servo_angle_markers")
        self.frame_id = rospy.get_param("~frame_id", self.device_ns.lstrip("/") + "/base_link")
        self.status_rate_hz = float(rospy.get_param("~status_rate", 1.0))
        self.max_update_rate = float(rospy.get_param("~max_update_rate", 30.0))
        self.stale_timeout = float(rospy.get_param("~stale_timeout", 1.0))
        # No data for this long -> assume the TCP link died and re-subscribe.
        self.reconnect_timeout = float(rospy.get_param("~reconnect_timeout", 3.0))

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
        # [r1, rho1, c1_pitch, c1_yaw, r2, rho2, c2_pitch, c2_yaw]; joint1=wrist,
        # joint2=elbow. Matches control_joint_angle.py's switch_diag_topic order.
        self.latest_switch = None
        self.last_publish_time = 0.0

        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)
        self.servo_watch = TopicWatch(
            "servo_labels", self.servo_topic, ServoStates, self.servo_cb, self.reconnect_timeout)
        self.switch_watch = TopicWatch(
            "servo_labels", self.switch_diag_topic, Float64MultiArray, self.switch_cb,
            self.reconnect_timeout)

        rospy.loginfo("servo_labels: servo_topic=%s switch_diag_topic=%s marker_topic=%s frame=%s",
                      self.servo_topic, self.switch_diag_topic, self.marker_topic, self.frame_id)

    def servo_cb(self, msg):
        ticks = {}
        for servo in msg.servos:
            sid = int(servo.index)
            if sid in self.ordered_ids:
                ticks[sid] = float(servo.angle)
        self.latest_ticks = ticks
        self.publish_if_due()

    def switch_cb(self, msg):
        if len(msg.data) >= 8:
            self.latest_switch = list(msg.data[:8])
        self.publish_if_due()

    def tick_to_rad(self, tick):
        return (float(tick) - self.center_tick) * 2.0 * math.pi / self.ticks_per_rev

    def servo_lines(self):
        age = self.servo_watch.age()
        if age is None:
            return ["servo angles", "waiting for %s" % self.servo_topic]

        lines = ["servo angles (center tick %.0f)" % self.center_tick]
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
        return lines

    def switch_lines(self):
        lines = ["joint switch (rho: 0=pitch, 1=yaw)"]
        age = self.switch_watch.age()
        if self.latest_switch is None:
            lines.append("waiting for %s" % self.switch_diag_topic)
            return lines
        if age is not None and age > self.stale_timeout:
            lines.append("stale: %.1fs" % age)

        r1, rho1, c1_pitch, c1_yaw, r2, rho2, c2_pitch, c2_yaw = self.latest_switch
        lines.append("J1 wrist rho=%4.2f pitch=%4.2f yaw=%4.2f  r=%+6.1fdeg" % (
            rho1, c1_pitch, c1_yaw, math.degrees(r1)))
        lines.append("J2 elbow rho=%4.2f pitch=%4.2f yaw=%4.2f  r=%+6.1fdeg" % (
            rho2, c2_pitch, c2_yaw, math.degrees(r2)))
        return lines

    def marker_text(self):
        return "\n".join(self.servo_lines() + [""] + self.switch_lines())

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
        self.last_publish_time = time.monotonic()

    def publish_if_due(self):
        if self.max_update_rate > 0.0:
            min_period = 1.0 / self.max_update_rate
            if (time.monotonic() - self.last_publish_time) < min_period:
                return
        self.publish_marker()

    def watchdog(self):
        self.servo_watch.watchdog()
        self.switch_watch.watchdog()

        # Keep the waiting/stale status visible even when an input is absent.
        servo_age = self.servo_watch.age()
        switch_age = self.switch_watch.age()
        servo_stale = servo_age is None or servo_age > self.stale_timeout
        switch_stale = switch_age is None or switch_age > self.stale_timeout
        if servo_stale or switch_stale:
            self.publish_marker()

    def spin(self):
        # Drive the watchdog with the wall clock: rospy.Timer follows /clock
        # under use_sim_time and would freeze if the simulation pauses.
        interval = 1.0 / max(self.status_rate_hz, 0.5)
        while not rospy.is_shutdown():
            self.watchdog()
            time.sleep(interval)


if __name__ == "__main__":
    try:
        ServoLabels().spin()
    except rospy.ROSInterruptException:
        pass
