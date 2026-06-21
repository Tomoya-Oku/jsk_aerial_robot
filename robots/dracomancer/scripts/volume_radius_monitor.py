#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Force/Torque volume radius monitor.

Republishes DRAGON's feasible control inradius (fc_f_min / fc_t_min) as
Dracomancer volume-radius topics, publishes the safety thresholds, accepts
runtime threshold updates, and computes the shape-safety scale used to limit
joint commands.

This runs as part of the device bringup (bringup.launch) so that the safety
monitoring keeps publishing even while teleoperation (position / orientation /
joint-angle control) is disabled, e.g. when DRAGON is not hovering.
"""

import rospy
from std_msgs.msg import Float64, Float64MultiArray


class VolumeRadiusMonitor:
    def __init__(self):
        rospy.init_node("volume_radius_monitor")

        self.robot_name = rospy.get_param("~robot_name", "dragon")
        self.device_ns = rospy.get_param("~device_ns", "/dracomancer").rstrip("/")
        self.rate_hz = rospy.get_param("~rate", 40.0)

        # Input: DRAGON feasible control inradius (force / torque).
        self.force_inradius_topic = rospy.get_param("~force_inradius_topic", "/" + self.robot_name + "/debug/fc_f_min")
        self.torque_inradius_topic = rospy.get_param("~torque_inradius_topic", "/" + self.robot_name + "/debug/fc_t_min")

        # Thresholds: [hard_min, min]. hard_min = danger, min = upper end of margin.
        self.force_inradius_min = rospy.get_param("~force_inradius_min", 0.2)
        self.torque_inradius_min = rospy.get_param("~torque_inradius_min", 0.02)
        self.force_inradius_hard_min = rospy.get_param("~force_inradius_hard_min", 0.1)
        self.torque_inradius_hard_min = rospy.get_param("~torque_inradius_hard_min", 0.01)

        self.inradius_timeout = rospy.get_param("~inradius_timeout", 0.5)
        self.enable_shape_safety = rospy.get_param("~enable_shape_safety", True)
        self.missing_inradius_scale = rospy.get_param("~missing_inradius_scale", 1.0)
        self.min_safety_scale = rospy.get_param("~min_safety_scale", 0.0)
        self.safety_log_period = rospy.get_param("~safety_log_period", 1.0)

        # Output topics.
        self.shape_safety_topic = rospy.get_param("~shape_safety_topic", self.device_ns + "/dragon_shape_safety")
        self.safety_scale_topic = rospy.get_param("~safety_scale_topic", self.device_ns + "/dragon_shape_safety_scale")
        self.force_volume_radius_topic = rospy.get_param(
            "~force_volume_radius_topic", self.device_ns + "/force_volume_radius")
        self.torque_volume_radius_topic = rospy.get_param(
            "~torque_volume_radius_topic", self.device_ns + "/torque_volume_radius")
        self.force_volume_radius_threshold_topic = rospy.get_param(
            "~force_volume_radius_threshold_topic", self.device_ns + "/force_volume_radius_threshold")
        self.torque_volume_radius_threshold_topic = rospy.get_param(
            "~torque_volume_radius_threshold_topic", self.device_ns + "/torque_volume_radius_threshold")
        self.force_volume_radius_threshold_cmd_topic = rospy.get_param(
            "~force_volume_radius_threshold_cmd_topic", self.device_ns + "/force_volume_radius_threshold_cmd")
        self.torque_volume_radius_threshold_cmd_topic = rospy.get_param(
            "~torque_volume_radius_threshold_cmd_topic", self.device_ns + "/torque_volume_radius_threshold_cmd")

        self.force_inradius = None
        self.torque_inradius = None
        self.last_inradius_stamp = rospy.Time(0)
        self.last_safety_state = None
        self.last_safety_log_stamp = rospy.Time(0)

        # Publishers.
        self.shape_safety_pub = rospy.Publisher(self.shape_safety_topic, Float64MultiArray, queue_size=1)
        self.safety_scale_pub = rospy.Publisher(self.safety_scale_topic, Float64, queue_size=1)
        self.force_volume_radius_pub = rospy.Publisher(self.force_volume_radius_topic, Float64, queue_size=1)
        self.torque_volume_radius_pub = rospy.Publisher(self.torque_volume_radius_topic, Float64, queue_size=1)
        self.force_volume_radius_threshold_pub = rospy.Publisher(
            self.force_volume_radius_threshold_topic, Float64MultiArray, queue_size=1)
        self.torque_volume_radius_threshold_pub = rospy.Publisher(
            self.torque_volume_radius_threshold_topic, Float64MultiArray, queue_size=1)

        # Subscribers.
        rospy.Subscriber(self.force_inradius_topic, Float64, self.force_inradius_cb, queue_size=1)
        rospy.Subscriber(self.torque_inradius_topic, Float64, self.torque_inradius_cb, queue_size=1)
        rospy.Subscriber(self.force_volume_radius_threshold_cmd_topic, Float64MultiArray,
                         self.force_volume_radius_threshold_cmd_cb, queue_size=1)
        rospy.Subscriber(self.torque_volume_radius_threshold_cmd_topic, Float64MultiArray,
                         self.torque_volume_radius_threshold_cmd_cb, queue_size=1)

        rospy.loginfo("volume_radius_monitor: robot=%s, inradius topics: %s, %s",
                      self.robot_name, self.force_inradius_topic, self.torque_inradius_topic)
        rospy.loginfo(
            "shape safety: enable=%s, missing_scale=%.3f, min_scale=%.3f, force=(%.3f/%.3f), torque=(%.3f/%.3f)",
            self.enable_shape_safety, self.missing_inradius_scale, self.min_safety_scale,
            self.force_inradius_hard_min, self.force_inradius_min,
            self.torque_inradius_hard_min, self.torque_inradius_min)
        rospy.loginfo("safety scale topic: %s", self.safety_scale_topic)

    def force_inradius_cb(self, msg):
        self.force_inradius = float(msg.data)
        self.last_inradius_stamp = rospy.Time.now()

    def torque_inradius_cb(self, msg):
        self.torque_inradius = float(msg.data)
        self.last_inradius_stamp = rospy.Time.now()

    @staticmethod
    def parse_threshold_cmd(msg, label):
        # Expect [hard_min, min]; hard_min must not exceed min.
        if len(msg.data) < 2:
            rospy.logwarn("ignore %s threshold cmd: need [hard_min, min], got %d value(s)",
                          label, len(msg.data))
            return None
        hard_min = float(msg.data[0])
        soft_min = float(msg.data[1])
        if hard_min > soft_min:
            rospy.logwarn("ignore %s threshold cmd: hard_min(%.4f) > min(%.4f)",
                          label, hard_min, soft_min)
            return None
        return hard_min, soft_min

    def force_volume_radius_threshold_cmd_cb(self, msg):
        parsed = self.parse_threshold_cmd(msg, "force")
        if parsed is None:
            return
        self.force_inradius_hard_min, self.force_inradius_min = parsed
        rospy.loginfo("force volume radius threshold updated: hard_min=%.4f min=%.4f",
                      self.force_inradius_hard_min, self.force_inradius_min)

    def torque_volume_radius_threshold_cmd_cb(self, msg):
        parsed = self.parse_threshold_cmd(msg, "torque")
        if parsed is None:
            return
        self.torque_inradius_hard_min, self.torque_inradius_min = parsed
        rospy.loginfo("torque volume radius threshold updated: hard_min=%.4f min=%.4f",
                      self.torque_inradius_hard_min, self.torque_inradius_min)

    def inradius_ready(self):
        if self.force_inradius is None or self.torque_inradius is None:
            return False
        return (rospy.Time.now() - self.last_inradius_stamp).to_sec() <= self.inradius_timeout

    def safety_scale(self):
        if not self.enable_shape_safety:
            return 1.0
        if not self.inradius_ready():
            return max(0.0, min(1.0, self.missing_inradius_scale))
        if (self.force_inradius <= self.force_inradius_hard_min or
                self.torque_inradius <= self.torque_inradius_hard_min):
            return max(0.0, min(1.0, self.min_safety_scale))
        force_margin = (self.force_inradius - self.force_inradius_hard_min) / max(
            self.force_inradius_min - self.force_inradius_hard_min, 1e-6)
        torque_margin = (self.torque_inradius - self.torque_inradius_hard_min) / max(
            self.torque_inradius_min - self.torque_inradius_hard_min, 1e-6)
        return max(self.min_safety_scale, min(1.0, force_margin, torque_margin))

    def safety_state(self, scale=None):
        if not self.enable_shape_safety:
            return "disabled"
        if not self.inradius_ready():
            return "missing_inradius"
        if (self.force_inradius <= self.force_inradius_hard_min or
                self.torque_inradius <= self.torque_inradius_hard_min):
            return "danger"
        if scale is None:
            scale = self.safety_scale()
        if scale < 1.0:
            return "warning"
        return "safe"

    def log_safety_state(self, scale, state, force_radius, torque_radius):
        message = (
            "shape_safety state=%s "
            "force_volume_radius=%.4f torque_volume_radius=%.4f safety_scale=%.3f "
            "thresholds force(hard/min)=%.4f/%.4f torque(hard/min)=%.4f/%.4f"
        ) % (
            state, force_radius, torque_radius, scale,
            self.force_inradius_hard_min, self.force_inradius_min,
            self.torque_inradius_hard_min, self.torque_inradius_min,
        )

        if state != self.last_safety_state:
            self.last_safety_state = state

        now = rospy.Time.now()
        if self.safety_log_period > 0.0:
            if (now - self.last_safety_log_stamp).to_sec() < self.safety_log_period:
                return
        self.last_safety_log_stamp = now

        if state in ("danger", "missing_inradius"):
            rospy.logwarn(message)
        else:
            rospy.loginfo(message)

    def publish(self):
        force_radius = float(self.force_inradius if self.force_inradius is not None else -1.0)
        torque_radius = float(self.torque_inradius if self.torque_inradius is not None else -1.0)
        scale = float(self.safety_scale())
        state = self.safety_state(scale)

        self.shape_safety_pub.publish(
            Float64MultiArray(data=[force_radius, torque_radius, scale]))
        self.safety_scale_pub.publish(Float64(scale))
        self.force_volume_radius_pub.publish(Float64(force_radius))
        self.torque_volume_radius_pub.publish(Float64(torque_radius))
        self.force_volume_radius_threshold_pub.publish(
            Float64MultiArray(data=[float(self.force_inradius_hard_min), float(self.force_inradius_min)]))
        self.torque_volume_radius_threshold_pub.publish(
            Float64MultiArray(data=[float(self.torque_inradius_hard_min), float(self.torque_inradius_min)]))
        self.log_safety_state(scale, state, force_radius, torque_radius)

    def main(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


if __name__ == "__main__":
    try:
        VolumeRadiusMonitor().main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))
