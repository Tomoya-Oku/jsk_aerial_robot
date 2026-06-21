#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Low-pass filter for DRAGON's feasible control inradius (fc_f_min / fc_t_min).

The controlled robot publishes its *measured* feasible control inradius on
/<robot>/debug/fc_f_min and /<robot>/debug/fc_t_min. These signals are noisy,
which makes the live danger judgment in volume_radius_monitor jittery. This node
applies a first-order IIR (exponential) low-pass and republishes the filtered
values, so the safety logic can consume a smoother estimate.

Run this node in the controlled robot's namespace (ns="dragon") so the input /
output stay under /<robot>/debug/. The code lives in the dracomancer package but
publishes into the controlled robot's namespace, mirroring shape_feasibility_node.
"""

import math
import rospy
from std_msgs.msg import Float64


class FirstOrderLowPass:
    """Time-based first-order low-pass: y += alpha * (x - y),
    alpha = dt / (RC + dt), RC = 1 / (2*pi*cutoff_hz).

    Uses wall-clock dt between samples (the fc topics carry no header), so the
    cutoff is robust to the input rate. Resets to the raw value after a gap
    longer than reset_dt to avoid trusting stale state."""

    def __init__(self, cutoff_hz, reset_dt):
        self.rc = 1.0 / (2.0 * math.pi * cutoff_hz) if cutoff_hz > 0.0 else 0.0
        self.reset_dt = reset_dt
        self.value = None
        self.last_stamp = None

    def update(self, x, now):
        if self.value is None or self.last_stamp is None:
            self.value = x
            self.last_stamp = now
            return self.value
        dt = (now - self.last_stamp).to_sec()
        self.last_stamp = now
        if dt <= 0.0 or (self.reset_dt > 0.0 and dt > self.reset_dt) or self.rc <= 0.0:
            # Stale / out-of-order / passthrough: reset to the raw value.
            self.value = x
            return self.value
        alpha = dt / (self.rc + dt)
        self.value += alpha * (x - self.value)
        return self.value


class FcMinLowPass:
    def __init__(self):
        rospy.init_node("fc_min_lowpass")

        self.robot_name = rospy.get_param("~robot_name", "dragon")
        cutoff_hz = rospy.get_param("~cutoff_hz", 2.0)
        reset_dt = rospy.get_param("~reset_dt", 0.5)

        self.force_in_topic = rospy.get_param(
            "~force_inradius_topic", "/" + self.robot_name + "/debug/fc_f_min")
        self.torque_in_topic = rospy.get_param(
            "~torque_inradius_topic", "/" + self.robot_name + "/debug/fc_t_min")
        self.force_out_topic = rospy.get_param(
            "~force_inradius_filtered_topic", "/" + self.robot_name + "/debug/fc_f_min_filtered")
        self.torque_out_topic = rospy.get_param(
            "~torque_inradius_filtered_topic", "/" + self.robot_name + "/debug/fc_t_min_filtered")

        self.force_lpf = FirstOrderLowPass(cutoff_hz, reset_dt)
        self.torque_lpf = FirstOrderLowPass(cutoff_hz, reset_dt)

        self.force_pub = rospy.Publisher(self.force_out_topic, Float64, queue_size=1)
        self.torque_pub = rospy.Publisher(self.torque_out_topic, Float64, queue_size=1)

        rospy.Subscriber(self.force_in_topic, Float64, self.force_cb, queue_size=1)
        rospy.Subscriber(self.torque_in_topic, Float64, self.torque_cb, queue_size=1)

        rospy.loginfo("fc_min_lowpass: cutoff=%.2f Hz, reset_dt=%.2f s", cutoff_hz, reset_dt)
        rospy.loginfo("fc_min_lowpass: %s -> %s", self.force_in_topic, self.force_out_topic)
        rospy.loginfo("fc_min_lowpass: %s -> %s", self.torque_in_topic, self.torque_out_topic)

    def force_cb(self, msg):
        y = self.force_lpf.update(float(msg.data), rospy.Time.now())
        self.force_pub.publish(Float64(y))

    def torque_cb(self, msg):
        y = self.torque_lpf.update(float(msg.data), rospy.Time.now())
        self.torque_pub.publish(Float64(y))


if __name__ == "__main__":
    try:
        FcMinLowPass()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))
