#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Part B: disturbance-rejection test for fc threshold validation.

At a fixed (hovering) shape, applies a step external wrench of increasing
magnitude to DRAGON's root link via Gazebo's /gazebo/apply_body_wrench and
records the resulting COG position / attitude error. The largest disturbance the
robot still rejects (bounded error) is the empirical control authority of that
shape; comparing it against the shape's fc_f_min / fc_t_min validates the
"fc == rejectable disturbance" interpretation and grounds the hard_min safety
factor.

This does NOT change fc (fc is shape-only); the disturbance is the independent
variable used to probe how much margin a given fc actually buys.

Run while DRAGON hovers. Record a rosbag alongside (see run_fc_experiment.sh).
"""

import csv
import os
import rospy
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, Vector3
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest


class DisturbanceTest:
    def __init__(self):
        rospy.init_node("disturbance_test")
        self.robot = rospy.get_param("~robot_name", "dragon")
        self.body = rospy.get_param("~body_name", "dragon::root")
        self.axis = rospy.get_param("~axis", "fx")          # fx/fy/fz/tx/ty/tz
        self.mags = rospy.get_param("~magnitudes", [0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0])
        self.duration = rospy.get_param("~duration", 2.0)   # wrench applied [s]
        self.recover = rospy.get_param("~recover", 3.0)     # recovery wait [s]
        self.err_limit = rospy.get_param("~error_limit", 0.5)  # m / rad: "lost control"
        out_default = os.path.expanduser("~/dracomancer_disturbance.csv")
        self.out_path = rospy.get_param("~out_path", out_default)

        self.fc_f = None
        self.fc_t = None
        self.pos0 = None
        self.max_err = 0.0

        rospy.Subscriber("/" + self.robot + "/debug/fc_f_min", Float64, lambda m: setattr(self, "fc_f", m.data))
        rospy.Subscriber("/" + self.robot + "/debug/fc_t_min", Float64, lambda m: setattr(self, "fc_t", m.data))
        rospy.Subscriber("/" + self.robot + "/uav/cog/odom", Odometry, self._odom_cb, queue_size=1)

        rospy.wait_for_service("/gazebo/apply_body_wrench")
        self.apply = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
        self.rows = []
        rospy.sleep(1.0)

    def _odom_cb(self, m):
        p = m.pose.pose.position
        if self.pos0 is None:
            return
        err = np.linalg.norm([p.x - self.pos0[0], p.y - self.pos0[1], p.z - self.pos0[2]])
        self.max_err = max(self.max_err, err)

    def _wrench(self, mag):
        w = Wrench()
        comp = {"fx": ("force", "x"), "fy": ("force", "y"), "fz": ("force", "z"),
                "tx": ("torque", "x"), "ty": ("torque", "y"), "tz": ("torque", "z")}[self.axis]
        setattr(getattr(w, comp[0]), comp[1], float(mag))
        return w

    def apply_step(self, mag):
        req = ApplyBodyWrenchRequest()
        req.body_name = self.body
        req.reference_frame = self.body
        req.wrench = self._wrench(mag)
        req.start_time = rospy.Time(0)
        req.duration = rospy.Duration(self.duration)
        self.apply(req)

    def run(self):
        for mag in self.mags:
            cur = rospy.wait_for_message("/" + self.robot + "/uav/cog/odom", Odometry)
            self.pos0 = [cur.pose.pose.position.x, cur.pose.pose.position.y, cur.pose.pose.position.z]
            self.max_err = 0.0
            fc_f, fc_t = self.fc_f, self.fc_t
            self.apply_step(mag)
            t0 = rospy.Time.now()
            while (rospy.Time.now() - t0).to_sec() < self.duration + self.recover and not rospy.is_shutdown():
                rospy.sleep(0.05)
            rejected = self.max_err < self.err_limit
            row = {"axis": self.axis, "magnitude": mag, "max_err": self.max_err,
                   "rejected": int(rejected), "fc_f_min": fc_f, "fc_t_min": fc_t}
            self.rows.append(row)
            rospy.loginfo("axis=%s mag=%.2f max_err=%.3f rejected=%s (fc_f=%.4f fc_t=%.4f)",
                          self.axis, mag, self.max_err, rejected, fc_f or -1, fc_t or -1)
            if not rejected:
                rospy.logwarn("control lost at mag=%.2f on %s; stopping ramp", mag, self.axis)
                break
        self.finish()

    def finish(self):
        if not self.rows:
            return
        with open(self.out_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            w.writeheader()
            for r in self.rows:
                w.writerow(r)
        rospy.loginfo("wrote %d rows to %s", len(self.rows), self.out_path)


if __name__ == "__main__":
    try:
        DisturbanceTest().run()
    except rospy.ROSInterruptException:
        pass
