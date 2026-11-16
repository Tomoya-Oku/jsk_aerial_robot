#!/usr/bin/env python3
"""ROS adapter mixed into the existing single command owner."""
import json
import time
import xml.etree.ElementTree as ET

import numpy as np
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float64
from visualization_msgs.msg import Marker, MarkerArray

from shape_mapping import (UrdfModel, HumanChain, DragonChain, ShapeMapper,
                           DRAGON_JOINTS, vector)


class ShapeRuntime:
    def init_shape(self):
        self.shape_mapper = None
        self.shape_initialized = False
        self.shape_packet_valid = False
        self.shape_robot_valid = False
        self.shape_input_stamp = rospy.Time(0)
        self.shape_input_received = rospy.Time(0)
        self.shape_timeout = float(rospy.get_param("~shape_input_timeout", .5))
        if not np.isfinite(self.shape_timeout) or self.shape_timeout <= 0:
            raise ValueError("shape_input_timeout must be positive")
        self.shape_cached = None
        self.shape_solution = None
        self.shape_solve_stamp = None
        self.shape_validated_candidate = None
        self.shape_result = dict(ok=False, status="waiting")
        self.shape_human = None
        self.shape_marker_stamp = None
        self.shape_diagnostic_pub = rospy.Publisher(self.device_ns + "/shape/diagnostics", String, queue_size=1)
        self.shape_rmse_pub = rospy.Publisher(self.device_ns + "/shape/rmse", Float64, queue_size=1)
        self.shape_markers_pub = rospy.Publisher(self.device_ns + "/shape/markers", MarkerArray, queue_size=1)

    def shape_setup(self):
        if self.shape_mapper is not None:
            return True
        try:
            config = rospy.get_param("~shape")
            human = UrdfModel(rospy.get_param(rospy.get_param(
                "~human_description", "/dracomancer/robot_description")))
            dragon = UrdfModel(rospy.get_param("/" + self.robot_name + "/robot_description"))
            if self.joint_names != DRAGON_JOINTS:
                raise ValueError("shape mode requires URDF DRAGON joint order")
            chain = DragonChain(dragon, config["dragon_tail"])
            # The configured tail must match the straight physical link length.
            length = dragon.joints["joint1_pitch"]["origin"][0, 3]
            if not np.allclose(chain.tail, [length, 0, 0], atol=1e-8):
                raise ValueError("dragon_tail disagrees with loaded DRAGON model")
            self.shape_mapper = ShapeMapper(HumanChain(human, config), chain, config, self.startup_pose)
            self.shape_update_rate = float(config.get("update_rate", 10.))
            if not np.isfinite(self.shape_update_rate) or self.shape_update_rate <= 0:
                self.shape_mapper = None
                raise ValueError("shape update_rate must be positive")
            self.joint_velocity_limits = np.minimum(self.joint_velocity_limits, chain.velocity)
            return True
        except (KeyError, ValueError, TypeError, ET.ParseError, AttributeError) as error:
            self.shape_result = dict(ok=False, status="configuration: " + str(error))
            rospy.logwarn_throttle(3., "shape disabled: %s", str(error))
            return False

    @staticmethod
    def valid_joint_packet(msg):
        return (len(msg.name) == len(msg.position) and len(set(msg.name)) == len(msg.name)
                and bool(msg.name) and np.all(np.isfinite(msg.position)))

    def shape_input_ready(self):
        now = rospy.Time.now()
        age = (now - self.shape_input_stamp).to_sec()
        received_age = (now - self.shape_input_received).to_sec()
        if (not self.shape_packet_valid or self.shape_input_stamp == rospy.Time(0)
                or not 0 <= age <= self.shape_timeout or not 0 <= received_age <= self.shape_timeout):
            return False
        return self.shape_mapper.human.required.issubset(self.latest_device_joints)

    def shape_ready(self):
        if not self.shape_setup():
            return False
        if not self.shape_initialized:
            # Never start the limiter from a guessed zero/startup command.
            age = (rospy.Time.now() - self.latest_robot_joint_stamp).to_sec()
            if (not self.shape_robot_valid or not 0 <= age <= self.shape_timeout
                    or not self.shape_input_ready()):
                return False
            try:
                q = vector([self.latest_robot_joints[n] for n in self.joint_names], 6)
                if np.any(q < self.shape_mapper.dragon.lower) or np.any(q > self.shape_mapper.dragon.upper):
                    return False
                if self.enable_feasibility_gate:
                    # Let Gazebo's existing quick_init_pose finish before taking
                    # ownership; a guessed/unsafe startup must not pin its joints.
                    feasible, _, _, _ = self.evaluate_feasibility(q.tolist())
                    if feasible is not True:
                        self.shape_result = dict(ok=False, status="waiting for feasible measured pose")
                        return False
                self.current_target = q.tolist()
                self.last_feasible_target = q.tolist()
                self.shape_initialized = True
            except (KeyError, ValueError):
                return False
        return True

    def shape_target(self):
        if self.shape_cached is not None:
            return list(self.shape_cached)
        previous = list(self.current_target)
        start = time.perf_counter()
        try:
            if not self.shape_input_ready():
                raise ValueError("missing, malformed, stale or future JointState")
            robot_age = (rospy.Time.now()-self.latest_robot_joint_stamp).to_sec()
            if not self.shape_robot_valid or not 0 <= robot_age <= self.shape_timeout:
                raise ValueError("robot JointState unavailable")
            self.shape_human = self.shape_mapper.human_shape(dict(self.latest_device_joints))
            now = rospy.Time.now()
            elapsed = None if self.shape_solve_stamp is None else (now-self.shape_solve_stamp).to_sec()
            updated = self.shape_solution is None or elapsed is None or elapsed < 0 or elapsed >= 1./self.shape_update_rate
            if updated:
                result = self.shape_mapper.solve(self.shape_human, previous)
                result["seconds"] = time.perf_counter() - start
                if result["seconds"] > self.shape_mapper.budget:
                    result.update(ok=False, status="timeout", q=np.asarray(previous))
                    result["rmse"] = float(np.linalg.norm((self.shape_mapper.dragon_shape(previous)
                        - self.shape_human).ravel() * self.shape_mapper.sqrt_w))
                self.shape_solution = dict(result)
                self.shape_solve_stamp = now
            else:
                result = dict(self.shape_solution)
            result["updated"] = updated
            self.shape_result = result
            self.shape_cached = result["q"].tolist() if result["ok"] else previous
        except (ValueError, KeyError, np.linalg.LinAlgError) as error:
            self.shape_solution = None
            self.shape_result = dict(ok=False, status="hold: " + str(error), seconds=time.perf_counter()-start)
            self.shape_cached = previous
        return list(self.shape_cached)

    def shape_gated_target(self):
        candidate = self.mapped_target()
        self.publish_candidate_target(candidate)
        if not self.shape_result.get("ok", False):
            return list(self.current_target)
        if not self.enable_feasibility_gate:
            self.shape_result["gate"] = "disabled"
            return candidate
        now = rospy.Time.now()
        if self.feasibility_rate > 0 and (now-self.last_feasibility_eval_stamp).to_sec() < 1./self.feasibility_rate:
            # Reuse only this exact verified goal. Every actual incremental
            # command is still checked below, including on service failure.
            if (self.shape_validated_candidate is not None and
                    np.array_equal(candidate, self.shape_validated_candidate)):
                self.shape_result["gate"] = "cached"
                return candidate
            self.shape_result["gate"] = "waiting"
            return list(self.current_target)
        self.last_feasibility_eval_stamp = now
        feasible, force, torque, stable = self.evaluate_feasibility(candidate)
        self.publish_candidate_fc(force, torque)
        recovered = self.feasibility_recovered(force, torque, stable)
        accepted = feasible is True and (not self.feasibility_hysteresis_holding or recovered)
        self.feasibility_hysteresis_holding = not accepted
        self.shape_validated_candidate = list(candidate) if accepted else None
        self.shape_result["gate"] = "accepted" if accepted else "hold"
        self.log_gate(accepted, force, torque, stable)
        # Freeze at the last SENT command, not a previously accepted distant goal.
        return candidate if accepted else list(self.current_target)

    def shape_check_command(self, candidate):
        if (not self.enable_feasibility_gate or not self.teleop_mode
                or np.allclose(candidate, self.current_target, atol=1e-12, rtol=0)):
            return candidate
        feasible, force, torque, _ = self.evaluate_feasibility(candidate)
        if force is not None:
            self.target_fc_f_pub.publish(Float64(force))
        if torque is not None:
            self.target_fc_t_pub.publish(Float64(torque))
        if feasible is not True:
            self.shape_validated_candidate = None
            self.shape_result["gate"] = "command_hold"
            return list(self.current_target)
        return candidate

    def publish_shape_diagnostics(self):
        data = {k: v for k, v in self.shape_result.items() if k != "q"}
        data.update(q_command=list(self.current_target), stamp=rospy.Time.now().to_sec(),
                    teleop=self.teleop_mode)
        if self.shape_cached is not None:
            data["q_ref"] = list(self.shape_cached)
        if self.shape_mapper is not None and self.shape_human is not None and self.shape_initialized:
            robot = self.shape_mapper.dragon_shape(self.current_target)
            data["command_rmse"] = float(np.sqrt(np.sum(self.shape_mapper.weights * np.sum(
                (robot-self.shape_human)**2, axis=1))/self.shape_mapper.weights.sum()))
            self.shape_rmse_pub.publish(Float64(data["command_rmse"]))
            self.publish_shape_markers(robot)
        self.shape_diagnostic_pub.publish(String(json.dumps(data, allow_nan=False)))

    def publish_shape_markers(self, robot):
        # Overlay normalized polylines and samples alongside the physical models.
        now = rospy.Time.now()
        if self.shape_marker_stamp is not None and 0 <= (now-self.shape_marker_stamp).to_sec() < .1:
            return
        self.shape_marker_stamp = now
        markers = []
        human_points = self.shape_mapper.human.points(self.latest_device_joints) if self.shape_input_ready() else None
        series = [(self.shape_human, "world", [0, -2, 1.5], (0.1, .8, 1.)),
                  (robot, "world", [0, -2, 1.5], (1., .5, .1)),
                  (human_points, self.shape_mapper.human.base, [0, 0, 0], (.1, .8, 1.)),
                  (self.shape_mapper.dragon.points(self.current_target), self.robot_name + "/link4",
                   self.shape_mapper.dragon.tail, (1., .5, .1))]
        for index, (points, frame, offset, color) in enumerate(series):
            if points is None:
                continue
            for kind in (Marker.LINE_STRIP, Marker.SPHERE_LIST):
                marker = Marker()
                marker.header.frame_id, marker.header.stamp = frame, rospy.Time.now()
                marker.ns, marker.id = "shape_%d" % index, kind
                marker.type, marker.action = kind, Marker.ADD
                marker.pose.orientation.w = 1.
                marker.scale.x = .012 if kind == Marker.LINE_STRIP else .022
                marker.scale.y = marker.scale.z = marker.scale.x
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 1.
                marker.lifetime = rospy.Duration(.5)
                marker.points = [Point(*p) for p in np.asarray(points) + np.asarray(offset)]
                markers.append(marker)
        self.shape_markers_pub.publish(MarkerArray(markers))
