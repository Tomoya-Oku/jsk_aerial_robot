#!/usr/bin/env python3
"""Mk-II shape commands, input validation, feasibility hold and rate limiting."""
import json
import time
import xml.etree.ElementTree as ET

import numpy as np
import rospy
from geometry_msgs.msg import Point, WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, String, UInt8
from visualization_msgs.msg import Marker, MarkerArray

from control.contacts import ContactInputs, ContactModel
from control.shape import DRAGON_JOINTS, DragonChain, UrdfModel, limit_step, make_mapper, vector
from dracomancer.srv import ShapeFeasibility, ShapeFeasibilityRequest


class ControlJoints:
    def __init__(self):
        rospy.init_node('control_joint_angle')
        self.robot_name = rospy.get_param('~robot_name', 'dragon')
        self.device_ns = rospy.get_param('~device_ns', '/dracomancer').rstrip('/')
        self.mapping_mode = rospy.get_param('~mapping_mode', 'shape')
        if self.mapping_mode not in ('shape', 'joint', 'dual_shape', 'dual_joint'):
            raise ValueError('Mk-I mappings were removed; use shape, joint, dual_shape, or dual_joint')
        if rospy.get_param('~feasibility_gate_mode', 'hold') != 'hold':
            raise ValueError('Mk-II shape supports only the hold feasibility gate')
        self.rate_hz = float(rospy.get_param('~rate', 40.0))
        self.max_step = float(rospy.get_param('~max_step', .015))
        velocity = rospy.get_param('~max_joint_velocity', .1)
        self.joint_velocity_limits = vector([velocity]*6 if np.isscalar(velocity) else velocity, 6)
        if (not np.isfinite(self.rate_hz) or self.rate_hz <= 0 or
                not np.isfinite(self.max_step) or self.max_step <= 0 or
                np.any(self.joint_velocity_limits <= 0)):
            raise ValueError('Control rate, step, and velocity limits must be positive')
        self.joint_names = rospy.get_param('~dragon_joint_names', DRAGON_JOINTS)
        self.startup_pose = vector(rospy.get_param('~startup_pose', [0, np.pi/2]*3), 6).tolist()
        self.current_target = list(self.startup_pose)
        self.last_feasible_target = list(self.startup_pose)
        self.last_command_stamp = None
        self.latest_device_joints, self.latest_robot_joints = {}, {}
        self.latest_robot_joint_stamp = rospy.Time(0)
        self.robot_hovering = False
        self.hover_flight_state = int(rospy.get_param('~hover_flight_state', 5))
        self.enabled = bool(rospy.get_param('~enabled', False))
        self.publish_only_when_hovering = bool(rospy.get_param('~publish_only_when_hovering', True))
        self.enable_feasibility_gate = bool(rospy.get_param('~enable_feasibility_gate', True))
        self.feasibility_service_name = rospy.get_param('~feasibility_service', '/' + self.robot_name + '/shape_feasibility/check_shape')
        self.feasibility_service_timeout = float(rospy.get_param('~feasibility_service_timeout', 2.))
        self.feasibility_rate = float(rospy.get_param('~feasibility_rate', 10.))
        self.force_radius_threshold = float(rospy.get_param('~force_radius_threshold', .108990))
        self.torque_radius_threshold = float(rospy.get_param('~torque_radius_threshold', .015400))
        self.force_radius_recover_threshold = float(rospy.get_param('~force_radius_recover_threshold', .249220))
        self.torque_radius_recover_threshold = float(rospy.get_param('~torque_radius_recover_threshold', .278159))
        self.valid_thresholds(self.force_radius_threshold, self.force_radius_recover_threshold)
        self.valid_thresholds(self.torque_radius_threshold, self.torque_radius_recover_threshold)
        self.feasibility_hysteresis_holding = False
        self.last_feasibility_eval_stamp = rospy.Time(0)
        self.last_gate_log_stamp = rospy.Time(0)
        self.last_gate_feasible = None
        self.gate_log_period = float(rospy.get_param('~gate_log_period', 1.))
        self.feasibility_srv = None
        self.feasibility_context = None
        self.use_contact_wrench = bool(rospy.get_param('~use_contact_wrench', False))
        self.fc_to_cog = None
        self.contacts=ContactInputs(rospy.get_param('~contacts',[{}]),self.robot_name)
        self.contact_geometry=None
        self.grasp_ok=False
        self.grasp_received=0.
        self.joints_ctrl_pub = rospy.Publisher(rospy.get_param('~command_topic', '/' + self.robot_name + '/joints_ctrl'), JointState, queue_size=1)
        self.shape_error_pub = rospy.Publisher(self.device_ns + '/shape_control_error', Float64MultiArray, queue_size=1)
        self.candidate_target_pub = rospy.Publisher(self.device_ns + '/candidate/joint_target', JointState, queue_size=1)
        self.candidate_fc_f_pub = rospy.Publisher(self.device_ns + '/candidate/fc_f_min', Float64, queue_size=1)
        self.candidate_fc_t_pub = rospy.Publisher(self.device_ns + '/candidate/fc_t_min', Float64, queue_size=1)
        self.target_fc_f_pub = rospy.Publisher(self.device_ns + '/target/fc_f_min', Float64, queue_size=1)
        self.target_fc_t_pub = rospy.Publisher(self.device_ns + '/target/fc_t_min', Float64, queue_size=1)
        self.init_shape()
        rospy.Subscriber(rospy.get_param('~device_joint_topic', self.device_ns + '/measured_joint_states'), JointState, self.device_joint_cb, queue_size=1)
        rospy.Subscriber('/' + self.robot_name + '/joint_states', JointState, self.robot_joint_cb, queue_size=1)
        rospy.Subscriber('/' + self.robot_name + '/flight_state', UInt8, self.robot_flight_state_cb, queue_size=1)
        rospy.Subscriber(self.device_ns + '/enabled', Bool, self.enabled_cb, queue_size=1)
        rospy.Subscriber(self.device_ns + '/force_volume_radius_threshold', Float64MultiArray, self.force_threshold_cb, queue_size=1)
        rospy.Subscriber(self.device_ns + '/torque_volume_radius_threshold', Float64MultiArray, self.torque_threshold_cb, queue_size=1)
        if self.use_contact_wrench:
            for i,spec in enumerate(self.contacts.specs):
                rospy.Subscriber(spec['topic'],WrenchStamped,lambda msg,index=i:self.contact_cb(msg,index),queue_size=1)
            if len(self.contacts.specs)>1:
                rospy.Subscriber('/'+self.robot_name+'/contact/constraints_ok',Bool,self.grasp_cb,queue_size=1)

    def contact_cb(self, msg, index=0):
        self.contacts.update(index,msg,time.monotonic())
        self.shape_validated_candidate = None

    def grasp_cb(self,msg):
        # Supplied by a contact-specific friction/joint-load validator. A
        # whole-body wrench balance alone cannot certify a two-point grasp.
        self.grasp_ok,self.grasp_received=bool(msg.data),time.monotonic()
        self.shape_validated_candidate=None

    def candidate_wrench(self, candidate):
        if not getattr(self,'use_contact_wrench',False):
            return [],[],[],[]
        if len(self.contacts.specs)>1 and (not self.grasp_ok or time.monotonic()-self.grasp_received>.25):
            raise ValueError('fresh friction/joint-load validation required for multiple contacts')
        values=self.contacts.values(rospy.Time.now().to_sec(),time.monotonic())
        if getattr(self,'contact_geometry',None) is None:
            self.contact_geometry=ContactModel(rospy.get_param('/'+self.robot_name+'/robot_description'))
        if not self.contact_geometry.required_joints.issubset(self.latest_robot_joints):
            raise ValueError('measured geometry required to transform contact wrenches')
        # Sensor link axes belong to the measured shape, not the hypothetical
        # q_ref. Convert first; the server then evaluates moment arms at q_ref.
        values=[(link,point,self.contact_geometry.wrench_in_fc(self.latest_robot_joints,np.asarray(wrench),link).tolist()
                 if local else wrench,False) for link,point,wrench,local in values]
        return ([v[0] for v in values],sum([v[1] for v in values],[]),
                sum([v[2] for v in values],[]),[v[3] for v in values])

    def device_joint_cb(self, msg):
        self.shape_packet_valid = self.valid_joint_packet(msg)
        self.shape_input_stamp = msg.header.stamp
        self.shape_input_received = rospy.Time.now()
        if self.shape_packet_valid:
            self.latest_device_joints = dict(zip(msg.name, msg.position))

    def robot_joint_cb(self, msg):
        self.shape_robot_valid = (self.valid_joint_packet(msg) and msg.header.stamp != rospy.Time(0)
                                  and set(self.joint_names).issubset(msg.name))
        if self.shape_robot_valid:
            self.latest_robot_joints = dict(zip(msg.name, msg.position))
            self.latest_robot_joint_stamp = msg.header.stamp

    def enabled_cb(self, msg):
        self.enabled = bool(msg.data)
        self.shape_solution = self.shape_cached = None
        self.last_command_stamp = None

    def robot_flight_state_cb(self, msg):
        self.robot_hovering = int(msg.data) == self.hover_flight_state
        # Ground demos keep publishing with the same rate limiter.
        if self.publish_only_when_hovering and not self.robot_hovering:
            self.last_command_stamp = None

    def mapped_target(self):
        return self.shape_target()

    def desired_target(self):
        # Clutch holds the last emitted command instead of moving to CAD zero.
        return self.shape_gated_target() if self.enabled else list(self.current_target)

    def rate_limit(self, target):
        now = rospy.Time.now()
        dt = 0. if self.last_command_stamp is None else max(0., min((now-self.last_command_stamp).to_sec(), 1./self.rate_hz))
        return limit_step(self.current_target, target, self.joint_velocity_limits, dt, self.max_step).tolist()

    def make_joint_msg(self):
        start = time.perf_counter()
        self.feasibility_context = None
        self.cycle_previous = list(self.current_target)
        self.shape_cached = None
        target = self.desired_target()
        desired = self.mapped_target() if self.enabled else target
        command = self.shape_check_command(self.rate_limit(target))
        self.current_target = self.last_feasible_target = list(command)
        self.shape_result['control_seconds'] = time.perf_counter()-start
        self.publish_shape_error(desired, command)
        self.publish_shape_diagnostics()
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name, msg.position = list(self.joint_names), list(command)
        return msg

    def can_publish_joint_command(self):
        if not self.shape_ready():
            self.publish_shape_diagnostics()
            return False
        return not self.publish_only_when_hovering or self.robot_hovering

    def main(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.can_publish_joint_command():
                self.joints_ctrl_pub.publish(self.make_joint_msg())
                self.last_command_stamp = rospy.Time.now()
            else:
                self.last_command_stamp = None
            rate.sleep()

    def connect_feasibility_service(self):
        try:
            rospy.wait_for_service(self.feasibility_service_name, timeout=self.feasibility_service_timeout)
            self.feasibility_srv = rospy.ServiceProxy(self.feasibility_service_name, ShapeFeasibility, persistent=True)
            rospy.loginfo("connected to shape feasibility service: %s", self.feasibility_service_name)
            return True
        except rospy.ROSException:
            rospy.logwarn_throttle(5.0, "shape feasibility service '%s' not available", self.feasibility_service_name)
            self.feasibility_srv = None
            return False

    @staticmethod
    def valid_thresholds(hard, recover):
        # Thesis eq:gate-conditions requires strict hysteresis, not equal limits.
        if not np.all(np.isfinite([hard, recover])) or not 0 <= hard < recover:
            raise ValueError('feasibility thresholds require 0 <= hard < recover')

    def update_thresholds(self, msg, kind):
        hard_name, recover_name = kind + '_radius_threshold', kind + '_radius_recover_threshold'
        hard = float(msg.data[0]) if msg.data else getattr(self, hard_name)
        recover = float(msg.data[1]) if len(msg.data) > 1 else getattr(self, recover_name)
        try:
            self.valid_thresholds(hard, recover)
        except ValueError as error:
            rospy.logwarn_throttle(3., '%s thresholds ignored: %s', kind, str(error))
            return
        setattr(self, hard_name, hard)
        setattr(self, recover_name, recover)
        # A cached goal was checked against the previous thresholds.
        self.shape_validated_candidate = None

    def force_threshold_cb(self, msg):
        self.update_thresholds(msg, 'force')

    def torque_threshold_cb(self, msg):
        self.update_thresholds(msg, 'torque')

    def evaluate_feasibility(self, candidate):
        # Returns (feasible, fc_f_min, fc_t_min, stability_ok). On service
        # failure returns (None, ...) so the caller can hold the last feasible
        # shape conservatively.
        if self.feasibility_srv is None:
            if not self.connect_feasibility_service():
                return None, None, None, None
        req = ShapeFeasibilityRequest()
        req.name = list(self.joint_names)
        req.position = list(candidate)
        try:
            req.contact_link,req.contact_point,req.contact_wrench,req.contact_local=self.candidate_wrench(candidate)
        except (ValueError, KeyError):
            return None, None, None, None
        context = getattr(self, 'feasibility_context', None)
        if context:
            req.evaluation_gimbals, req.evaluation_rotation = context
        try:
            started=time.monotonic()
            res = self.feasibility_srv.call(req)
        except (rospy.ServiceException, TypeError):
            rospy.logwarn_throttle(5.0, "shape feasibility service call failed; reconnecting")
            self.feasibility_srv = None
            return None, None, None, None
        if time.monotonic()-started>min(.25,getattr(self,'shape_timeout',.5)):
            return None,None,None,None
        if not res.valid or not np.all(np.isfinite([res.fc_f_min, res.fc_t_min])):
            return None, None, None, None
        gimbals, orientation = getattr(res, 'evaluation_gimbals', []), getattr(res, 'evaluation_rotation', [])
        if gimbals and len(orientation) == 9:
            self.feasibility_context = (list(gimbals), list(orientation))
        frame = getattr(res, 'fc_to_cog', [])
        if len(frame) == 9:
            self.fc_to_cog = list(frame)
        stability_ok = bool(res.stability_ok)
        feasible = (stability_ok and
                    res.fc_f_min >= self.force_radius_threshold and
                    res.fc_t_min >= self.torque_radius_threshold)
        return feasible, res.fc_f_min, res.fc_t_min, stability_ok

    def feasibility_recovered(self, fc_f, fc_t, stability_ok):
        if (fc_f is None or fc_t is None or not stability_ok or
                not np.all(np.isfinite([fc_f, fc_t]))):
            return False
        return (fc_f >= self.force_radius_recover_threshold and
                fc_t >= self.torque_radius_recover_threshold)

    def publish_candidate_fc(self, fc_f, fc_t):
        if fc_f is not None:
            self.candidate_fc_f_pub.publish(Float64(fc_f))
        if fc_t is not None:
            self.candidate_fc_t_pub.publish(Float64(fc_t))

    def publish_candidate_target(self, candidate):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = list(candidate)
        self.candidate_target_pub.publish(msg)

    def log_gate(self, feasible, fc_f, fc_t, stability_ok):
        if feasible != self.last_gate_feasible:
            self.last_gate_feasible = feasible
        now = rospy.Time.now()
        if self.gate_log_period > 0.0 and (now - self.last_gate_log_stamp).to_sec() < self.gate_log_period:
            return
        self.last_gate_log_stamp = now
        f = "n/a" if fc_f is None else "%.4f" % fc_f
        t = "n/a" if fc_t is None else "%.4f" % fc_t
        coupled = "n/a" if stability_ok is None else str(bool(stability_ok))
        msg = ("shape feasibility: deform=%s fc_f_min=%s fc_t_min=%s stability_ok=%s "
               "hard(force/torque)=%.4f/%.4f recover(force/torque)=%.4f/%.4f holding=%s") % (
            feasible, f, t, coupled, self.force_radius_threshold, self.torque_radius_threshold,
            self.force_radius_recover_threshold, self.torque_radius_recover_threshold,
            self.feasibility_hysteresis_holding)
        if feasible:
            rospy.loginfo(msg)
        else:
            rospy.logwarn(msg)

    def publish_shape_error(self, desired, target):
        msg = Float64MultiArray()
        msg.data = [float(d - t) for d, t in zip(desired, target)]
        self.shape_error_pub.publish(msg)

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
        self.shape_solution_human = None
        self.shape_solution_stamp = None
        self.shape_solution_previous = None
        self.shape_solve_stamp = None
        self.shape_validated_candidate = None
        self.shape_result = dict(ok=False, status="waiting")
        self.shape_human = None
        self.shape_marker_stamp = None
        self.dragon_marker_frame = rospy.get_param("~dragon_marker_frame", self.robot_name + "/link4")
        self.shape_marker_origin = vector(rospy.get_param("~shape_marker_origin", [0, -2, 1.5]), 3)
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
            self.shape_mapper = make_mapper(
                human, chain, config, self.startup_pose, self.mapping_mode,
                simulation=rospy.get_param('/use_sim_time', False))
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
            values = dict(self.latest_device_joints)
            input_stamp = self.shape_input_stamp.to_sec()
            self.shape_human = self.shape_mapper.human_shape(values)
            now = rospy.Time.now()
            elapsed = None if self.shape_solve_stamp is None else (now-self.shape_solve_stamp).to_sec()
            updated = self.shape_solution is None or elapsed is None or elapsed < 0 or elapsed >= 1./self.shape_update_rate
            if updated:
                result = self.shape_mapper.solve(self.shape_human, previous)
                result["seconds"] = time.perf_counter() - start
                if result["seconds"] > self.shape_mapper.budget:
                    result.update(ok=False, status="timeout", q=np.asarray(previous))
                    if self.shape_mapper.is_shape:
                        result["rmse"] = float(np.linalg.norm((self.shape_mapper.dragon_shape(previous)
                            - self.shape_human).ravel() * self.shape_mapper.sqrt_w))
                self.shape_solution = dict(result)
                self.shape_solution_human = values
                self.shape_solution_stamp = input_stamp
                self.shape_solution_previous = list(previous)
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
            self.feasibility_hysteresis_holding = True
            self.shape_validated_candidate = None
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
                self.feasibility_context = getattr(self, 'shape_validated_context', None)
                return candidate
            self.shape_result["gate"] = "waiting"
            self.feasibility_hysteresis_holding = True
            return list(self.current_target)
        self.last_feasibility_eval_stamp = now
        feasible, force, torque, stable = self.evaluate_feasibility(candidate)
        self.publish_candidate_fc(force, torque)
        recovered = self.feasibility_recovered(force, torque, stable)
        accepted = feasible is True and (not self.feasibility_hysteresis_holding or recovered)
        self.feasibility_hysteresis_holding = not accepted
        self.shape_validated_candidate = list(candidate) if accepted else None
        self.shape_validated_context = getattr(self, 'feasibility_context', None) if accepted else None
        self.shape_result["gate"] = "accepted" if accepted else "hold"
        self.log_gate(accepted, force, torque, stable)
        # Freeze at the last SENT command, not a previously accepted distant goal.
        return candidate if accepted else list(self.current_target)

    def shape_check_command(self, candidate):
        if not self.enable_feasibility_gate or not self.enabled:
            return candidate
        feasible, force, torque, _ = self.evaluate_feasibility(candidate)
        if force is not None:
            self.target_fc_f_pub.publish(Float64(force))
        if torque is not None:
            self.target_fc_t_pub.publish(Float64(torque))
        if feasible is not True:
            # Thesis eq:rechecked-command: h_k = 1 - a_k D_k. A rejected
            # rate-limited command also requires recovery on the next cycle.
            self.feasibility_hysteresis_holding = True
            self.shape_validated_candidate = None
            self.shape_result["gate"] = "command_hold"
            return list(self.current_target)
        return candidate

    def publish_shape_diagnostics(self):
        data = {k: v for k, v in self.shape_result.items() if k != "q"}
        data.update(q_command=list(self.current_target), stamp=rospy.Time.now().to_sec(),
                    teleop=self.enabled)
        # One immutable nominal packet for the feedback/anchor consumers.
        data.update(q_previous=self.shape_solution_previous or getattr(self, 'cycle_previous', list(self.current_target)),
                    human=self.shape_solution_human or dict(self.latest_device_joints), robot=dict(self.latest_robot_joints),
                    input_stamp=self.shape_solution_stamp or self.shape_input_stamp.to_sec())
        data['fc_to_cog'] = getattr(self, 'fc_to_cog', None)
        context=getattr(self,'feasibility_context',None)
        data['evaluation_gimbals']=context[0] if context else []
        data['active'] = (self.enabled and self.shape_initialized and self.shape_input_ready()
                          and self.shape_robot_valid
                          and 0 <= (rospy.Time.now()-self.latest_robot_joint_stamp).to_sec() <= self.shape_timeout
                          and (not self.publish_only_when_hovering or self.robot_hovering))
        if self.shape_cached is not None:
            data["q_ref"] = list(self.shape_cached)
        data['mapping_mode'] = self.mapping_mode
        if (self.shape_mapper is not None and self.shape_mapper.is_shape
                and self.shape_human is not None and self.shape_initialized):
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
        series = [(self.shape_human, "world", self.shape_marker_origin, (0.1, .8, 1.)),
                  (robot, "world", self.shape_marker_origin, (1., .5, .1)),
                  (human_points, self.shape_mapper.human.base, [0, 0, 0], (.1, .8, 1.)),
                  (self.shape_mapper.dragon.points(self.current_target), self.dragon_marker_frame,
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


if __name__ == '__main__':
    ControlJoints().main()
