#!/usr/bin/env python3
"""Mk-II CAD centerline retargeting, feasibility hold, and joint rate limiting."""
import time
from pathlib import Path
import sys
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, UInt8
from dracomancer.srv import ShapeFeasibility, ShapeFeasibilityRequest

# Catkin installs the entrypoint in lib/ and these helpers under share/scripts.
helper_dir = Path(__file__).resolve().parent
if not (helper_dir / 'shape_mapping.py').is_file():
    import rospkg
    helper_dir = Path(rospkg.RosPack().get_path('dracomancer')) / 'scripts/control'
sys.path.insert(0, str(helper_dir))
from shape_mapping import DRAGON_JOINTS, limit_step, vector
from shape_runtime import ShapeRuntime


class ControlJoints(ShapeRuntime):
    def __init__(self):
        rospy.init_node('control_joint_angle')
        self.robot_name = rospy.get_param('~robot_name', 'dragon')
        self.device_ns = rospy.get_param('~device_ns', '/dracomancer').rstrip('/')
        self.mapping_mode = rospy.get_param('~mapping_mode', 'shape')
        if self.mapping_mode != 'shape':
            raise ValueError('Mk-I mappings were removed; mapping_mode must be shape')
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
        self.teleop_mode = bool(rospy.get_param('~teleop_mode', False))
        self.publish_only_when_hovering = bool(rospy.get_param('~publish_only_when_hovering', True))
        self.enable_feasibility_gate = bool(rospy.get_param('~enable_feasibility_gate', True))
        self.feasibility_service_name = rospy.get_param('~feasibility_service', '/' + self.robot_name + '/shape_feasibility/check_shape')
        self.feasibility_service_timeout = float(rospy.get_param('~feasibility_service_timeout', 2.))
        self.feasibility_rate = float(rospy.get_param('~feasibility_rate', 10.))
        self.force_radius_threshold = float(rospy.get_param('~force_radius_threshold', .108990))
        self.torque_radius_threshold = float(rospy.get_param('~torque_radius_threshold', .015400))
        self.force_radius_recover_threshold = float(rospy.get_param('~force_radius_recover_threshold', .249220))
        self.torque_radius_recover_threshold = float(rospy.get_param('~torque_radius_recover_threshold', .278159))
        self.feasibility_hysteresis_holding = False
        self.last_feasibility_eval_stamp = rospy.Time(0)
        self.last_gate_log_stamp = rospy.Time(0)
        self.last_gate_feasible = None
        self.gate_log_period = float(rospy.get_param('~gate_log_period', 1.))
        self.feasibility_srv = None
        self.joints_ctrl_pub = rospy.Publisher(rospy.get_param('~command_topic', '/' + self.robot_name + '/joints_ctrl'), JointState, queue_size=1)
        self.shape_error_pub = rospy.Publisher(self.device_ns + '/shape_control_error', Float64MultiArray, queue_size=1)
        self.candidate_target_pub = rospy.Publisher(self.device_ns + '/candidate/joint_target', JointState, queue_size=1)
        self.candidate_fc_f_pub = rospy.Publisher(self.device_ns + '/candidate/fc_f_min', Float64, queue_size=1)
        self.candidate_fc_t_pub = rospy.Publisher(self.device_ns + '/candidate/fc_t_min', Float64, queue_size=1)
        self.target_fc_f_pub = rospy.Publisher(self.device_ns + '/target/fc_f_min', Float64, queue_size=1)
        self.target_fc_t_pub = rospy.Publisher(self.device_ns + '/target/fc_t_min', Float64, queue_size=1)
        self.init_shape()
        rospy.Subscriber(rospy.get_param('~device_joint_topic', self.device_ns + '/joint_states'), JointState, self.device_joint_cb, queue_size=1)
        rospy.Subscriber('/' + self.robot_name + '/joint_states', JointState, self.robot_joint_cb, queue_size=1)
        rospy.Subscriber('/' + self.robot_name + '/flight_state', UInt8, self.robot_flight_state_cb, queue_size=1)
        rospy.Subscriber(self.device_ns + '/teleop_mode', Bool, self.mode_cb, queue_size=1)
        rospy.Subscriber(self.device_ns + '/force_volume_radius_threshold', Float64MultiArray, self.force_threshold_cb, queue_size=1)
        rospy.Subscriber(self.device_ns + '/torque_volume_radius_threshold', Float64MultiArray, self.torque_threshold_cb, queue_size=1)

    def device_joint_cb(self, msg):
        self.shape_packet_valid = self.valid_joint_packet(msg)
        self.shape_input_stamp = msg.header.stamp
        self.shape_input_received = rospy.Time.now()
        if self.shape_packet_valid:
            self.latest_device_joints = dict(zip(msg.name, msg.position))

    def robot_joint_cb(self, msg):
        self.shape_robot_valid = self.valid_joint_packet(msg) and msg.header.stamp != rospy.Time(0)
        if self.shape_robot_valid:
            self.latest_robot_joints = dict(zip(msg.name, msg.position))
            self.latest_robot_joint_stamp = msg.header.stamp

    def mode_cb(self, msg):
        self.teleop_mode = bool(msg.data)
        self.shape_solution = self.shape_cached = None
        self.last_command_stamp = None

    def robot_flight_state_cb(self, msg):
        self.robot_hovering = int(msg.data) == self.hover_flight_state
        if not self.robot_hovering:
            self.last_command_stamp = None

    def mapped_target(self):
        return self.shape_target()

    def desired_target(self):
        # Clutch holds the last emitted command instead of moving to CAD zero.
        return self.shape_gated_target() if self.teleop_mode else list(self.current_target)

    def rate_limit(self, target):
        now = rospy.Time.now()
        dt = 0. if self.last_command_stamp is None else max(0., min((now-self.last_command_stamp).to_sec(), 1./self.rate_hz))
        return limit_step(self.current_target, target, self.joint_velocity_limits, dt, self.max_step).tolist()

    def make_joint_msg(self):
        start = time.perf_counter()
        self.shape_cached = None
        target = self.desired_target()
        desired = self.mapped_target() if self.teleop_mode else target
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

    def force_threshold_cb(self, msg):
        if len(msg.data) > 0:
            self.force_radius_threshold = float(msg.data[0])
        if len(msg.data) > 1:
            self.force_radius_recover_threshold = max(self.force_radius_threshold, float(msg.data[1]))

    def torque_threshold_cb(self, msg):
        if len(msg.data) > 0:
            self.torque_radius_threshold = float(msg.data[0])
        if len(msg.data) > 1:
            self.torque_radius_recover_threshold = max(self.torque_radius_threshold, float(msg.data[1]))

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
            res = self.feasibility_srv.call(req)
        except (rospy.ServiceException, TypeError):
            rospy.logwarn_throttle(5.0, "shape feasibility service call failed; reconnecting")
            self.feasibility_srv = None
            return None, None, None, None
        if not res.valid:
            return None, res.fc_f_min, res.fc_t_min, None
        stability_ok = bool(res.stability_ok)
        feasible = (stability_ok and
                    res.fc_f_min >= self.force_radius_threshold and
                    res.fc_t_min >= self.torque_radius_threshold)
        return feasible, res.fc_f_min, res.fc_t_min, stability_ok

    def feasibility_recovered(self, fc_f, fc_t, stability_ok):
        if fc_f is None or fc_t is None or not stability_ok:
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

if __name__ == '__main__':
    ControlJoints().main()
