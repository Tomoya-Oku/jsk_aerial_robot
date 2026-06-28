#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray, UInt8, String
from sensor_msgs.msg import JointState
from dracomancer.srv import ShapeFeasibility, ShapeFeasibilityRequest

class ControlJoints:
    def __init__(self):
        rospy.init_node("control_joint_angle")

        # Parameters
        self.robot_name = rospy.get_param("~robot_name", "dragon")
        self.device_ns = rospy.get_param("~device_ns", "/dracomancer").rstrip("/")
        self.device_joint_topic = rospy.get_param("~device_joint_topic", "/dracomancer/joint_states")
        self.command_topic = rospy.get_param("~command_topic", "/" + self.robot_name + "/joints_ctrl")
        self.rate_hz = rospy.get_param("~rate", 40.0)
        self.valid_modes = ("startup", "teleoperation")
        self.teleop_mode = self.normalize_mode(rospy.get_param("~teleop_mode", "startup"))
        self.mode_topic = rospy.get_param("~mode_topic", self.device_ns + "/teleop_mode")
        if self.teleop_mode not in self.valid_modes:
            rospy.logwarn("unknown teleop_mode '%s', fall back to 'startup'", self.teleop_mode)
            self.teleop_mode = "startup"

        self.joint_names = rospy.get_param("~dragon_joint_names", [
            "joint1_pitch",
            "joint1_yaw",
            "joint2_pitch",
            "joint2_yaw",
            "joint3_pitch",
            "joint3_yaw",
        ])
        self.startup_pose = rospy.get_param("~startup_pose", [0.0, np.pi / 2.0, 0.0, np.pi / 2.0, 0.0, np.pi / 2.0])
        self.safe_pose = rospy.get_param("~safe_pose", self.startup_pose)

        # Mapping strategy: "elbow_only" (default, single-DOF: the elbow drives only
        # DRAGON's middle joint, every other joint held unchanged), "joint_pairing"
        # (medium-term), or "geometric" (long-term, FK + plane projection). See README.md.
        self.mapping_mode = str(rospy.get_param("~mapping_mode", "elbow_only")).lower()
        if self.mapping_mode not in ("joint_pairing", "geometric", "elbow_only"):
            rospy.logwarn("unknown mapping_mode '%s', fall back to 'joint_pairing'", self.mapping_mode)
            self.mapping_mode = "joint_pairing"
        # Reference (base) pose the mapping deforms around. Shared by every mapping_mode:
        #   straight : map around 0 rad (DRAGON straightens out).
        #   circular : map around startup_pose (DRAGON keeps its circular takeoff shape). Default.
        # The legacy name (joint_pairing_reference) and legacy values (zero/startup) are
        # still accepted for backward compatibility.
        mapping_reference = rospy.get_param(
            "~mapping_reference", rospy.get_param("~joint_pairing_reference", "circular"))
        self.mapping_reference = str(mapping_reference).strip().lower()
        legacy_reference = {"zero": "straight", "startup": "circular"}
        self.mapping_reference = legacy_reference.get(self.mapping_reference, self.mapping_reference)
        if self.mapping_reference not in ("straight", "circular"):
            rospy.logwarn("unknown mapping_reference '%s', fall back to 'straight'",
                          self.mapping_reference)
            self.mapping_reference = "straight"

        # --- joint_pairing (medium-term) mapping ---------------------------------
        # The three arm flexion joints share the same -X axis, so moving only them
        # keeps the arm in one plane. Route them to the three DRAGON yaw joints so the
        # operator's planar arm bend becomes DRAGON's in-plane (planar) shape; keep the
        # pitch joints at 0 (empty source name => constant = offset) so DRAGON stays
        # planar. This preserves plane-parallelism by construction (no offset hack).
        self.source_joint_names = rospy.get_param("~source_joint_names", [
            "",                                  # joint1_pitch (constant 0)
            "wrist_flexion_extension_joint",     # joint1_yaw
            "",                                  # joint2_pitch (constant 0)
            "elbow_flexion_extension_joint",     # joint2_yaw
            "",                                  # joint3_pitch (constant 0)
            "shoulder_flexion_extension_joint",  # joint3_yaw
        ])
        # Signs of the yaw entries set the serpentine curl direction (all -1 so that
        # the curl matches the geometric mode, keeping the two modes consistent). Flip
        # all three together on hardware if DRAGON curls the wrong way. Pitch entries
        # are unused (constant source).
        self.signs = rospy.get_param("~signs", [1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
        joint_pairing_scale = rospy.get_param("~joint_pairing_scale", 1.0)
        base_scales = rospy.get_param("~scales", [1.0] * len(self.joint_names))
        self.scales = [float(joint_pairing_scale) * float(scale) for scale in base_scales]
        default_offsets = ([0.0] * len(self.joint_names)
                           if self.mapping_reference == "straight"
                           else list(self.startup_pose))
        # offset[i] is the constant value when source is empty and the additive bias
        # otherwise. "circular" keeps mapped commands near DRAGON's circular shape.
        self.offsets = rospy.get_param("~offsets", default_offsets)

        # --- elbow_only mapping --------------------------------------------------
        # Map the single Dracomancer elbow flexion angle (relative to its neutral) to
        # DRAGON's middle yaw joint only; every other joint is held at its offset
        # (reference pose). Reuses signs/scales/offsets and mapping_reference, so
        # with mapping_reference=circular the rest of DRAGON stays circular and only the middle
        # joint bends. elbow_target_joint defaults to the middle yaw in joint_names.
        self.elbow_source_joint = rospy.get_param("~elbow_source_joint", "elbow_flexion_extension_joint")
        default_yaws = [n for n in self.joint_names if n.endswith("_yaw")]
        default_elbow_target = default_yaws[len(default_yaws) // 2] if default_yaws else ""
        self.elbow_target_joint = rospy.get_param("~elbow_target_joint", default_elbow_target)
        if self.mapping_mode == "elbow_only" and self.elbow_target_joint not in self.joint_names:
            rospy.logwarn("elbow_target_joint '%s' not in dragon_joint_names; "
                          "elbow_only will not move any joint", self.elbow_target_joint)

        # --- geometric (long-term) mapping ---------------------------------------
        # Arm kinematic chain (parent->child joint origins / axes) taken from
        # urdf/dracomancer.urdf (all joint rpy are 0). Used to forward-kinematics the
        # arm and decompose consecutive link directions into yaw (in-plane) / pitch
        # (out-of-plane) components about geom_plane_normal.
        self.geom_chain = rospy.get_param("~geom_chain", [
            ["shoulder_abduction_adduction_joint", [0.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            ["shoulder_flexion_extension_joint", [0.0925, 0.001, -0.1025], [-1.0, 0.0, 0.0]],
            ["upper_arm_external_internal_rotation_joint", [-0.0209, 0.0, -0.09475], [0.0, 0.0, -1.0]],
            ["elbow_flexion_extension_joint", [-0.00675, 0.0, -0.1209], [-1.0, 0.0, 0.0]],
            ["wrist_supination_joint", [-0.0209, 0.0, -0.09475], [0.0, 0.0, -1.0]],
            ["wrist_flexion_extension_joint", [-0.00675, 0.0, -0.1209], [-1.0, 0.0, 0.0]],
            ["wrist_abduction_adduction_joint", [-0.03915, 0.0209, -0.0225], [0.0, 1.0, 0.0]],
        ])
        # The arm's natural bending plane normal (flexion joints rotate about +-X), so
        # in-plane motion is the Y-Z plane. yaw = azimuth in that plane, pitch = tilt
        # toward the normal. Output sign lets you flip DRAGON's curl/tilt sense.
        self.geom_plane_normal = rospy.get_param("~geom_plane_normal", [1.0, 0.0, 0.0])
        self.geom_yaw_sign = rospy.get_param("~geom_yaw_sign", 1.0)
        self.geom_pitch_sign = rospy.get_param("~geom_pitch_sign", 1.0)
        self.geom_yaw_scale = rospy.get_param("~geom_yaw_scale", 1.0)
        self.geom_pitch_scale = rospy.get_param("~geom_pitch_scale", 1.0)
        self.joint_limit = rospy.get_param("~joint_limit", np.pi / 2.0)
        self.max_step = rospy.get_param("~max_step", 0.04)
        self.capture_neutral = rospy.get_param("~capture_neutral_on_first_msg", False)
        self.publish_only_when_hovering = rospy.get_param("~publish_only_when_hovering", True)
        self.publish_before_device_ready = rospy.get_param("~publish_before_device_ready", False)

        self.shape_error_topic = rospy.get_param("~shape_error_topic", self.device_ns + "/shape_control_error")
        # Predicted fc of the *candidate* (mapped) shape, republished from the
        # feasibility service response (teleoperation mode only). Useful for plotting
        # / recording; distinct from the controlled robot's measured fc.
        self.candidate_force_radius_topic = rospy.get_param(
            "~candidate_force_radius_topic", self.device_ns + "/candidate/fc_f_min")
        self.candidate_torque_radius_topic = rospy.get_param(
            "~candidate_torque_radius_topic", self.device_ns + "/candidate/fc_t_min")

        # Predictive shape-feasibility gate (teleoperation mode):
        #   candidate shape -> shape_feasibility service -> fc_f_min / fc_t_min.
        #   Deform only when BOTH radii are at or above their lower thresholds;
        #   otherwise hold the last feasible shape.
        self.enable_feasibility_gate = rospy.get_param("~enable_feasibility_gate", True)
        self.feasibility_gate_mode = str(rospy.get_param("~feasibility_gate_mode", "hold")).lower()
        if self.feasibility_gate_mode not in ("hold", "step_search", "soft_scale"):
            rospy.logwarn("unknown feasibility_gate_mode '%s', fall back to 'hold'",
                          self.feasibility_gate_mode)
            self.feasibility_gate_mode = "hold"
        self.feasibility_service_name = rospy.get_param(
            "~feasibility_service", "/" + self.robot_name + "/shape_feasibility/check_shape")
        self.feasibility_service_timeout = rospy.get_param("~feasibility_service_timeout", 2.0)
        # The model plugin runs a gimbal-planning optimization per call, so throttle
        # how often the candidate is re-evaluated (Hz). Between checks the last
        # feasible shape is held.
        self.feasibility_rate = rospy.get_param("~feasibility_rate", 20.0)
        # Lower thresholds (fallback params; overridden by the threshold topics below).
        self.force_radius_threshold = rospy.get_param("~force_radius_threshold", 0.1)
        self.torque_radius_threshold = rospy.get_param("~torque_radius_threshold", 0.01)
        self.feasibility_step_fraction = rospy.get_param("~feasibility_step_fraction", 0.25)
        self.feasibility_min_step_fraction = rospy.get_param("~feasibility_min_step_fraction", 0.03)
        self.feasibility_soft_min_scale = rospy.get_param("~feasibility_soft_min_scale", 0.0)
        # Threshold topics carry [hard_min, min]; hard_min ([0]) is used as the gate bound.
        self.force_threshold_topic = rospy.get_param(
            "~force_volume_radius_threshold_topic", self.device_ns + "/force_volume_radius_threshold")
        self.torque_threshold_topic = rospy.get_param(
            "~torque_volume_radius_threshold_topic", self.device_ns + "/torque_volume_radius_threshold")
        self.gate_log_period = rospy.get_param("~gate_log_period", 1.0)

        self.latest_device_joints = {}
        self.neutral_device_joints = {}
        self.current_target = list(self.safe_pose)
        self.last_feasible_target = list(self.safe_pose)
        self.robot_hovering = False
        self.last_gate_log_stamp = rospy.Time(0)
        self.last_gate_feasible = None
        self.last_feasibility_eval_stamp = rospy.Time(0)
        # Geometric-mode reference (neutral) relative angles, computed lazily so a
        # captured neutral pose can be used if available.
        self.geom_ref = None

        # Publisher
        self.joints_ctrl_pub = rospy.Publisher(self.command_topic, JointState, queue_size=10)
        self.shape_error_pub = rospy.Publisher(self.shape_error_topic, Float64MultiArray, queue_size=1)
        self.candidate_fc_f_pub = rospy.Publisher(self.candidate_force_radius_topic, Float64, queue_size=1)
        self.candidate_fc_t_pub = rospy.Publisher(self.candidate_torque_radius_topic, Float64, queue_size=1)

        # Service client (persistent for rate; reconnected on failure)
        self.feasibility_srv = None
        if self.enable_feasibility_gate:
            self.connect_feasibility_service()

        # Subscriber
        self.device_joint_sub = rospy.Subscriber(self.device_joint_topic, JointState, self.device_joint_cb, queue_size=1)
        self.robot_flight_state_sub = rospy.Subscriber('/' + self.robot_name + '/flight_state', UInt8, self.robot_flight_state_cb, queue_size=1)
        self.mode_sub = rospy.Subscriber(self.mode_topic, String, self.mode_cb, queue_size=1)
        self.force_threshold_sub = rospy.Subscriber(self.force_threshold_topic, Float64MultiArray, self.force_threshold_cb, queue_size=1)
        self.torque_threshold_sub = rospy.Subscriber(self.torque_threshold_topic, Float64MultiArray, self.torque_threshold_cb, queue_size=1)

        rospy.loginfo("teleop_mode: %s, mode_topic: %s", self.teleop_mode, self.mode_topic)
        rospy.loginfo("device_joint_topic: %s", self.device_joint_topic)
        rospy.loginfo("command_topic: %s", self.command_topic)
        rospy.loginfo("mapping_mode: %s, mapping_reference: %s",
                      self.mapping_mode, self.mapping_reference)
        rospy.loginfo("joint mapping: %s",
                      ", ".join("{}<-{}".format(dst, src)
                                for dst, src in zip(self.joint_names, self.source_joint_names)))
        rospy.loginfo("joint mapping scale/sign/offset: %s",
                      ", ".join("{}:{:.3f}/{:.3f}/{:.3f}".format(
                          name, scale, sign, offset)
                                for name, scale, sign, offset in zip(
                                    self.joint_names, self.scales, self.signs, self.offsets)))
        rospy.loginfo("feasibility gate: enable=%s, mode=%s, service=%s, thresholds force/torque=%.4f/%.4f",
                      self.enable_feasibility_gate, self.feasibility_gate_mode, self.feasibility_service_name,
                      self.force_radius_threshold, self.torque_radius_threshold)
        rospy.loginfo("joint command gating: only_when_hovering=%s, before_device_ready=%s",
                      self.publish_only_when_hovering, self.publish_before_device_ready)

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
    def normalize_mode(mode):
        # "teleop" is accepted as a shorthand alias for "teleoperation".
        mode = str(mode).strip().lower()
        return "teleoperation" if mode == "teleop" else mode

    def mode_cb(self, msg):
        mode = self.normalize_mode(msg.data)
        if mode not in self.valid_modes:
            rospy.logwarn("ignore unknown teleop mode '%s'", mode)
            return
        if mode != self.teleop_mode:
            rospy.loginfo("teleop mode: %s -> %s", self.teleop_mode, mode)
            self.teleop_mode = mode

    def clamp(self, x):
        return max(-self.joint_limit, min(self.joint_limit, x))

    def clamp_target(self, target):
        return [self.clamp(v) for v in target]

    def device_joint_cb(self, msg):
        self.latest_device_joints = {
            name: float(pos) for name, pos in zip(msg.name, msg.position)
        }
        if self.capture_neutral and not self.neutral_device_joints:
            # Capture the joints actually used as mapping sources. In joint_pairing the
            # empty (constant) entries are skipped; in geometric every chain joint is
            # used; in elbow_only only the elbow source is needed.
            if self.mapping_mode == "geometric":
                needed = [j[0] for j in self.geom_chain]
            elif self.mapping_mode == "elbow_only":
                needed = [self.elbow_source_joint]
            else:
                needed = [n for n in self.source_joint_names if n]
            if needed and all(name in self.latest_device_joints for name in needed):
                self.neutral_device_joints = {
                    name: self.latest_device_joints[name] for name in needed
                }
                rospy.loginfo("Captured dracomancer neutral joints for DRAGON mapping")

    def robot_flight_state_cb(self, msg):
        self.robot_hovering = int(msg.data) >= 4

    def force_threshold_cb(self, msg):
        if len(msg.data) > 0:
            self.force_radius_threshold = float(msg.data[0])

    def torque_threshold_cb(self, msg):
        if len(msg.data) > 0:
            self.torque_radius_threshold = float(msg.data[0])

    def mapped_target(self):
        if not self.latest_device_joints:
            return list(self.last_feasible_target)
        if self.mapping_mode == "geometric":
            return self.geometric_target()
        if self.mapping_mode == "elbow_only":
            return self.elbow_only_target()
        return self.joint_pairing_target()

    def joint_pairing_target(self):
        # Medium-term: per-DRAGON-joint 1:1 mapping with sign/scale/offset.
        # An empty source name means a constant joint (value = offset), used to keep
        # the pitch joints at 0 so DRAGON stays planar.
        target = []
        for i, source_name in enumerate(self.source_joint_names):
            if not source_name:
                target.append(self.clamp(self.offsets[i]))
                continue
            source = self.latest_device_joints.get(source_name)
            if source is None:
                target.append(self.last_feasible_target[i])
                continue
            neutral = self.neutral_device_joints.get(source_name, 0.0)
            mapped = self.offsets[i] + self.signs[i] * self.scales[i] * (source - neutral)
            target.append(self.clamp(mapped))
        return target

    def elbow_only_target(self):
        # Single-DOF: the elbow flexion angle (delta from neutral) drives only DRAGON's
        # middle yaw joint (elbow_target_joint). Every other joint is held at its last
        # commanded (feasible) value, so elbow_only never moves any joint but the middle
        # one (DRAGON keeps the shape it took off / was last left in).
        target = list(self.last_feasible_target)
        elbow = self.latest_device_joints.get(self.elbow_source_joint)
        if elbow is None:
            return target
        neutral = self.neutral_device_joints.get(self.elbow_source_joint, 0.0)
        delta = elbow - neutral
        for i, name in enumerate(self.joint_names):
            if name == self.elbow_target_joint:
                mapped = self.offsets[i] + self.signs[i] * self.scales[i] * delta
                target[i] = self.clamp(mapped)
        return target

    # --- geometric (long-term) mapping --------------------------------------
    @staticmethod
    def rot_axis_angle(axis, angle):
        # Rodrigues' rotation matrix for a unit (or near-unit) axis.
        a = np.asarray(axis, dtype=float)
        n = np.linalg.norm(a)
        if n < 1e-9:
            return np.eye(3)
        a = a / n
        c, s = np.cos(angle), np.sin(angle)
        x, y, z = a
        return np.array([
            [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
            [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
            [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
        ])

    def arm_node_positions(self, joints):
        # Forward kinematics over geom_chain (all joint rpy are 0): returns the base-
        # frame positions of every joint origin plus the final hand point.
        T = np.eye(4)
        positions = []
        for name, origin, axis in self.geom_chain:
            trans = np.eye(4)
            trans[:3, 3] = np.asarray(origin, dtype=float)
            T = T.dot(trans)
            positions.append(T[:3, 3].copy())  # joint location (before its own rotation)
            rot = np.eye(4)
            rot[:3, :3] = self.rot_axis_angle(axis, float(joints.get(name, 0.0)))
            T = T.dot(rot)
        positions.append(T[:3, 3].copy())  # hand tip
        return positions

    def segment_angles(self, joints):
        # Decompose the upper-arm / forearm / hand segment directions into azimuth
        # (in-plane, -> yaw) and elevation (toward plane normal, -> pitch).
        pos = self.arm_node_positions(joints)
        # node indices in geom_chain: 1=shoulder_flexion, 3=elbow, 5=wrist, 7=hand tip
        shoulder, elbow, wrist, hand = pos[1], pos[3], pos[5], pos[7]
        segments = [elbow - shoulder, wrist - elbow, hand - wrist]  # upper arm, forearm, hand

        n = np.asarray(self.geom_plane_normal, dtype=float)
        n = n / max(np.linalg.norm(n), 1e-9)
        # Build an in-plane orthonormal basis (e1, e2) spanning the plane.
        ref = np.array([0.0, 0.0, 1.0]) if abs(n[2]) < 0.9 else np.array([0.0, 1.0, 0.0])
        e1 = ref - n * ref.dot(n)
        e1 = e1 / max(np.linalg.norm(e1), 1e-9)
        e2 = np.cross(n, e1)

        az, el = [], []
        for v in segments:
            if np.linalg.norm(v) < 1e-9:
                az.append(0.0)
                el.append(0.0)
                continue
            u = v / np.linalg.norm(v)
            az.append(np.arctan2(u.dot(e2), u.dot(e1)))  # azimuth in plane
            el.append(np.arcsin(max(-1.0, min(1.0, u.dot(n)))))  # elevation toward normal
        return az, el

    @staticmethod
    def wrap(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def geometric_relative(self, joints):
        # Inter-segment (child - parent) azimuth/elevation per arm joint, ordered
        # [shoulder(joint3), elbow(joint2), wrist(joint1)]. The shoulder's parent is the
        # fixed torso, so its relative angle is the upper-arm direction itself; the
        # constant torso offset is removed later by subtracting the neutral reference.
        az, el = self.segment_angles(joints)  # segments: 0=upper arm, 1=forearm, 2=hand
        rel_az = [az[0], self.wrap(az[1] - az[0]), self.wrap(az[2] - az[1])]
        rel_el = [el[0], self.wrap(el[1] - el[0]), self.wrap(el[2] - el[1])]
        return rel_az, rel_el

    def geometric_target(self):
        if self.geom_ref is None:
            neutral_joints = self.neutral_device_joints if self.neutral_device_joints else {}
            self.geom_ref = self.geometric_relative(neutral_joints)
        az0, el0 = self.geom_ref
        az, el = self.geometric_relative(self.latest_device_joints)

        # DRAGON joints carried by the shoulder/elbow/wrist articulation.
        # joint3 <- shoulder (segment 0), joint2 <- elbow (segment 1), joint1 <- wrist (segment 2).
        yaw = [self.geom_yaw_sign * self.geom_yaw_scale * self.wrap(az[k] - az0[k]) for k in range(3)]
        pitch = [self.geom_pitch_sign * self.geom_pitch_scale * self.wrap(el[k] - el0[k]) for k in range(3)]

        # joint_names order: [j1_pitch, j1_yaw, j2_pitch, j2_yaw, j3_pitch, j3_yaw].
        # segment index: wrist=2 -> joint1, elbow=1 -> joint2, shoulder=0 -> joint3.
        ordered = [
            pitch[2], yaw[2],   # joint1
            pitch[1], yaw[1],   # joint2
            pitch[0], yaw[0],   # joint3
        ]
        return [self.clamp(v) for v in ordered]

    def evaluate_feasibility(self, candidate):
        # Returns (feasible: bool, fc_f_min, fc_t_min). On service failure returns
        # (None, ...) so the caller can hold the last feasible shape conservatively.
        if self.feasibility_srv is None:
            if not self.connect_feasibility_service():
                return None, None, None
        req = ShapeFeasibilityRequest()
        req.name = list(self.joint_names)
        req.position = list(candidate)
        try:
            res = self.feasibility_srv.call(req)
        except (rospy.ServiceException, TypeError):
            rospy.logwarn_throttle(5.0, "shape feasibility service call failed; reconnecting")
            self.feasibility_srv = None
            return None, None, None
        if not res.valid:
            return None, res.fc_f_min, res.fc_t_min
        feasible = (res.fc_f_min >= self.force_radius_threshold and
                    res.fc_t_min >= self.torque_radius_threshold)
        return feasible, res.fc_f_min, res.fc_t_min

    def publish_candidate_fc(self, fc_f, fc_t):
        if fc_f is not None:
            self.candidate_fc_f_pub.publish(Float64(fc_f))
        if fc_t is not None:
            self.candidate_fc_t_pub.publish(Float64(fc_t))

    def interpolate_target(self, src, dst, fraction):
        return self.clamp_target([
            float(a) + float(fraction) * (float(b) - float(a))
            for a, b in zip(src, dst)
        ])

    def feasibility_soft_scale(self, fc_f, fc_t):
        if fc_f is None or fc_t is None:
            return 0.0
        force_scale = 1.0 if self.force_radius_threshold <= 0.0 else fc_f / self.force_radius_threshold
        torque_scale = 1.0 if self.torque_radius_threshold <= 0.0 else fc_t / self.torque_radius_threshold
        scale = max(0.0, min(1.0, force_scale, torque_scale))
        return max(float(self.feasibility_soft_min_scale), scale)

    def step_search_target(self, candidate):
        fraction = min(1.0, max(0.0, float(self.feasibility_step_fraction)))
        min_fraction = min(fraction, max(0.0, float(self.feasibility_min_step_fraction)))
        while fraction >= min_fraction and fraction > 0.0:
            trial = self.interpolate_target(self.last_feasible_target, candidate, fraction)
            feasible, fc_f, fc_t = self.evaluate_feasibility(trial)
            if feasible:
                self.publish_candidate_fc(fc_f, fc_t)
                self.last_feasible_target = trial
                self.log_gate(True, fc_f, fc_t)
                return list(self.last_feasible_target)
            fraction *= 0.5
        return list(self.last_feasible_target)

    def log_gate(self, feasible, fc_f, fc_t):
        if feasible != self.last_gate_feasible:
            self.last_gate_feasible = feasible
        now = rospy.Time.now()
        if self.gate_log_period > 0.0 and (now - self.last_gate_log_stamp).to_sec() < self.gate_log_period:
            return
        self.last_gate_log_stamp = now
        f = "n/a" if fc_f is None else "%.4f" % fc_f
        t = "n/a" if fc_t is None else "%.4f" % fc_t
        msg = ("shape feasibility: deform=%s fc_f_min=%s fc_t_min=%s "
               "thresholds(force/torque)=%.4f/%.4f") % (
            feasible, f, t, self.force_radius_threshold, self.torque_radius_threshold)
        if feasible:
            rospy.loginfo(msg)
        else:
            rospy.logwarn(msg)

    def teleop_shape_target(self):
        # Map the arm to a candidate DRAGON shape, then gate by predicted feasibility.
        candidate = self.mapped_target()
        if not self.enable_feasibility_gate:
            self.last_feasible_target = candidate
            return candidate

        # Throttle the (expensive) feasibility evaluation; hold between checks.
        now = rospy.Time.now()
        if self.feasibility_rate > 0.0 and \
                (now - self.last_feasibility_eval_stamp).to_sec() < 1.0 / self.feasibility_rate:
            return list(self.last_feasible_target)
        self.last_feasibility_eval_stamp = now

        feasible, fc_f, fc_t = self.evaluate_feasibility(candidate)
        self.publish_candidate_fc(fc_f, fc_t)
        if feasible:
            self.last_feasible_target = candidate
        elif self.feasibility_gate_mode == "step_search":
            self.log_gate(False, fc_f, fc_t)
            return self.step_search_target(candidate)
        elif self.feasibility_gate_mode == "soft_scale":
            scale = self.feasibility_soft_scale(fc_f, fc_t)
            target = self.interpolate_target(self.last_feasible_target, candidate, scale)
            self.last_feasible_target = target
            self.log_gate(scale > 0.0, fc_f, fc_t)
            return list(self.last_feasible_target)
        # feasible False or None (service failure / invalid) -> hold last feasible.
        self.log_gate(bool(feasible), fc_f, fc_t)
        return list(self.last_feasible_target)

    def desired_target(self):
        if self.teleop_mode == "startup":
            self.last_feasible_target = list(self.startup_pose)
            return list(self.startup_pose)
        return self.teleop_shape_target()

    def rate_limit(self, target):
        limited = []
        for cur, dst in zip(self.current_target, target):
            delta = max(-self.max_step, min(self.max_step, dst - cur))
            limited.append(cur + delta)
        return limited

    def publish_shape_error(self, desired, target):
        msg = Float64MultiArray()
        msg.data = [float(d - t) for d, t in zip(desired, target)]
        self.shape_error_pub.publish(msg)

    def make_joint_msg(self):
        target = self.desired_target()
        # desired (raw mapping) vs target (feasible-gated) error, for haptic feedback.
        desired = self.mapped_target() if self.teleop_mode == "teleoperation" else target
        self.publish_shape_error(desired, target)
        self.current_target = self.rate_limit(target)

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = list(self.current_target)
        return msg

    def can_publish_joint_command(self):
        if self.publish_only_when_hovering and not self.robot_hovering:
            return False
        if self.teleop_mode == "teleoperation" and not self.publish_before_device_ready and not self.latest_device_joints:
            return False
        return True

    def main(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            if self.can_publish_joint_command():
                self.joints_ctrl_pub.publish(self.make_joint_msg())
            rate.sleep()


if __name__ == "__main__":
    try:
        node = ControlJoints()
        node.main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))
