#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf2_ros
import tf.transformations as tft
from std_msgs.msg import Float64, Float64MultiArray, UInt8, String, Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav
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
        # aerial_robot_control::HOVER_STATE. Navigation commands are ignored by
        # DRAGON outside this state, so link4 anchor must not treat LAND/PRE_LAND as
        # anchor-capable just because their numeric value is larger.
        self.hover_flight_state = int(rospy.get_param("~hover_flight_state", 5))
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

        # Mapping strategy: "distal" (default, absolute match of distal arm joints
        # (wrist + elbow) to DRAGON joints, every unmapped joint held unchanged),
        # "joint_pairing" (medium-term), or "geometric" (long-term, FK + plane
        # projection). "elbow_only" is a backward-compatible alias of "distal".
        # See README.md.
        self.mapping_mode = str(rospy.get_param("~mapping_mode", "distal")).lower()
        if self.mapping_mode == "elbow_only":
            self.mapping_mode = "distal"  # backward-compatible alias
        if self.mapping_mode not in ("joint_pairing", "geometric", "distal"):
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

        # --- distal mapping (absolute match of distal arm joints) ----------------
        # Each listed human arm joint is matched in ABSOLUTE angle to a DRAGON joint:
        #   target_joint = clamp(sign * scale * source_angle + offset)  (no neutral).
        # Default: wrist flexion (beckoning) -> joint1_pitch, wrist abduction (sweep
        # parallel to the palm) -> joint1_yaw, elbow flexion -> joint2 (pitch/yaw
        # selected by upper-arm roll below), shoulder flexion -> joint3_pitch,
        # shoulder abduction -> joint3_yaw. Signs default to [1, -1, 1, 1, -1] (flip
        # per joint if DRAGON bends the wrong way); scales 1.0 give a 1:1 angle match;
        # offsets shift the zero. The device measures shoulder flexion as negative
        # (~-pi/2 at a 90deg operator shoulder), so joint3_pitch uses sign=+1,
        # offset=pi/2 -> source -pi/2 maps to joint3_pitch = 0. Joints not listed are
        # held at their last commanded value, so only the mapped joints move.
        self.joint_index = {name: i for i, name in enumerate(self.joint_names)}
        distal_sources = rospy.get_param("~distal_source_joints", [
            "wrist_flexion_extension_joint",
            "wrist_abduction_adduction_joint",
            "elbow_flexion_extension_joint",
            "shoulder_flexion_extension_joint",
            "shoulder_abduction_adduction_joint",
        ])
        distal_targets = rospy.get_param("~distal_target_joints", [
            "joint1_pitch",
            "joint1_yaw",
            "joint2_yaw",
            "joint3_pitch",
            "joint3_yaw",
        ])
        distal_signs = rospy.get_param("~distal_signs", [1.0, -1.0, 1.0, 1.0, -1.0])
        distal_scales = rospy.get_param("~distal_scales", [1.0, 1.0, 1.0, 1.0, 1.0])
        # Per-joint additive offset [rad]: target = clamp(sign*scale*source + offset).
        # shoulder flexion (joint3_pitch) uses offset=pi/2 with sign=+1 so that a 90deg
        # operator shoulder maps to joint3_pitch = 0.
        distal_offsets = rospy.get_param("~distal_offsets", [0.0, 0.0, 0.0, np.pi / 2.0, 0.0])
        self.distal_map = []  # list of (source_joint, target_joint, sign, scale, offset)
        for k in range(min(len(distal_sources), len(distal_targets))):
            dst = distal_targets[k]
            if dst not in self.joint_index:
                rospy.logwarn("distal: target joint '%s' not in dragon_joint_names; skipped", dst)
                continue
            sign = float(distal_signs[k]) if k < len(distal_signs) else 1.0
            scale = float(distal_scales[k]) if k < len(distal_scales) else 1.0
            offset = float(distal_offsets[k]) if k < len(distal_offsets) else 0.0
            self.distal_map.append((distal_sources[k], dst, sign, scale, offset))
        if self.mapping_mode == "distal" and not self.distal_map:
            rospy.logwarn("distal: no valid source->target mapping; no joint will move")

        # Route elbow flexion to the DRAGON joint2 axis that matches the operator's
        # visible elbow plane. When upper-arm roll is near zero, the forearm is close
        # to vertical and elbow flexion should bend joint2_pitch. When upper-arm roll
        # is far from zero, the elbow opens/closes in a ground-parallel plane and
        # should bend joint2_yaw. By default pitch/yaw zones are both 15deg, so the
        # mapping switches at +/-15deg; widening yaw_zone enables a blend band.
        self.enable_elbow_roll_switching = rospy.get_param("~enable_elbow_roll_switching", True)
        self.elbow_source_joint = rospy.get_param(
            "~elbow_source_joint", "elbow_flexion_extension_joint")
        self.elbow_roll_joint = rospy.get_param(
            "~elbow_roll_joint", "upper_arm_external_internal_rotation_joint")
        self.elbow_pitch_target_joint = rospy.get_param(
            "~elbow_pitch_target_joint", "joint2_pitch")
        self.elbow_yaw_target_joint = rospy.get_param(
            "~elbow_yaw_target_joint", "joint2_yaw")
        self.elbow_roll_pitch_zone = abs(float(rospy.get_param(
            "~elbow_roll_pitch_zone", np.deg2rad(15.0))))
        self.elbow_roll_yaw_zone = abs(float(rospy.get_param(
            "~elbow_roll_yaw_zone", np.deg2rad(15.0))))
        if self.elbow_roll_yaw_zone < self.elbow_roll_pitch_zone:
            rospy.logwarn("elbow_roll_yaw_zone is smaller than pitch_zone; using pitch_zone")
            self.elbow_roll_yaw_zone = self.elbow_roll_pitch_zone
        self.elbow_pitch_sign = float(rospy.get_param("~elbow_pitch_sign", 1.0))
        self.elbow_yaw_sign = float(rospy.get_param("~elbow_yaw_sign", 1.0))
        self.elbow_pitch_scale = float(rospy.get_param("~elbow_pitch_scale", 1.0))
        self.elbow_yaw_scale = float(rospy.get_param("~elbow_yaw_scale", 1.0))
        self.elbow_pitch_offset = float(rospy.get_param("~elbow_pitch_offset", 0.0))
        self.elbow_yaw_offset = float(rospy.get_param("~elbow_yaw_offset", 0.0))

        # Same roll-plane routing for wrist flexion: upper-arm + forearm roll
        # estimates the palm orientation. Near the palm-horizontal reference the
        # wrist bend is expressed as DRAGON joint1_pitch; near vertical it is
        # expressed as joint1_yaw.
        self.enable_wrist_roll_switching = rospy.get_param("~enable_wrist_roll_switching", True)
        self.wrist_source_joint = rospy.get_param(
            "~wrist_source_joint", "wrist_flexion_extension_joint")
        self.wrist_roll_joints = rospy.get_param("~wrist_roll_joints", [
            "upper_arm_external_internal_rotation_joint",
            "wrist_supination_joint",
        ])
        if isinstance(self.wrist_roll_joints, str):
            self.wrist_roll_joints = [self.wrist_roll_joints]
        self.wrist_yaw_source_joint = rospy.get_param(
            "~wrist_yaw_source_joint", "wrist_abduction_adduction_joint")
        self.wrist_pitch_target_joint = rospy.get_param(
            "~wrist_pitch_target_joint", "joint1_pitch")
        self.wrist_yaw_target_joint = rospy.get_param(
            "~wrist_yaw_target_joint", "joint1_yaw")
        self.wrist_roll_parallel_offset = float(rospy.get_param(
            "~wrist_roll_parallel_offset", 0.0))
        self.wrist_roll_pitch_zone = abs(float(rospy.get_param(
            "~wrist_roll_pitch_zone", np.deg2rad(45.0))))
        self.wrist_roll_yaw_zone = abs(float(rospy.get_param(
            "~wrist_roll_yaw_zone", np.deg2rad(45.0))))
        if self.wrist_roll_yaw_zone < self.wrist_roll_pitch_zone:
            rospy.logwarn("wrist_roll_yaw_zone is smaller than pitch_zone; using pitch_zone")
            self.wrist_roll_yaw_zone = self.wrist_roll_pitch_zone
        self.wrist_pitch_sign = float(rospy.get_param("~wrist_pitch_sign", 1.0))
        self.wrist_yaw_sign = float(rospy.get_param("~wrist_yaw_sign", 1.0))
        self.wrist_pitch_scale = float(rospy.get_param("~wrist_pitch_scale", 1.0))
        self.wrist_yaw_scale = float(rospy.get_param("~wrist_yaw_scale", 1.0))
        self.wrist_yaw_source_sign = float(rospy.get_param("~wrist_yaw_source_sign", -1.0))
        self.wrist_yaw_source_scale = float(rospy.get_param("~wrist_yaw_source_scale", 1.0))
        self.wrist_pitch_offset = float(rospy.get_param("~wrist_pitch_offset", 0.0))
        self.wrist_yaw_offset = float(rospy.get_param("~wrist_yaw_offset", 0.0))

        # Latest roll-switch diagnostics (see switch_diag_topic above), held at the
        # "all pitch" default until apply_roll_plane_switching runs at least once.
        self.switch_diag = {
            "wrist": {"r": 0.0, "rho": 0.0, "c_pitch": 1.0, "c_yaw": 0.0},
            "elbow": {"r": 0.0, "rho": 0.0, "c_pitch": 1.0, "c_yaw": 0.0},
        }

        # --- distal link4 anchor -------------------------------------------------
        # Experimental helper: keep the arm-tip link (default link4) roughly fixed in
        # the world while joints bend. The default mode anchors only the link4
        # position via COG POS_MODE; full pose anchoring also compensates baselink
        # attitude, but is much more aggressive for a flying platform.
        self.enable_link4_anchor = rospy.get_param("~enable_link4_anchor", False)
        self.link4_anchor_mode = rospy.get_param("~link4_anchor_mode", "position_only")
        if self.link4_anchor_mode not in ("position_only", "full"):
            rospy.logwarn("unknown link4_anchor_mode '%s'; fallback to position_only",
                          self.link4_anchor_mode)
            self.link4_anchor_mode = "position_only"
        self.link4_anchor_full_pose = self.link4_anchor_mode == "full"
        self.world_frame = rospy.get_param("~world_frame", "world")
        anchor_link = rospy.get_param("~anchor_link", "link4")
        self.anchor_frame = rospy.get_param("~anchor_frame", self.robot_name + "/" + anchor_link)
        self.cog_frame = rospy.get_param("~cog_frame", self.robot_name + "/cog")
        self.baselink_frame = rospy.get_param("~baselink_frame", self.robot_name + "/fc")
        self.nav_topic = rospy.get_param("~nav_topic", "/" + self.robot_name + "/uav/nav")
        self.baselink_rpy_topic = rospy.get_param(
            "~baselink_rpy_topic", "/" + self.robot_name + "/final_target_baselink_rpy")
        self.baselink_motion_topic = rospy.get_param(
            "~baselink_motion_topic", "/" + self.robot_name + "/target_rotation_motion")
        self.publish_baselink_motion = rospy.get_param("~publish_baselink_motion", False)
        self.enable_baselink_roll_mapping = rospy.get_param("~enable_baselink_roll_mapping", True)
        self.baselink_roll_source_joints = rospy.get_param("~baselink_roll_source_joints", [
            "upper_arm_external_internal_rotation_joint",
            "wrist_supination_joint",
        ])
        self.baselink_roll_signs = rospy.get_param("~baselink_roll_signs", [-1.0, -1.0])
        self.baselink_roll_scales = rospy.get_param("~baselink_roll_scales", [1.0, 1.0])
        self.baselink_roll_offset = float(rospy.get_param("~baselink_roll_offset", 0.0))
        self.baselink_roll_limit = abs(float(rospy.get_param("~baselink_roll_limit", np.pi / 2.0)))
        self.baselink_roll_neutral_joints = {}
        self.want_capture_baselink_roll_neutral = True
        self.enable_link4_anchor_body_step_scaling = rospy.get_param(
            "~enable_link4_anchor_body_step_scaling", True)
        self.link4_anchor_max_body_pos_rate = float(rospy.get_param(
            "~link4_anchor_max_body_pos_rate", 0.4))
        self.link4_anchor_max_body_rpy_rate = float(rospy.get_param(
            "~link4_anchor_max_body_rpy_rate", 0.8))
        self.enable_link4_anchor_body_safety = rospy.get_param(
            "~enable_link4_anchor_body_safety", True)
        # Keep these below DRAGON's attitude failsafe (about 1.0 rad in the simulator
        # log) so a link4 target that requires large body tilt is rejected before it is
        # sent to the flight controller.
        self.link4_anchor_max_abs_roll = abs(float(rospy.get_param(
            "~link4_anchor_max_abs_roll", 0.6)))
        self.link4_anchor_max_abs_pitch = abs(float(rospy.get_param(
            "~link4_anchor_max_abs_pitch", 0.6)))
        self.link4_anchor_min_cog_z = float(rospy.get_param(
            "~link4_anchor_min_cog_z", 0.6))
        # A non-positive max disables each absolute COG boundary.
        self.link4_anchor_max_cog_z = float(rospy.get_param(
            "~link4_anchor_max_cog_z", 2.5))
        self.link4_anchor_max_cog_xy_offset = float(rospy.get_param(
            "~link4_anchor_max_cog_xy_offset", 1.0))
        # Minimal DRAGON v1/v1.5 kinematics used for target-shape feed-forward.
        # fc is fixed to link2; link4 is reached through joint2 and joint3.
        self.dragon_link_length = float(rospy.get_param("~dragon_link_length", 0.474))
        self.dragon_inter_joint_x_offset = float(rospy.get_param("~dragon_inter_joint_x_offset", 0.02575))
        self.dragon_link2_fc_xyz = rospy.get_param("~dragon_link2_fc_xyz", [0.3245, -0.0010, 0.0280])
        self.anchor_mat = None
        self.want_capture_anchor = False
        self.cog_fc_mat = None
        self.world_cog_mat = None
        self.anchor_cog_pos = None

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
        # Candidate DRAGON joint target before the feasibility gate (i.e. the
        # direct output of mapped_target()). Republished so analysis scripts do
        # not have to reconstruct it from joints_ctrl + shape_control_error.
        self.candidate_target_topic = rospy.get_param(
            "~candidate_target_topic", self.device_ns + "/candidate/joint_target")
        # Diagnostics for the roll-based pitch/yaw allocation applied in distal
        # mode (joint1=wrist, joint2=elbow): fixed order
        # [r1, rho1, c1_pitch, c1_yaw, r2, rho2, c2_pitch, c2_yaw], where r_i is
        # the roll sum feeding the switch, rho_i in [0,1] is the smoothstep
        # allocation ratio (0=all pitch, 1=all yaw), and c_i_pitch/c_i_yaw are
        # the cos/sin(pi/2 * rho_i) weights actually applied to the target.
        self.switch_diag_topic = rospy.get_param(
            "~switch_diag_topic", self.device_ns + "/joint_map/switch_ratio")

        # Predictive shape-feasibility gate (teleoperation mode):
        #   candidate shape -> shape_feasibility service -> fc_f_min / fc_t_min.
        #   Deform only when BOTH radii are at or above their hard thresholds.
        #   In hold mode, a failed candidate enters a hysteresis hold and resumes
        #   only after BOTH radii recover to their min thresholds.
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
        # Threshold fallback params; overridden by [hard_min, min] threshold topics.
        self.force_radius_threshold = rospy.get_param("~force_radius_threshold", 0.1)
        self.torque_radius_threshold = rospy.get_param("~torque_radius_threshold", 0.01)
        self.force_radius_recover_threshold = rospy.get_param(
            "~force_radius_recover_threshold", self.force_radius_threshold)
        self.torque_radius_recover_threshold = rospy.get_param(
            "~torque_radius_recover_threshold", self.torque_radius_threshold)
        self.feasibility_step_fraction = rospy.get_param("~feasibility_step_fraction", 0.25)
        self.feasibility_min_step_fraction = rospy.get_param("~feasibility_min_step_fraction", 0.03)
        self.feasibility_soft_min_scale = rospy.get_param("~feasibility_soft_min_scale", 0.0)
        # Threshold topics carry [hard_min, min]. hard_min ([0]) starts the hold;
        # min ([1]) releases it, which avoids chattering near the danger boundary.
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
        self.last_link4_body_safe = None
        self.last_gate_log_stamp = rospy.Time(0)
        self.last_gate_feasible = None
        self.feasibility_hysteresis_holding = False
        self.last_feasibility_eval_stamp = rospy.Time(0)
        # Geometric-mode reference (neutral) relative angles, computed lazily so a
        # captured neutral pose can be used if available.
        self.geom_ref = None

        # Publisher
        self.joints_ctrl_pub = rospy.Publisher(self.command_topic, JointState, queue_size=10)
        self.shape_error_pub = rospy.Publisher(self.shape_error_topic, Float64MultiArray, queue_size=1)
        self.candidate_fc_f_pub = rospy.Publisher(self.candidate_force_radius_topic, Float64, queue_size=1)
        self.candidate_fc_t_pub = rospy.Publisher(self.candidate_torque_radius_topic, Float64, queue_size=1)
        self.candidate_target_pub = rospy.Publisher(self.candidate_target_topic, JointState, queue_size=1)
        self.switch_diag_pub = rospy.Publisher(self.switch_diag_topic, Float64MultiArray, queue_size=1)
        self.nav_pub = rospy.Publisher(self.nav_topic, FlightNav, queue_size=1)
        self.baselink_rpy_pub = rospy.Publisher(self.baselink_rpy_topic, Vector3Stamped, queue_size=1)
        self.baselink_motion_pub = rospy.Publisher(self.baselink_motion_topic, Odometry, queue_size=1)

        # TF for the link4 anchor
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

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
        self.recapture_anchor_sub = rospy.Subscriber("~recapture_anchor", Empty, self.recapture_anchor_cb, queue_size=1)

        rospy.loginfo("teleop_mode: %s, mode_topic: %s", self.teleop_mode, self.mode_topic)
        rospy.loginfo("device_joint_topic: %s", self.device_joint_topic)
        rospy.loginfo("command_topic: %s", self.command_topic)
        rospy.loginfo("mapping_mode: %s, mapping_reference: %s",
                      self.mapping_mode, self.mapping_reference)
        if self.mapping_mode == "distal":
            rospy.loginfo("distal absolute match: %s", ", ".join(
                "{}<-clamp({:+.1f}*{:.2f}*{}{:+.3f})".format(dst, sign, scale, src, offset)
                for src, dst, sign, scale, offset in self.distal_map))
            rospy.loginfo(
                "distal elbow roll switching: enable=%s, elbow=%s, roll=%s, pitch=%s, yaw=%s, zones=%.3f/%.3f rad",
                self.enable_elbow_roll_switching,
                self.elbow_source_joint,
                self.elbow_roll_joint,
                self.elbow_pitch_target_joint,
                self.elbow_yaw_target_joint,
                self.elbow_roll_pitch_zone,
                self.elbow_roll_yaw_zone)
            rospy.loginfo(
                "distal wrist roll switching: enable=%s, wrist=%s, yaw_source=%s, roll=%s, pitch=%s, yaw=%s, offset=%.3f rad, zones=%.3f/%.3f rad",
                self.enable_wrist_roll_switching,
                self.wrist_source_joint,
                self.wrist_yaw_source_joint,
                "+".join(self.wrist_roll_joints),
                self.wrist_pitch_target_joint,
                self.wrist_yaw_target_joint,
                self.wrist_roll_parallel_offset,
                self.wrist_roll_pitch_zone,
                self.wrist_roll_yaw_zone)
        rospy.loginfo("joint mapping: %s",
                      ", ".join("{}<-{}".format(dst, src)
                                for dst, src in zip(self.joint_names, self.source_joint_names)))
        rospy.loginfo("joint mapping scale/sign/offset: %s",
                      ", ".join("{}:{:.3f}/{:.3f}/{:.3f}".format(
                          name, scale, sign, offset)
                                for name, scale, sign, offset in zip(
                                    self.joint_names, self.scales, self.signs, self.offsets)))
        rospy.loginfo("feasibility gate: enable=%s, mode=%s, service=%s, hard force/torque=%.4f/%.4f, recover force/torque=%.4f/%.4f",
                      self.enable_feasibility_gate, self.feasibility_gate_mode, self.feasibility_service_name,
                      self.force_radius_threshold, self.torque_radius_threshold,
                      self.force_radius_recover_threshold, self.torque_radius_recover_threshold)
        rospy.loginfo("joint command gating: only_when_hovering=%s, before_device_ready=%s",
                      self.publish_only_when_hovering, self.publish_before_device_ready)
        rospy.loginfo("hover flight_state for joint/link4 commands: %d", self.hover_flight_state)
        rospy.loginfo("link4 anchor: enable=%s, mode=%s, anchor=%s, world=%s, cog=%s, baselink=%s",
                      self.enable_link4_anchor, self.link4_anchor_mode, self.anchor_frame, self.world_frame,
                      self.cog_frame, self.baselink_frame)
        rospy.loginfo("link4 anchor baselink motion: enable=%s, topic=%s",
                      self.publish_baselink_motion, self.baselink_motion_topic)
        rospy.loginfo("link4 anchor body step scaling: enable=%s, pos/rpy rate=%.3f/%.3f",
                      self.enable_link4_anchor_body_step_scaling,
                      self.link4_anchor_max_body_pos_rate,
                      self.link4_anchor_max_body_rpy_rate)
        rospy.loginfo("link4 anchor body safety: enable=%s, max roll/pitch=%.3f/%.3f, z min/max=%.3f/%.3f, xy leash=%.3f",
                      self.enable_link4_anchor_body_safety,
                      self.link4_anchor_max_abs_roll,
                      self.link4_anchor_max_abs_pitch,
                      self.link4_anchor_min_cog_z,
                      self.link4_anchor_max_cog_z,
                      self.link4_anchor_max_cog_xy_offset)

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

    def maybe_capture_baselink_roll_neutral(self, force=False):
        if not self.enable_baselink_roll_mapping:
            return
        if not force and not self.want_capture_baselink_roll_neutral:
            return
        missing = [
            name for name in self.baselink_roll_source_joints
            if name not in self.latest_device_joints
        ]
        if missing:
            rospy.logwarn_throttle(
                2.0,
                "baselink roll neutral capture waiting for joints: %s",
                ", ".join(missing))
            return
        self.baselink_roll_neutral_joints = {
            name: self.latest_device_joints[name]
            for name in self.baselink_roll_source_joints
        }
        self.want_capture_baselink_roll_neutral = False
        rospy.loginfo("Captured dracomancer roll neutral joints for baselink roll mapping")

    def baselink_roll_delta(self):
        if not self.enable_baselink_roll_mapping:
            return 0.0
        self.maybe_capture_baselink_roll_neutral()
        if not self.baselink_roll_neutral_joints:
            return 0.0

        total = self.baselink_roll_offset
        for i, name in enumerate(self.baselink_roll_source_joints):
            if name not in self.latest_device_joints:
                return 0.0
            neutral = self.baselink_roll_neutral_joints.get(name)
            if neutral is None:
                return 0.0
            sign = float(self.baselink_roll_signs[i]) if i < len(self.baselink_roll_signs) else 1.0
            scale = float(self.baselink_roll_scales[i]) if i < len(self.baselink_roll_scales) else 1.0
            total += sign * scale * self.wrap(self.latest_device_joints[name] - neutral)

        if self.baselink_roll_limit > 0.0:
            total = max(-self.baselink_roll_limit, min(self.baselink_roll_limit, total))
        return total

    def device_joint_cb(self, msg):
        self.latest_device_joints = {
            name: float(pos) for name, pos in zip(msg.name, msg.position)
        }
        self.maybe_capture_baselink_roll_neutral()
        if self.capture_neutral and not self.neutral_device_joints:
            # Capture the joints actually used as mapping sources. In joint_pairing the
            # empty (constant) entries are skipped; in geometric every chain joint is
            # used. distal does an absolute match and needs no neutral.
            if self.mapping_mode == "geometric":
                needed = [j[0] for j in self.geom_chain]
            else:
                needed = [n for n in self.source_joint_names if n]
            if needed and all(name in self.latest_device_joints for name in needed):
                self.neutral_device_joints = {
                    name: self.latest_device_joints[name] for name in needed
                }
                rospy.loginfo("Captured dracomancer neutral joints for DRAGON mapping")

    def robot_flight_state_cb(self, msg):
        hovering = int(msg.data) == self.hover_flight_state
        # Capture the link4 anchor at the moment hovering starts.
        if hovering and not self.robot_hovering:
            self.want_capture_anchor = True
            self.want_capture_baselink_roll_neutral = True
        self.robot_hovering = hovering

    def recapture_anchor_cb(self, msg):
        self.want_capture_anchor = True
        self.want_capture_baselink_roll_neutral = True
        rospy.loginfo("link4 anchor recapture requested")

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

    def mapped_target(self):
        if not self.latest_device_joints:
            return list(self.last_feasible_target)
        if self.mapping_mode == "geometric":
            return self.geometric_target()
        if self.mapping_mode == "distal":
            return self.distal_target()
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

    def distal_target(self):
        # Absolute match: each mapped DRAGON joint directly tracks its human arm joint
        # angle (clamp(sign*scale*source)), with no neutral capture. Every unmapped
        # joint is held at its last commanded (feasible) value, so only the mapped
        # joints move.
        target = list(self.last_feasible_target)
        for src, dst, sign, scale, offset in self.distal_map:
            val = self.latest_device_joints.get(src)
            if val is None:
                continue
            target[self.joint_index[dst]] = self.clamp(sign * scale * val + offset)
        self.apply_elbow_roll_switching(target)
        self.apply_wrist_roll_switching(target)
        return target

    def apply_elbow_roll_switching(self, target):
        self.apply_roll_plane_switching(
            target,
            self.enable_elbow_roll_switching,
            self.elbow_source_joint,
            self.elbow_roll_joint,
            self.elbow_pitch_target_joint,
            self.elbow_yaw_target_joint,
            self.elbow_roll_pitch_zone,
            self.elbow_roll_yaw_zone,
            self.elbow_pitch_sign,
            self.elbow_yaw_sign,
            self.elbow_pitch_scale,
            self.elbow_yaw_scale,
            self.elbow_pitch_offset,
            self.elbow_yaw_offset,
            "elbow",
            None,
            0.0,
            0.0)

    def apply_wrist_roll_switching(self, target):
        self.apply_roll_plane_switching(
            target,
            self.enable_wrist_roll_switching,
            self.wrist_source_joint,
            self.wrist_roll_joints,
            self.wrist_pitch_target_joint,
            self.wrist_yaw_target_joint,
            self.wrist_roll_pitch_zone,
            self.wrist_roll_yaw_zone,
            self.wrist_pitch_sign,
            self.wrist_yaw_sign,
            self.wrist_pitch_scale,
            self.wrist_yaw_scale,
            self.wrist_pitch_offset,
            self.wrist_yaw_offset,
            "wrist",
            self.wrist_yaw_source_joint,
            self.wrist_yaw_source_sign,
            self.wrist_yaw_source_scale,
            self.wrist_roll_parallel_offset)

    def apply_roll_plane_switching(self, target, enabled, source_joint, roll_joint,
                                   pitch_target_joint, yaw_target_joint,
                                   pitch_zone, yaw_zone, pitch_sign, yaw_sign,
                                   pitch_scale, yaw_scale, pitch_offset, yaw_offset,
                                   label, yaw_source_joint=None,
                                   yaw_source_sign=1.0, yaw_source_scale=1.0,
                                   roll_offset=0.0):
        if not enabled:
            return
        if (pitch_target_joint not in self.joint_index or
                yaw_target_joint not in self.joint_index):
            rospy.logwarn_throttle(
                2.0,
                "%s roll switching skipped: target joint is missing (%s, %s)",
                label,
                pitch_target_joint,
                yaw_target_joint)
            return

        source = self.latest_device_joints.get(source_joint)
        roll_joint_names = roll_joint if isinstance(roll_joint, list) else [roll_joint]
        roll_values = [self.latest_device_joints.get(name) for name in roll_joint_names]
        yaw_source = None
        if yaw_source_joint:
            yaw_source = self.latest_device_joints.get(yaw_source_joint)
        if (source is None or any(value is None for value in roll_values) or
                (yaw_source_joint and yaw_source is None)):
            rospy.logwarn_throttle(
                2.0,
                "%s roll switching waiting for joints: source=%s yaw_source=%s roll=%s",
                label,
                source is not None,
                (not yaw_source_joint) or (yaw_source is not None),
                [value is not None for value in roll_values])
            return

        roll_sum = self.wrap(sum(self.wrap(value) for value in roll_values) - roll_offset)
        abs_roll = abs(self.wrap(roll_sum))
        if abs_roll <= pitch_zone:
            yaw_ratio = 0.0
        elif abs_roll >= yaw_zone:
            yaw_ratio = 1.0
        else:
            width = max(1e-9, yaw_zone - pitch_zone)
            yaw_ratio = self.smoothstep((abs_roll - pitch_zone) / width)

        # Treat pitch/yaw as a bend vector. For elbow the yaw component is zero, so
        # the vector simply rotates from pitch to yaw. For wrist, abduction/adduction
        # supplies the initial yaw component and rotates with flexion/extension.
        theta = yaw_ratio * np.pi / 2.0
        pitch_weight = np.cos(theta)
        yaw_weight = np.sin(theta)
        pitch_component = pitch_sign * pitch_scale * source
        yaw_component = 0.0
        if yaw_source_joint:
            yaw_component = yaw_source_sign * yaw_source_scale * yaw_source
        pitch = pitch_offset + pitch_weight * pitch_component - yaw_weight * yaw_component
        yaw = yaw_offset + yaw_weight * yaw_sign * yaw_scale * source + pitch_weight * yaw_component

        target[self.joint_index[pitch_target_joint]] = self.clamp(pitch)
        target[self.joint_index[yaw_target_joint]] = self.clamp(yaw)

        self.switch_diag[label] = {
            "r": roll_sum, "rho": yaw_ratio, "c_pitch": pitch_weight, "c_yaw": yaw_weight,
        }

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

    @staticmethod
    def smoothstep(x):
        x = max(0.0, min(1.0, float(x)))
        return x * x * (3.0 - 2.0 * x)

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

    def feasibility_recovered(self, fc_f, fc_t):
        if fc_f is None or fc_t is None:
            return False
        return (fc_f >= self.force_radius_recover_threshold and
                fc_t >= self.torque_radius_recover_threshold)

    def hold_gate_target(self, candidate, feasible, fc_f, fc_t):
        # Hysteresis for the default hold gate:
        #   - entering hold: either radius falls below hard_min.
        #   - leaving hold: both radii recover to min.
        # Service failures (feasible is None) hold conservatively without changing
        # the hysteresis state, so a transient service outage does not make the gate
        # sticky by itself.
        if feasible is None:
            self.log_gate(False, fc_f, fc_t)
            return list(self.last_feasible_target)

        if self.feasibility_hysteresis_holding:
            if self.feasibility_recovered(fc_f, fc_t):
                self.feasibility_hysteresis_holding = False
                self.last_feasible_target = candidate
                self.log_gate(True, fc_f, fc_t)
            else:
                self.log_gate(False, fc_f, fc_t)
            return list(self.last_feasible_target)

        if feasible:
            self.last_feasible_target = candidate
            self.log_gate(True, fc_f, fc_t)
        else:
            self.feasibility_hysteresis_holding = True
            self.log_gate(False, fc_f, fc_t)
        return list(self.last_feasible_target)

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
               "hard(force/torque)=%.4f/%.4f recover(force/torque)=%.4f/%.4f holding=%s") % (
            feasible, f, t, self.force_radius_threshold, self.torque_radius_threshold,
            self.force_radius_recover_threshold, self.torque_radius_recover_threshold,
            self.feasibility_hysteresis_holding)
        if feasible:
            rospy.loginfo(msg)
        else:
            rospy.logwarn(msg)

    def teleop_shape_target(self):
        # Map the arm to a candidate DRAGON shape, then gate by predicted feasibility.
        candidate = self.mapped_target()
        self.publish_candidate_target(candidate)
        if not self.enable_feasibility_gate:
            self.feasibility_hysteresis_holding = False
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
        if self.feasibility_gate_mode == "hold":
            return self.hold_gate_target(candidate, feasible, fc_f, fc_t)
        if feasible:
            self.feasibility_hysteresis_holding = False
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
            self.feasibility_hysteresis_holding = False
            self.last_feasible_target = list(self.startup_pose)
            return list(self.startup_pose)
        return self.teleop_shape_target()

    def rate_limit(self, target):
        limited = []
        for cur, dst in zip(self.current_target, target):
            delta = max(-self.max_step, min(self.max_step, dst - cur))
            limited.append(cur + delta)
        return self.link4_body_step_scale(limited)

    def link4_body_step_scale(self, target):
        if not self.enable_link4_anchor_body_step_scaling:
            return target
        if not self.enable_link4_anchor or self.mapping_mode != "distal":
            return target
        if not self.robot_hovering or self.teleop_mode != "teleoperation":
            return target
        if self.anchor_mat is None or self.cog_fc_mat is None:
            return list(self.current_target)

        current_body = self.link4_anchor_body_target(self.current_target)
        target_body = self.link4_anchor_body_target(target)
        if current_body is None or target_body is None:
            rospy.logwarn_throttle(1.0, "link4 anchor body step scaling holds target: body target unavailable")
            return list(self.current_target)

        pos_limit = self.link4_anchor_max_body_pos_rate / self.rate_hz if self.rate_hz > 0.0 else 0.0
        rpy_limit = self.link4_anchor_max_body_rpy_rate / self.rate_hz if self.rate_hz > 0.0 else 0.0
        if pos_limit <= 0.0 and rpy_limit <= 0.0:
            return target

        cur_pos, cur_rpy = current_body
        dst_pos, dst_rpy = target_body
        pos_delta = float(np.linalg.norm(dst_pos - cur_pos))
        rpy_delta = 0.0
        if self.link4_anchor_full_pose:
            rpy_delta = max(abs(self.wrap(dst_rpy[i] - cur_rpy[i])) for i in range(3))

        scale = 1.0
        if pos_limit > 0.0 and pos_delta > pos_limit:
            scale = min(scale, pos_limit / pos_delta)
        if self.link4_anchor_full_pose and rpy_limit > 0.0 and rpy_delta > rpy_limit:
            scale = min(scale, rpy_limit / rpy_delta)
        if scale >= 1.0:
            return target

        rospy.loginfo_throttle(
            1.0,
            "link4 anchor body step scaling: scale=%.3f pos=%.4f/%.4f rpy=%.4f/%.4f",
            scale, pos_delta, pos_limit, rpy_delta, rpy_limit)
        return self.interpolate_target(self.current_target, target, scale)

    def link4_body_target_safe(self, body_target):
        if not self.enable_link4_anchor or self.mapping_mode != "distal":
            return True
        if body_target is None:
            rospy.logwarn_throttle(1.0, "link4 anchor body safety rejected target: body target unavailable")
            return False
        if not self.enable_link4_anchor_body_safety:
            return True

        pos, rpy = body_target
        reasons = []
        if self.link4_anchor_full_pose:
            roll, pitch, _ = rpy
            if self.link4_anchor_max_abs_roll > 0.0 and abs(roll) > self.link4_anchor_max_abs_roll:
                reasons.append("roll %.3f > %.3f" % (roll, self.link4_anchor_max_abs_roll))
            if self.link4_anchor_max_abs_pitch > 0.0 and abs(pitch) > self.link4_anchor_max_abs_pitch:
                reasons.append("pitch %.3f > %.3f" % (pitch, self.link4_anchor_max_abs_pitch))
        if pos[2] < self.link4_anchor_min_cog_z:
            reasons.append("cog_z %.3f < %.3f" % (pos[2], self.link4_anchor_min_cog_z))
        if self.link4_anchor_max_cog_z > 0.0 and pos[2] > self.link4_anchor_max_cog_z:
            reasons.append("cog_z %.3f > %.3f" % (pos[2], self.link4_anchor_max_cog_z))
        if self.link4_anchor_max_cog_xy_offset > 0.0 and self.anchor_cog_pos is not None:
            xy_offset = float(np.linalg.norm(pos[0:2] - self.anchor_cog_pos[0:2]))
            if xy_offset > self.link4_anchor_max_cog_xy_offset:
                reasons.append("cog_xy_offset %.3f > %.3f" %
                               (xy_offset, self.link4_anchor_max_cog_xy_offset))

        safe = not reasons
        if safe:
            self.last_link4_body_safe = True
            return True

        if self.last_link4_body_safe is not False:
            self.last_link4_body_safe = False
        rospy.logwarn_throttle(
            1.0,
            "link4 anchor body safety rejected target: %s",
            "; ".join(reasons))
        return False

    def link4_body_safety_gate(self, target):
        if not self.enable_link4_anchor or self.mapping_mode != "distal":
            return target
        if not self.robot_hovering or self.teleop_mode != "teleoperation":
            return target
        if self.anchor_mat is None or self.cog_fc_mat is None:
            return list(self.current_target)
        body_target = self.link4_anchor_body_target(target)
        if self.link4_body_target_safe(body_target):
            return target
        # Do not advance the joint target if holding link4 would require an unsafe
        # body command. Keeping current_target also prevents update_link4_anchor()
        # from publishing the rejected body command for the same cycle.
        return list(self.current_target)

    def publish_shape_error(self, desired, target):
        msg = Float64MultiArray()
        msg.data = [float(d - t) for d, t in zip(desired, target)]
        self.shape_error_pub.publish(msg)

    def publish_switch_diag(self):
        wrist = self.switch_diag["wrist"]
        elbow = self.switch_diag["elbow"]
        msg = Float64MultiArray()
        msg.data = [wrist["r"], wrist["rho"], wrist["c_pitch"], wrist["c_yaw"],
                    elbow["r"], elbow["rho"], elbow["c_pitch"], elbow["c_yaw"]]
        self.switch_diag_pub.publish(msg)

    def make_joint_msg(self):
        target = self.desired_target()
        # desired (raw mapping) vs target (feasible-gated) error, for haptic feedback.
        desired = self.mapped_target() if self.teleop_mode == "teleoperation" else target
        self.current_target = self.link4_body_safety_gate(self.rate_limit(target))
        self.publish_shape_error(desired, self.current_target)
        self.publish_switch_diag()

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

    @staticmethod
    def tf_to_matrix(tr):
        t = tr.transform.translation
        q = tr.transform.rotation
        m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        m[0, 3], m[1, 3], m[2, 3] = t.x, t.y, t.z
        return m

    @staticmethod
    def xyz_to_matrix(xyz):
        return tft.translation_matrix([float(xyz[0]), float(xyz[1]), float(xyz[2])])

    def lookup_matrix(self, target_frame, source_frame):
        # pose of source_frame expressed in target_frame
        tr = self.tf_buffer.lookup_transform(target_frame, source_frame,
                                             rospy.Time(0), rospy.Duration(0.1))
        return self.tf_to_matrix(tr)

    def clear_link4_anchor_state(self):
        self.anchor_mat = None
        self.cog_fc_mat = None
        self.world_cog_mat = None
        self.anchor_cog_pos = None

    def prepare_link4_anchor_tf(self):
        if not self.enable_link4_anchor or self.mapping_mode != "distal":
            return True
        if not self.robot_hovering:
            self.clear_link4_anchor_state()
            return False

        try:
            captured = False
            if self.anchor_mat is None or self.want_capture_anchor:
                self.anchor_mat = self.lookup_matrix(self.world_frame, self.anchor_frame)
                self.want_capture_anchor = False
                self.anchor_cog_pos = None
                captured = True

            # Refresh these once near the beginning of the control cycle. The same
            # transforms are then used by step scaling, safety gate, and nav publish.
            self.cog_fc_mat = self.lookup_matrix(self.cog_frame, self.baselink_frame)
            self.world_cog_mat = self.lookup_matrix(self.world_frame, self.cog_frame)
            if self.anchor_cog_pos is None:
                self.anchor_cog_pos = np.array(self.world_cog_mat[0:3, 3], dtype=float)
            if captured:
                self.maybe_capture_baselink_roll_neutral(force=True)
                rospy.loginfo("link4 anchor captured (%s in %s)", self.anchor_frame, self.world_frame)
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            if captured:
                self.anchor_mat = None
                self.anchor_cog_pos = None
                self.want_capture_anchor = True
            self.cog_fc_mat = None
            self.world_cog_mat = None
            rospy.logwarn_throttle(2.0, "link4 anchor TF refresh failed: %s", str(e))
            return False

    def dragon_joint_value(self, joints, name):
        idx = self.joint_index.get(name)
        if idx is None or joints is None or idx >= len(joints):
            return 0.0
        return float(joints[idx])

    def dragon_fc_to_link4_matrix(self, joints):
        # URDF v1/v1.5 chain:
        # fc fixed on link2, then link2 -> joint2_pitch -> joint2_yaw -> link3
        # -> joint3_pitch -> joint3_yaw -> link4.
        m = tft.inverse_matrix(self.xyz_to_matrix(self.dragon_link2_fc_xyz))
        for joint_id in (2, 3):
            pitch = self.dragon_joint_value(joints, "joint%d_pitch" % joint_id)
            yaw = self.dragon_joint_value(joints, "joint%d_yaw" % joint_id)
            m = m.dot(tft.translation_matrix([self.dragon_link_length, 0.0, 0.0]))
            m = m.dot(tft.rotation_matrix(pitch, [0.0, 1.0, 0.0]))
            m = m.dot(tft.translation_matrix([2.0 * self.dragon_inter_joint_x_offset, 0.0, 0.0]))
            m = m.dot(tft.rotation_matrix(yaw, [0.0, 0.0, 1.0]))
        return m

    def link4_anchor_body_target(self, target_joints):
        if self.anchor_mat is None or self.cog_fc_mat is None or target_joints is None:
            return None
        m_bl_link4 = self.dragon_fc_to_link4_matrix(target_joints)
        m_cog_link4 = self.cog_fc_mat.dot(m_bl_link4)

        if not self.link4_anchor_full_pose:
            if self.world_cog_mat is None:
                return None
            # In position_only mode we do not command baselink attitude. Compute the
            # COG position that places link4 at the anchor under the current body
            # orientation, instead of using the full-pose inverse solution.
            rot_world_cog = self.world_cog_mat[0:3, 0:3]
            pos = self.anchor_mat[0:3, 3] - rot_world_cog.dot(m_cog_link4[0:3, 3])
            return np.array(pos, dtype=float), [0.0, 0.0, 0.0]

        # Full mode: world poses that hold link4 at the anchor:
        # T_world_x = T_anchor * T_x_link4^-1
        m_world_cog = self.anchor_mat.dot(tft.inverse_matrix(m_cog_link4))
        m_world_bl = self.anchor_mat.dot(tft.inverse_matrix(m_bl_link4))
        roll, pitch, yaw = tft.euler_from_matrix(m_world_bl)
        roll += self.baselink_roll_delta()
        return np.array([m_world_cog[0, 3], m_world_cog[1, 3], m_world_cog[2, 3]], dtype=float), \
            [roll, pitch, yaw]

    def update_link4_anchor(self, target_joints=None):
        # Keep the anchor link (link4) near the hover-start reference. The default
        # mode fixes only the link4 position by commanding COG position; full mode
        # additionally commands baselink attitude to fix the link4 pose.
        if not self.enable_link4_anchor or self.mapping_mode != "distal":
            return
        if not self.robot_hovering:
            self.clear_link4_anchor_state()  # re-capture on next hover
            return
        # only steer the body while the operator is actively shaping
        if self.teleop_mode != "teleoperation":
            return
        if target_joints is None:
            return
        if self.anchor_mat is None or self.cog_fc_mat is None or self.world_cog_mat is None:
            rospy.logwarn_throttle(1.0, "link4 anchor command skipped: TF unavailable")
            return
        body_target = self.link4_anchor_body_target(target_joints)
        if body_target is None:
            return
        if not self.link4_body_target_safe(body_target):
            return
        pos, rpy_target = body_target

        nav = FlightNav()
        nav.header.stamp = rospy.Time.now()
        nav.control_frame = FlightNav.WORLD_FRAME
        nav.target = FlightNav.COG
        nav.pos_xy_nav_mode = FlightNav.POS_MODE
        nav.pos_z_nav_mode = FlightNav.POS_MODE
        nav.target_pos_x = float(pos[0])
        nav.target_pos_y = float(pos[1])
        nav.target_pos_z = float(pos[2])
        self.nav_pub.publish(nav)

        if self.link4_anchor_full_pose:
            roll, pitch, yaw = rpy_target
            rpy = Vector3Stamped()
            rpy.header.stamp = rospy.Time.now()
            rpy.vector.x, rpy.vector.y, rpy.vector.z = roll, pitch, yaw
            self.baselink_rpy_pub.publish(rpy)

            if self.publish_baselink_motion:
                q = tft.quaternion_from_euler(roll, pitch, yaw)
                motion = Odometry()
                motion.header.stamp = rpy.header.stamp
                # DragonNavigator::targetRotationMotionCallback expects exactly
                # "baselink" here and applies the target immediately.
                motion.header.frame_id = "baselink"
                motion.pose.pose.orientation.x = q[0]
                motion.pose.pose.orientation.y = q[1]
                motion.pose.pose.orientation.z = q[2]
                motion.pose.pose.orientation.w = q[3]
                self.baselink_motion_pub.publish(motion)

    def main(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            self.prepare_link4_anchor_tf()
            if self.can_publish_joint_command():
                joint_msg = self.make_joint_msg()
                self.update_link4_anchor(joint_msg.position)
                self.joints_ctrl_pub.publish(joint_msg)
            else:
                self.update_link4_anchor()
            rate.sleep()


if __name__ == "__main__":
    try:
        node = ControlJoints()
        node.main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))
