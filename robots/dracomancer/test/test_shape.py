#!/usr/bin/env python3
"""Numerical and controller regression tests; run after sourcing the workspace."""
import copy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch, Mock

import numpy as np
import yaml
import xacro

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts/control"))
from shape_mapping import (UrdfModel, HumanChain, DragonChain, ShapeMapper,
                           centerline, limit_step, DRAGON_JOINTS, transform)


class ShapeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        robots = PACKAGE.parent
        cls.human_model = UrdfModel(xacro.process_file(str(
            robots / "dracomancer/urdf/dracomancer.urdf.xacro")).toxml())
        cls.dragon_model = UrdfModel(xacro.process_file(str(
            robots / "dragon/robots/quad/v1_5_202601.urdf.xacro")).toxml())
        cls.config = yaml.safe_load((PACKAGE / "config/control/shape.yaml").read_text())["shape"]
        cls.config["torso_point"] = [0, 0, 0]
        cls.human = HumanChain(cls.human_model, cls.config)
        cls.dragon = DragonChain(cls.dragon_model, cls.config["dragon_tail"])
        cls.zero = {"right_joint_%d" % i: 0. for i in range(1, 8)}

    def mapper(self, **overrides):
        config = dict(self.config, **overrides)
        return ShapeMapper(self.human, self.dragon, config, [0]*6)

    def test_names_not_indices(self):
        pairs = list(self.zero.items())
        pairs[3] = (pairs[3][0], .4)
        np.testing.assert_allclose(self.human.points(dict(pairs)), self.human.points(dict(pairs[::-1])))

    def test_missing_and_nonfinite(self):
        values = dict(self.zero)
        del values["right_joint_3"]
        with self.assertRaises(KeyError):
            self.human.points(values)
        for value in (float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                self.human.points(dict(self.zero, right_joint_4=value))

    def test_unconfigured_torso_rejected(self):
        with self.assertRaises(ValueError):
            HumanChain(self.human_model, dict(self.config, torso_point=[]))

    def test_sampling_and_similarity(self):
        points = np.array([[0, 0, 0], [2, 0, 0], [2, 1, 0]])
        expected = np.array([[0, 0, 0], [1.5, 0, 0], [2, 1, 0]])/3
        np.testing.assert_allclose(centerline(points, 3), expected)
        np.testing.assert_allclose(centerline(points), centerline(points*5+[2, 3, 4]), atol=1e-14)
        with self.assertRaises(ValueError):
            centerline([[0, 0, 0], [0, 0, 0]])

    def test_fixed_mount_and_shoulder(self):
        baseline = self.mapper().human_shape(self.zero)
        tilted = dict(self.zero, right_joint_1=.3)
        self.assertGreater(np.linalg.norm(baseline-self.mapper().human_shape(tilted)), .01)
        # base_link FK removes the fixed world mounting, never shoulder rotation.
        model = copy.deepcopy(self.human_model)
        model.parents["base_link"]["origin"] = transform([9, 8, 7], [.4, -.3, .2])
        other = HumanChain(model, self.config)
        np.testing.assert_allclose(self.human.points(tilted), other.points(tilted))

    def test_dragon_physical_segments_and_reverse(self):
        q = np.array([.2, .4, -.3, .5, .1, -.2])
        points = self.dragon.points(q)
        self.assertEqual(points.shape, (8, 3))
        np.testing.assert_allclose(np.linalg.norm(np.diff(points, axis=0), axis=1),
                                   [.474, .0515, .474, .0515, .474, .0515, .474], atol=1e-12)
        values = dict(zip(DRAGON_JOINTS, q))
        last = self.dragon_model.pose("link1", "link4", values).dot(transform(self.dragon.tail))
        np.testing.assert_allclose(points[-1], -last[:3, :3].T.dot(last[:3, 3]), atol=1e-12)
        np.testing.assert_allclose(self.dragon.velocity, [.1]*6)
        np.testing.assert_allclose(self.dragon.lower, [-np.pi/2]*6)

    def test_fk_against_kdl(self):
        import PyKDL as kdl
        for values in (self.zero, {k: .13*i for i, k in enumerate(self.zero)}):
            for node in self.config["human_points"]:
                result = kdl.Frame()
                for joint in self.human_model.chain("base_link", node["link"]):
                    t = joint["origin"]
                    rot = kdl.Rotation(*t[:3, :3].ravel())
                    origin = kdl.Frame(rot, kdl.Vector(*t[:3, 3]))
                    motion = kdl.Frame(kdl.Rotation.Rot(kdl.Vector(*joint["axis"]), values[joint["name"]]))
                    result = result * origin * motion
                actual = self.human_model.pose("base_link", node["link"], values)
                np.testing.assert_allclose(actual[:3, 3], [result.p[i] for i in range(3)], atol=1e-12)
                np.testing.assert_allclose(actual[:3, :3], [[result.M[i, j] for j in range(3)] for i in range(3)], atol=1e-12)

    def test_batched_fk_matches_scalar(self):
        angles = np.random.default_rng(12).uniform(-1.4, 1.4, (6, 6))
        points = self.dragon.point_batch(angles)
        for q, actual in zip(angles, points):
            np.testing.assert_allclose(actual, self.dragon.points(q), atol=1e-12)

    def test_feedback_fk_can_describe_joint_limit_overshoot(self):
        values = dict.fromkeys(DRAGON_JOINTS, 0.)
        values["joint1_yaw"] = np.pi/2+.001
        with self.assertRaises(ValueError):
            self.dragon_model.pose("link1", "link4", values)
        actual = self.dragon_model.pose("link1", "link4", values, enforce_limits=False)
        self.assertTrue(np.all(np.isfinite(actual)))

    def test_reachable_shape(self):
        mapper = self.mapper(continuity=0., rest_weight=0., max_iterations=50, time_budget=2.)
        target = mapper.dragon_shape([.2, .3, -.2, .4, .1, -.3])
        result = mapper.solve(target, [0]*6)
        self.assertTrue(result["ok"])
        self.assertLess(result["rmse"], 1e-4)

    def test_unreachable_shape_is_bounded(self):
        mapper = self.mapper(time_budget=2.)
        result = mapper.solve(mapper.human_shape(dict(self.zero, right_joint_4=1.2)), [0]*6)
        self.assertTrue(result["ok"])
        self.assertTrue(np.isfinite(result["rmse"]))
        self.assertTrue(np.all(result["q"] <= self.dragon.upper))
        self.assertTrue(np.all(result["q"] >= self.dragon.lower))

    def test_timeout_holds_previous(self):
        mapper = self.mapper(time_budget=1e-12)
        previous = [.1]*6
        result = mapper.solve(mapper.dragon_shape([.5]*6), previous)
        self.assertFalse(result["ok"])
        np.testing.assert_array_equal(result["q"], previous)

    def test_regularization_and_continuity(self):
        mapper = self.mapper(continuity=10., time_budget=2.)
        previous = np.zeros(6)
        for i in range(5):
            result = mapper.solve(mapper.dragon_shape([i*.03]*6), previous)
            self.assertLess(np.linalg.norm(result["q"]-previous), .01)
            previous = result["q"]
        strong_rest = self.mapper(continuity=0., rest_weight=100., time_budget=2.)
        result = strong_rest.solve(strong_rest.dragon_shape([.5]*6), [.3]*6)
        self.assertLess(np.linalg.norm(result["q"]), .02)

    def test_active_joint_bound_does_not_stall_free_axes(self):
        mapper = self.mapper(max_iterations=1, time_budget=2.)
        previous = np.array([.15, 1.4, .17, np.pi/2, -.17, np.pi/2])
        target = mapper.human_shape(dict(self.zero, right_joint_1=.25))
        initial = np.linalg.norm((mapper.dragon_shape(previous)-target).ravel()*mapper.sqrt_w)
        result = mapper.solve(target, previous)
        self.assertTrue(result["ok"])
        self.assertLess(result["rmse"], initial)
        self.assertGreater(np.linalg.norm(result["q"]-previous), .01)

    def test_last_wrist_rotation_changes_hand_point(self):
        baseline = self.human.points(self.zero)
        changed = self.human.points(dict(self.zero, right_joint_7=.3))
        np.testing.assert_allclose(baseline[:-1], changed[:-1])
        self.assertGreater(np.linalg.norm(changed[-1]-baseline[-1]), .001)

    def test_velocity_and_step_limits(self):
        for dt in (0, .001, .025, .1):
            q = limit_step([0]*6, [1, -1]*3, [.1]*6, dt, .015)
            self.assertTrue(np.all(np.abs(q) <= min(.1*dt, .015)+1e-12))
        with self.assertRaises(ValueError):
            limit_step([0]*6, [np.nan]*6, [.1]*6, .025, .015)

    def test_single_joint_thesis_equations(self):
        from joint_mapping import JointMapper
        mapper = JointMapper(self.human_model,self.dragon,self.config,[0]*6)
        q = np.array([.1,.2,np.pi/4,.4,np.pi/4,.2,.3])
        # Wrist rotates by shoulder_rot + forearm_pron = pi/2.
        np.testing.assert_allclose(mapper.reference(q),[-.3,.2,.2,.2,.2,.1],atol=1e-12)

    def test_dual_joint_neutral_and_independent_elbows(self):
        from joint_mapping import JointMapper
        mapper = JointMapper(self.human_model,self.dragon,self.config,[0]*6,True)
        np.testing.assert_allclose(mapper.reference(np.zeros(14)),0.)
        values = np.zeros(14)
        values[10]=.4  # right elbow, left-first anatomical input order
        np.testing.assert_allclose(mapper.reference(values),[0,.4,0,0,0,0])
        values[3]=.2
        np.testing.assert_allclose(mapper.reference(values),[0,.4,0,0,0,.2])
        values[[0,7]]=.3
        values[[1,8]]=.2
        np.testing.assert_allclose(mapper.reference(values)[2:4],0.,atol=1e-12)

    def test_bimanual_centerline_uses_body_not_left_hand(self):
        from shape_mapping import BimanualHumanChain
        human = BimanualHumanChain(self.human_model,self.config)
        values = {n:0. for n in human.names}
        points = human.points(values)
        self.assertEqual(points.shape,(8,3))
        np.testing.assert_allclose(human.body(values),.5*(points[3]+points[4]))
        mapper = ShapeMapper(human,self.dragon,self.config,[0]*6)
        expected = centerline(points,mapper.samples,mapper.human_alignment,human.body(values))
        np.testing.assert_allclose(mapper.human_shape(values),expected)
        self.assertGreater(np.linalg.norm(expected[0]),.01)
        self.assertTrue(np.all(np.isfinite(mapper.dragon_shape([.1]*6))))

    def test_direct_jacobian_power_with_calibrated_coordinates(self):
        from joint_mapping import JointMapper
        mapper = JointMapper(self.human_model,self.dragon,self.config,[0]*6,True)
        values = {n:.05 for n in mapper.human.names}
        matrix = mapper.mapping_jacobian(values,[0]*6)
        self.assertEqual(matrix.shape,(6,14))
        velocity,wrench = np.linspace(-.1,.1,14),np.arange(6.)
        self.assertAlmostEqual(float(wrench @ (matrix @ velocity)),float((matrix.T @ wrench) @ velocity))


class ControllerTests(unittest.TestCase):
    def setUp(self):
        from control_joint_angle import ControlJoints
        self.node = ControlJoints.__new__(ControlJoints)

    def test_shape_mapping_dispatch(self):
        n = self.node
        n.shape_target = Mock(return_value=[.2]*6)
        self.assertEqual(n.mapped_target(), [.2]*6)
        n.shape_target.assert_called_once_with()

    def test_packet_validation(self):
        from sensor_msgs.msg import JointState
        self.assertTrue(self.node.valid_joint_packet(JointState(name=["a"], position=[0])))
        for msg in (JointState(name=["a", "a"], position=[0, 0]),
                    JointState(name=["a"], position=[]), JointState(name=["a"], position=[np.nan])):
            self.assertFalse(self.node.valid_joint_packet(msg))

    def test_single_solve_per_cycle(self):
        n = self.node
        n.shape_cached = [.4]*6
        n.shape_mapper = Mock()
        self.assertEqual(n.shape_target(), [.4]*6)
        self.assertEqual(n.shape_target(), [.4]*6)
        n.shape_mapper.solve.assert_not_called()

    def test_rate_limit_clock_reset_and_pause(self):
        import rospy
        n = self.node
        n.current_target, n.joint_velocity_limits = [0]*6, [.1]*6
        n.rate_hz, n.max_step = 40., .015
        for now, previous, bound in ((10, None, 0), (10, 9, .0025), (10, 10.1, 0), (10, 9.999, .0001)):
            n.last_command_stamp = rospy.Time.from_sec(previous) if previous is not None else None
            with patch("rospy.Time.now", return_value=rospy.Time.from_sec(now)):
                self.assertTrue(np.all(np.abs(n.rate_limit([1]*6)) <= bound+1e-12))

    def test_candidate_failure_holds_sent_command(self):
        import rospy
        n = self.node
        n.mapped_target = Mock(return_value=[1]*6)
        n.publish_candidate_target = Mock()
        n.current_target = [.2]*6
        n.shape_result, n.enable_feasibility_gate = {"ok": True}, True
        n.feasibility_rate, n.last_feasibility_eval_stamp = 0, rospy.Time(0)
        n.feasibility_hysteresis_holding = False
        n.evaluate_feasibility = Mock(return_value=(None, None, None, None))
        n.publish_candidate_fc = n.log_gate = Mock()
        with patch("rospy.Time.now", return_value=rospy.Time(10)):
            self.assertEqual(n.shape_gated_target(), [.2]*6)

    def test_stale_future_missing_and_zero_stamp(self):
        import rospy
        n = self.node
        n.shape_mapper = Mock()
        n.shape_mapper.human.required = {"a", "b"}
        n.latest_device_joints = {"a": 0, "b": 0}
        n.shape_packet_valid, n.shape_timeout = True, .5
        n.shape_input_received = rospy.Time(10)
        with patch("rospy.Time.now", return_value=rospy.Time(10)):
            for timestamp, expected in ((9.9, True), (9, False), (10.1, False), (0, False)):
                n.shape_input_stamp = rospy.Time.from_sec(timestamp)
                self.assertEqual(n.shape_input_ready(), expected)
            n.shape_input_stamp = rospy.Time(10)
            n.latest_device_joints = {"a": 0}
            self.assertFalse(n.shape_input_ready())

    def test_final_command_invalid_or_unstable_holds(self):
        n = self.node
        n.enable_feasibility_gate = n.teleop_mode = True
        n.current_target = [.2]*6
        n.target_fc_f_pub = n.target_fc_t_pub = Mock()
        for feasible in (False, None):
            n.shape_result = {}
            n.evaluate_feasibility = Mock(return_value=(feasible, .0, .0, False))
            self.assertEqual(n.shape_check_command([.3]*6), [.2]*6)
            self.assertEqual(n.shape_result["gate"], "command_hold")

    def test_hover_gate_and_unprepared_input(self):
        n = self.node
        n.shape_ready = Mock(return_value=True)
        n.publish_only_when_hovering, n.robot_hovering = True, False
        self.assertFalse(n.can_publish_joint_command())
        n.robot_hovering = True
        n.shape_ready.return_value = False
        n.publish_shape_diagnostics = Mock()
        self.assertFalse(n.can_publish_joint_command())

    def test_failed_solver_freezes_without_gate_call(self):
        n = self.node
        n.mapped_target = Mock(return_value=[1]*6)
        n.publish_candidate_target = Mock()
        n.current_target = [.2]*6
        n.shape_result = {"ok": False}
        n.evaluate_feasibility = Mock()
        self.assertEqual(n.shape_gated_target(), [.2]*6)
        n.evaluate_feasibility.assert_not_called()

    def test_shape_not_initialized_from_unsafe_measurement(self):
        import rospy
        n = self.node
        n.shape_setup = Mock(return_value=True)
        n.shape_initialized = False
        n.shape_robot_valid = True
        n.latest_robot_joint_stamp = rospy.Time(10)
        n.shape_timeout = .5
        n.shape_input_ready = Mock(return_value=True)
        n.joint_names = DRAGON_JOINTS
        n.latest_robot_joints = dict.fromkeys(DRAGON_JOINTS, 0.)
        n.shape_mapper = Mock()
        n.shape_mapper.dragon.lower = np.array([-np.pi/2]*6)
        n.shape_mapper.dragon.upper = -n.shape_mapper.dragon.lower
        n.enable_feasibility_gate = True
        n.evaluate_feasibility = Mock(return_value=(False, 0, 0, False))
        with patch("rospy.Time.now", return_value=rospy.Time(10)):
            self.assertFalse(n.shape_ready())
        self.assertFalse(n.shape_initialized)


if __name__ == "__main__":
    unittest.main()
