#!/usr/bin/env python3
"""Portable Mk-II measurement and command-owner checks with ROS interfaces stubbed."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
sys.path.insert(0, str(PACKAGE / 'scripts/control'))


class Stamp:
    def __init__(self, value=0):
        self.value = float(value)

    def to_sec(self):
        return self.value

    def __sub__(self, other):
        return Stamp(self.value-other.value)

    def __eq__(self, other):
        return isinstance(other, Stamp) and self.value == other.value

    def __lt__(self, other):
        return self.value < other.value

    @staticmethod
    def now():
        return Stamp(10)

    from_sec = classmethod(lambda cls, value: cls(value))


class JointState:
    def __init__(self, name=None, position=None, effort=None):
        self.header = SimpleNamespace(stamp=Stamp())
        self.name, self.position, self.effort = name or [], position or [], effort or []


def ros_modules():
    names = ['rospy', 'sensor_msgs', 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg',
             'std_srvs', 'std_srvs.srv', 'geometry_msgs', 'geometry_msgs.msg',
             'visualization_msgs', 'visualization_msgs.msg', 'spinal', 'spinal.msg',
             'spinal.srv', 'dracomancer.srv', 'controller_manager_msgs', 'controller_manager_msgs.srv']
    names += ['nav_msgs','nav_msgs.msg','aerial_robot_msgs','aerial_robot_msgs.msg']
    modules = {name: MagicMock() for name in names}
    modules['rospy'].Time = Stamp
    modules['rospy'].get_param.side_effect = lambda name, default=None: default
    modules['sensor_msgs.msg'].JointState = JointState
    return modules


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RuntimeTest(unittest.TestCase):
    def setUp(self):
        self.modules = ros_modules()
        patcher = patch.dict(sys.modules, self.modules)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.rospy = self.modules['rospy']
        self.direct = load('runtime_direct', PACKAGE / 'scripts/mkii/direct.py')
        self.control = load('runtime_shape', PACKAGE / 'scripts/control/control_joint_angle.py')
        self.simulation = load('runtime_simulation', PACKAGE / 'scripts/mkii/control.py')
        self.observe = load('runtime_observe', PACKAGE / 'scripts/mkii/observe.py')

    def observer_node(self, bus_ids=None):
        names = [arm+'_joint_'+str(i) for arm in ('right', 'left') for i in range(1, 8)]
        self.rospy.get_param.side_effect = lambda name, default=None: names if name == 'joints' else default
        servos = [SimpleNamespace(id=i, external_encoder_flag=0, joint_resolution=1, servo_resolution=1)
                  for i in (bus_ids if bus_ids is not None else range(1, 8))]
        self.rospy.ServiceProxy.return_value.return_value.boards = [SimpleNamespace(slave_id=0, servos=servos)]
        self.rospy.Publisher.side_effect = [MagicMock(), MagicMock()]
        self.modules['std_msgs.msg'].String.side_effect = lambda data: SimpleNamespace(data=data)
        return self.observe.Observer()

    @staticmethod
    def observer_sample(ticks, stamp=9.9, errors=None):
        return SimpleNamespace(stamp=Stamp(stamp), servos=[
            SimpleNamespace(index=i, angle=tick, error=(errors or {}).get(i, 0))
            for i, tick in enumerate(ticks)])

    def test_observer_discovers_scan_indexes_and_publishes_only_display_data(self):
        node = self.observer_node([7, 6, 5, 4, 3, 2, 1])
        self.assertEqual(node.indexes, [6, 5, 4, 3, 2, 1, 0])
        self.assertEqual([call.args[0] for call in self.rospy.Publisher.call_args_list],
                         ['joint_states', 'preview/status'])
        self.rospy.ServiceProxy.assert_called_once_with('/get_board_info', self.observe.GetBoardInfo)
        self.rospy.Service.assert_not_called()
        self.assertEqual(self.rospy.Subscriber.call_args.args[0], '/servo/states')

    def test_observer_starts_at_zero_and_tracks_relative_angles(self):
        node = self.observer_node([7, 6, 5, 4, 3, 2, 1])
        node.states(self.observer_sample([100]*7))
        self.assertEqual(node.publisher.publish.call_args.args[0].position, [0.]*14)
        node.states(self.observer_sample([100]*6+[1124], stamp=10))
        state = node.publisher.publish.call_args.args[0]
        self.assertAlmostEqual(state.position[0], np.pi/2)
        self.assertEqual(state.position[1:], [0.]*13)
        self.assertEqual(state.header.stamp, Stamp(10))

    def test_observer_rejects_missing_duplicate_stale_and_replayed_samples(self):
        node = self.observer_node()
        duplicate = self.observer_sample([100]*7)
        duplicate.servos[-1].index = 0
        for sample in (self.observer_sample([100]*6), duplicate,
                       self.observer_sample([100]*7, stamp=9),
                       self.observer_sample([100]*7, stamp=11)):
            node.states(sample)
        self.assertIsNone(node.zero)
        node.publisher.publish.assert_not_called()
        node.states(self.observer_sample([100]*7))
        node.publisher.publish.reset_mock()
        node.states(self.observer_sample([200]*7))
        node.publisher.publish.assert_not_called()

    def test_observer_unwraps_signed_counts_reports_errors_and_freezes_when_stale(self):
        node = self.observer_node()
        node.states(self.observer_sample([32760]*7))
        node.states(self.observer_sample([-32760]*7, stamp=10, errors={4: 64}))
        self.assertAlmostEqual(node.publisher.publish.call_args.args[0].position[0], 16*2*np.pi/4096)
        self.assertIn('5: 64', node.status.publish.call_args.args[0].data)
        node.publisher.publish.reset_mock()
        with patch.object(self.observe.time, 'monotonic', return_value=node.last_received+1):
            node.watchdog(None)
        self.assertIn('stale', node.status.publish.call_args.args[0].data)
        node.publisher.publish.assert_not_called()

    def test_observer_rejects_missing_or_duplicate_bus_ids_before_output(self):
        for ids in ([0, 1, 2, 3, 4, 5, 6], [1, 2, 3, 4, 5, 6, 6]):
            with self.subTest(ids=ids), self.assertRaisesRegex(ValueError, 'missing or duplicated'):
                self.observer_node(ids)
        self.rospy.Publisher.assert_not_called()

    def test_installed_shape_entrypoint_resolves_shared_helpers(self):
        package_resolver = MagicMock()
        package_resolver.RosPack.return_value.get_path.return_value = str(PACKAGE)
        source = PACKAGE / 'scripts/control/control_joint_angle.py'
        with tempfile.TemporaryDirectory() as directory:
            installed = Path(directory) / 'control_joint_angle.py'
            installed.write_bytes(source.read_bytes())
            search_path = [p for p in sys.path if Path(p).resolve() != source.parent.resolve()]
            with patch.dict(sys.modules, {'rospkg': package_resolver}), patch.object(sys, 'path', search_path):
                sys.modules.pop('shape_mapping', None)
                sys.modules.pop('shape_runtime', None)
                module = load('installed_shape', installed)
                self.assertEqual(module.helper_dir, source.parent)
                self.assertEqual(Path(sys.modules['shape_runtime'].__file__).parent, source.parent)
                package_resolver.RosPack.return_value.get_path.assert_called_once_with('dracomancer')

    def test_gazebo_forwards_original_measurement_time(self):
        n = self.simulation.Control.__new__(self.simulation.Control)
        n.lock = threading.RLock()
        n.position, n.last_state = {'right_joint_1': 0.}, {}
        n.measured_pub = MagicMock()
        msg = JointState(name=['right_joint_1'], position=[.2])
        msg.header.stamp = Stamp(9.9)
        n.states(msg)
        self.assertEqual(n.position['right_joint_1'], .2)
        self.assertIs(n.measured_pub.publish.call_args.args[0], msg)

    def test_gazebo_invalid_sample_is_not_forwarded(self):
        n = self.simulation.Control.__new__(self.simulation.Control)
        n.measured_pub = MagicMock()
        n.states(JointState(name=['right_joint_1'], position=[np.nan]))
        n.measured_pub.publish.assert_not_called()

    def measured_node(self):
        n = self.direct.SpinalControl.__new__(self.direct.SpinalControl)
        n.names = ['right_joint_%d' % i for i in range(1, 8)] + ['left_joint_1']
        n.config = {'right_joint_1': {}, 'right_joint_2': {}}
        n.calibrated = True
        n.position = dict.fromkeys(n.names, .2)
        n.measurement_stamp = {'right_joint_1': Stamp(9.9), 'right_joint_2': Stamp(9.95)}
        n.fresh = lambda name: True
        return n

    def test_placeholders_are_not_measurements_and_time_is_preserved(self):
        n = self.measured_node()
        first, second = n.measured_state(), n.measured_state()
        self.assertEqual(first.name, ['right_joint_1', 'right_joint_2'])
        self.assertEqual(first.header.stamp, Stamp(9.9))
        self.assertEqual(second.header.stamp, first.header.stamp)
        self.assertNotIn('left_joint_1', first.name)

    def test_uncalibrated_or_stale_axes_are_not_shape_input(self):
        n = self.measured_node()
        n.calibrated = False
        msg = n.measured_state()
        self.assertEqual(msg.name, [])
        self.assertEqual(msg.header.stamp, Stamp(0))
        n.calibrated = True
        n.fresh = lambda name: name == 'right_joint_1'
        self.assertEqual(n.measured_state().name, ['right_joint_1'])

    def test_real_sample_time_survives_servo_conversion(self):
        n = self.measured_node()
        n.config = {'right_joint_1': dict(index=2, zero_tick=2048, radians_per_tick=.01, direction=-1)}
        n.lock, n.error, n.last_state = threading.RLock(), {}, {}
        n.measurement_stamp['right_joint_1'] = Stamp()
        sample = SimpleNamespace(stamp=Stamp(9.9), servos=[SimpleNamespace(index=2, angle=2050, error=0)])
        n.states(sample)
        self.assertAlmostEqual(n.position['right_joint_1'], -.02)
        self.assertEqual(n.measurement_stamp['right_joint_1'], sample.stamp)
        n.states(SimpleNamespace(stamp=Stamp(0), servos=sample.servos))
        self.assertEqual(n.measurement_stamp['right_joint_1'], sample.stamp)

    def test_conflicting_direct_writer_fails_before_any_output(self):
        cfg = dict(index=0, id=7, zero_tick=2048, radians_per_tick=.0015, direction=1,
                   joint_nm_per_amp=2., max_current_amp=.5, max_torque_nm=.2)
        params = {'joints': ['right_joint_1'], '~calibrated': True, '~actuators': {'right_joint_1': cfg}}
        self.rospy.get_param.side_effect = lambda name, default=None: params.get(name, default)
        with patch.object(self.direct, 'claim_output', side_effect=RuntimeError('owned')):
            with self.assertRaisesRegex(RuntimeError, 'owned'):
                self.direct.SpinalControl()
        self.rospy.Publisher.assert_not_called()
        self.rospy.on_shutdown.assert_not_called()

    def test_configured_bus_ids_do_not_replace_scan_indexes(self):
        settings = yaml.safe_load((PACKAGE / 'config/mkii/hardware.yaml').read_text(encoding='utf-8'))
        common = dict(zero_tick=2048, radians_per_tick=.0015, direction=1,
                      joint_nm_per_amp=2., max_current_amp=.5, max_torque_nm=.2)
        params = {'joints': list(settings['servo_ids']), '~servo_ids': settings['servo_ids'],
                  '~actuators': {'right_joint_7': dict(common, index=2),
                                 'left_joint_7': dict(common, index=3)}}
        self.rospy.get_param.side_effect = lambda name, default=None: params.get(name, default)
        with patch.object(self.direct, 'DaemonExecutor'), patch.object(self.direct.threading, 'Thread'):
            node = self.direct.SpinalControl()
        self.assertEqual(node.config['right_joint_7']['id'], 7)
        self.assertEqual(node.config['left_joint_7']['id'], 17)
        self.assertEqual(node.config['right_joint_7']['index'], 2)
        self.assertEqual(node.config['left_joint_7']['index'], 3)
        self.assertFalse(node.calibrated)

    def test_dual_routes_duplicate_scan_indexes_by_bus_and_checks_skew(self):
        settings = yaml.safe_load((PACKAGE / 'config/mkii/hardware.yaml').read_text())
        ids = settings['servo_ids']
        right, left = 'right_joint_1', 'left_joint_1'
        common = dict(index=0, zero_tick=100, radians_per_tick=.01, direction=1,
                      joint_nm_per_amp=2., max_current_amp=.5, max_torque_nm=.2)
        params = {'joints': list(ids), '~servo_ids': ids, '~dual_spinal': True,
                  '~actuators': {right: common, left: common}}
        self.rospy.get_param.side_effect = lambda name, default=None: params.get(name, default)
        services = {}
        for bus, first in [('right', 1), ('left', 11)]:
            board = SimpleNamespace(slave_id=0, servos=[SimpleNamespace(id=i) for i in range(first, first+7)])
            services['/'+bus+'/get_board_info'] = MagicMock(return_value=SimpleNamespace(boards=[board]))
        self.rospy.ServiceProxy.side_effect = lambda path, _: services.get(path, MagicMock())
        self.rospy.Publisher.side_effect = lambda *args, **kwargs: MagicMock()
        self.modules['spinal.msg'].DirectServoCurrent.side_effect = lambda **fields: SimpleNamespace(**fields)
        with patch.object(self.direct, 'DaemonExecutor'), patch.object(self.direct.threading, 'Thread'):
            node = self.direct.SpinalControl()
        self.assertEqual([call.args[0] for call in self.rospy.Subscriber.call_args_list][:2],
                         ['/right/servo/states', '/left/servo/states'])
        node.calibrated = True
        node.states(self.observer_sample([150], 9.91), 'right')
        node.states(self.observer_sample([220], 9.98), 'left')
        self.assertEqual(node.measured_state().name, [left])
        node.states(self.observer_sample([160], 9.97), 'right')
        node.states(self.observer_sample([999], 9.96), 'right')
        self.assertEqual(node.measured_state().name, [right, left])
        self.assertAlmostEqual(node.position[right], .6)
        self.assertAlmostEqual(node.position[left], 1.2)
        self.assertEqual(node.measured_state().header.stamp, Stamp(9.97))
        node.token.update({right: 99, left: 80})
        node.raw.update({right: 12, left: -9})
        node.publish_current([right, left])
        for bus, value in [('right', 12), ('left', -9)]:
            packet = node.current_pub[bus].publish.call_args.args[0]
            self.assertEqual(packet.index, [0])
            self.assertEqual(packet.current, [value])

    def test_removed_mki_mapping_is_rejected(self):
        self.rospy.get_param.side_effect = lambda name, default=None: 'distal' if name == '~mapping_mode' else default
        with self.assertRaisesRegex(ValueError, 'Mk-I mappings were removed'):
            self.control.ControlJoints()
        self.rospy.Publisher.assert_not_called()

    def test_ten_direct_axes_and_scan_index_boundary(self):
        settings = yaml.safe_load((PACKAGE/'config/mkii/hardware.yaml').read_text(encoding='utf-8'))
        common = dict(zero_tick=2048,radians_per_tick=.0015,direction=1,
                      joint_nm_per_amp=2.,max_current_amp=.5,max_torque_nm=.2)
        names = list(settings['servo_ids'])[:10]
        actuators = {name:dict(common,index=i) for i,name in enumerate(names)}
        params = {'joints':names,'~servo_ids':settings['servo_ids'],'~actuators':actuators}
        self.rospy.get_param.side_effect = lambda name,default=None:params.get(name,default)
        with patch.object(self.direct,'DaemonExecutor'),patch.object(self.direct.threading,'Thread'):
            node = self.direct.SpinalControl()
        self.assertEqual(len(node.config),10)
        self.assertEqual(node.config[names[-1]]['index'],9)
        actuators[names[-1]]['index']=10
        with self.assertRaisesRegex(ValueError,'index/ID'):
            self.direct.SpinalControl()

    def test_excess_direct_axes_are_rejected_before_output(self):
        names = ['joint_'+str(i) for i in range(11)]
        params = {'joints': names, '~actuators': dict.fromkeys(names, {})}
        self.rospy.get_param.side_effect = lambda name, default=None: params.get(name, default)
        with self.assertRaisesRegex(ValueError, 'at most 10'):
            self.direct.SpinalControl()
        self.rospy.Publisher.assert_not_called()
        self.rospy.on_shutdown.assert_not_called()

    def test_shape_is_default_and_clutch_holds_last_command(self):
        n = self.control.ControlJoints()
        self.assertEqual(n.mapping_mode, 'shape')
        n.current_target = [.4]*6
        n.shape_gated_target = MagicMock()
        self.assertEqual(n.desired_target(), [.4]*6)
        n.shape_gated_target.assert_not_called()

    def test_rate_limit_does_not_catch_up_after_pause(self):
        n = self.control.ControlJoints.__new__(self.control.ControlJoints)
        n.current_target, n.joint_velocity_limits = [0]*6, [.1]*6
        n.rate_hz, n.max_step = 40., .015
        for previous, bound in [(None, 0), (Stamp(9), .0025), (Stamp(10), 0), (Stamp(11), 0)]:
            n.last_command_stamp = previous
            self.assertTrue(np.all(np.abs(n.rate_limit([1]*6)) <= bound+1e-12))

    def test_ground_flight_state_preserves_demo_rate_limit_history(self):
        n = self.control.ControlJoints.__new__(self.control.ControlJoints)
        n.hover_flight_state = 5
        ground = SimpleNamespace(data=0)
        for hover_required in (False, True):
            with self.subTest(hover_required=hover_required):
                n.publish_only_when_hovering = hover_required
                n.last_command_stamp = Stamp(9.975)
                n.robot_flight_state_cb(ground)
                self.assertFalse(n.robot_hovering)
                self.assertEqual(n.last_command_stamp, None if hover_required else Stamp(9.975))

    def test_recheck_rejection_requires_recovery_next_cycle(self):
        n = self.control.ControlJoints.__new__(self.control.ControlJoints)
        n.enable_feasibility_gate = n.teleop_mode = True
        n.current_target = [.2]*6
        n.shape_result = {'ok': True}
        n.target_fc_f_pub = n.target_fc_t_pub = MagicMock()
        n.feasibility_hysteresis_holding = False
        n.shape_validated_candidate = [.4]*6
        n.evaluate_feasibility = MagicMock(return_value=(False, .01, .01, False))
        self.assertEqual(n.shape_check_command([.21]*6), [.2]*6)
        self.assertTrue(n.feasibility_hysteresis_holding)
        self.assertIsNone(n.shape_validated_candidate)

        # A new reference passes hard limits but not recovery limits: it must
        # stay held after the previous cycle's rate-limited trial was rejected.
        n.mapped_target = MagicMock(return_value=[.4]*6)
        n.publish_candidate_target = n.publish_candidate_fc = n.log_gate = MagicMock()
        n.shape_result = {'ok': True}
        n.feasibility_rate, n.last_feasibility_eval_stamp = 0, Stamp()
        n.force_radius_recover_threshold = n.torque_radius_recover_threshold = .3
        n.evaluate_feasibility.return_value = (True, .2, .2, True)
        self.assertEqual(n.shape_gated_target(), [.2]*6)
        n.evaluate_feasibility.return_value = (True, .4, .4, True)
        self.assertEqual(n.shape_gated_target(), [.4]*6)
        self.assertFalse(n.feasibility_hysteresis_holding)

    def test_failed_mapping_enters_hold_and_invalidates_cached_goal(self):
        n = self.control.ControlJoints.__new__(self.control.ControlJoints)
        n.current_target = [.2]*6
        n.shape_result = {'ok': False}
        n.shape_validated_candidate = [.4]*6
        n.feasibility_hysteresis_holding = False
        n.mapped_target = MagicMock(return_value=[.4]*6)
        n.publish_candidate_target = MagicMock()
        self.assertEqual(n.shape_gated_target(), [.2]*6)
        self.assertTrue(n.feasibility_hysteresis_holding)
        self.assertIsNone(n.shape_validated_candidate)

    def test_nonfinite_feasibility_is_unavailable(self):
        n = self.control.ControlJoints.__new__(self.control.ControlJoints)
        n.joint_names = self.control.DRAGON_JOINTS
        n.feasibility_srv = MagicMock()
        for force in [np.inf, np.nan]:
            n.feasibility_srv.call.return_value = SimpleNamespace(valid=True, stability_ok=True, fc_f_min=force, fc_t_min=1.)
            self.assertEqual(n.evaluate_feasibility([0.]*6), (None, None, None, None))

    def test_threshold_update_is_atomic_and_strict(self):
        n = self.control.ControlJoints.__new__(self.control.ControlJoints)
        n.force_radius_threshold, n.force_radius_recover_threshold = .1, .3
        for pair in [[.2, .2], [-.1, .4], [np.nan, .4], [.2, np.inf]]:
            n.force_threshold_cb(SimpleNamespace(data=pair))
            self.assertEqual((n.force_radius_threshold, n.force_radius_recover_threshold), (.1, .3))
        n.force_threshold_cb(SimpleNamespace(data=[.15, .4]))
        self.assertEqual((n.force_radius_threshold, n.force_radius_recover_threshold), (.15, .4))

    def test_all_four_modes_are_selectable(self):
        for mode in ['shape','joint','dual_shape','dual_joint']:
            self.rospy.get_param.side_effect = lambda name,default=None:mode if name=='~mapping_mode' else default
            self.assertEqual(self.control.ControlJoints().mapping_mode,mode)

    def test_command_recheck_reuses_gimbal_and_orientation_context(self):
        n = self.control.ControlJoints.__new__(self.control.ControlJoints)
        n.joint_names = self.control.DRAGON_JOINTS
        n.feasibility_context = None
        n.force_radius_threshold=n.torque_radius_threshold=.1
        n.feasibility_srv=MagicMock()
        response=SimpleNamespace(valid=True,stability_ok=True,fc_f_min=.4,fc_t_min=.4,
                                 evaluation_gimbals=[.2]*8,evaluation_rotation=np.eye(3).ravel().tolist(),
                                 fc_to_cog=np.eye(3).ravel().tolist())
        n.feasibility_srv.call.return_value=response
        self.assertTrue(n.evaluate_feasibility([.2]*6)[0])
        n.evaluate_feasibility([.21]*6)
        request=n.feasibility_srv.call.call_args.args[0]
        self.assertEqual(request.evaluation_gimbals,[.2]*8)
        self.assertEqual(request.evaluation_rotation,response.evaluation_rotation)

    def test_cached_solution_freezes_human_input_and_mapping_history(self):
        n=self.control.ControlJoints.__new__(self.control.ControlJoints)
        n.current_target=[0.]*6
        n.latest_device_joints={'a':.1}
        n.latest_robot_joint_stamp=Stamp(10)
        n.shape_input_stamp=Stamp(10)
        n.shape_robot_valid,n.shape_timeout=True,.5
        n.shape_input_ready=MagicMock(return_value=True)
        n.shape_mapper=MagicMock()
        n.shape_mapper.human_shape.return_value=np.zeros((20,3))
        n.shape_mapper.budget=10.
        n.shape_mapper.solve.return_value=dict(q=np.full(6,.2),ok=True,status='success')
        n.shape_cached=n.shape_solution=n.shape_solve_stamp=None
        n.shape_update_rate=10.
        n.shape_target()
        n.latest_device_joints={'a':.4}
        n.current_target=[.1]*6
        n.shape_cached=None
        with patch.object(Stamp,'now',return_value=Stamp(10.01)):
            n.shape_target()
        n.shape_mapper.solve.assert_called_once()
        self.assertEqual(n.shape_solution_human,{'a':.1})
        self.assertEqual(n.shape_solution_previous,[0.]*6)

    def test_feedback_consumes_named_nominal_packet_and_rejects_stale(self):
        feedback=load('runtime_feedback',PACKAGE/'scripts/control/feedback.py')
        n=feedback.Feedback.__new__(feedback.Feedback)
        n.names=[f'left_joint_{i}' for i in range(1,8)]+[f'right_joint_{i}' for i in range(1,8)]
        n.mapper=SimpleNamespace(human=SimpleNamespace(names=n.names),
                                 mapping_jacobian=lambda *args:np.eye(6,14))
        n.contact_model=SimpleNamespace(jacobian=lambda *args,**kwargs:np.eye(6))
        n.contact=feedback.ContactFeedback(6,14,0.)
        n.safety=feedback.SafetyFeedback(np.ones(6),np.zeros(14))
        n.limit=np.ones(14)
        n.anchor,n.input_previous='none',None
        n.received=feedback.time.monotonic()
        n.data=dict(active=True,ok=True,stamp=10.,input_stamp=10.,human=dict.fromkeys(n.names,0.),
                    robot={},q_ref=[.2]*6,q_previous=[0.]*6,q_command=[0.]*6)
        np.testing.assert_allclose(n.torque(),[-.2]*6+[0.]*8)
        n.data['stamp']=8.
        with self.assertRaises(ValueError):n.torque()

    def test_anchor_uses_sent_shape_and_preserves_world_reference(self):
        anchor=load('runtime_anchor',PACKAGE/'scripts/control/anchor_runtime.py')
        n=anchor.AnchorRuntime.__new__(anchor.AnchorRuntime)
        n.anchor_mode,n.anchor_reference,n.anchor_last='tail',None,None
        n.anchor_tail=[.474,0,0]
        n.rate_hz,n.pos_limit=40.,False
        n.anchor_received=n.odom_received=anchor.time.monotonic()
        names=self.control.DRAGON_JOINTS
        n.anchor_data=dict(active=True,stamp=10.,fc_to_cog=np.eye(3).ravel().tolist(),
                           robot=dict.fromkeys(names,.1),q_command=[.2]*6)
        pose=SimpleNamespace(position=SimpleNamespace(x=2.,y=0.,z=1.),
                             orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
        n.anchor_odom=SimpleNamespace(header=SimpleNamespace(stamp=Stamp(10)),pose=SimpleNamespace(pose=pose))
        n.anchor_base_odom=n.anchor_odom
        n.base_received=anchor.time.monotonic()
        def point(positions,*args):
            result=np.eye(4);result[0,3]=positions[names[0]];return result
        n.anchor_model=SimpleNamespace(required_joints=set(names),point_pose=point,
                                       joints=[('fc_joint','fixed','link2','fc',np.eye(4),[1,0,0])],
                                       transforms=lambda values:{'link2':np.eye(4),'fc':np.eye(4)})
        nav=SimpleNamespace(COG=1,POS_MODE=2,target_vel_x=0.,target_vel_y=0.,target_vel_z=0.)
        self.assertTrue(n.anchor_navigation(nav))
        self.assertAlmostEqual(nav.target_pos_x,1.9)
        n.anchor_data['q_command']=[.4]*6
        self.assertTrue(n.anchor_navigation(nav))
        self.assertAlmostEqual(nav.target_pos_x,1.7)

    def test_multiple_contacts_require_fresh_grasp_validation(self):
        from contacts import ContactInputs
        n=self.control.ControlJoints.__new__(self.control.ControlJoints)
        n.use_contact_wrench=True
        n.contacts=ContactInputs([{'link':'link1','topic':'/head'},
                                  {'link':'link4','topic':'/tail','point':[.474,0,0]}],'dragon')
        for i,link in enumerate(['link1','link4']):
            force=SimpleNamespace(x=1.-2*i,y=0.,z=0.)
            torque=SimpleNamespace(x=0.,y=0.,z=0.)
            msg=SimpleNamespace(header=SimpleNamespace(stamp=Stamp(10),frame_id='dragon/'+link),
                                wrench=SimpleNamespace(force=force,torque=torque))
            n.contacts.update(i,msg,self.control.time.monotonic())
        n.grasp_ok=False
        n.grasp_received=self.control.time.monotonic()
        n.latest_robot_joints={}
        n.contact_geometry=SimpleNamespace(required_joints=set(),wrench_in_fc=lambda positions,wrench,link:wrench)
        with self.assertRaises(ValueError):n.candidate_wrench([0.]*6)
        n.grasp_ok=True
        links,points,wrenches,local=n.candidate_wrench([0.]*6)
        self.assertEqual(links,['link1','link4'])
        self.assertEqual(len(points),6)
        self.assertEqual(wrenches[0],-wrenches[6])
        self.assertEqual(local,[False,False])

    def test_view_navigation_uses_full_cog_rotation_interface(self):
        position=load('runtime_position',PACKAGE/'scripts/control/control_position.py')
        position.FlightNav=SimpleNamespace(VEL_MODE=1,NO_NAVIGATION=0)
        n=position.ControlPose.__new__(position.ControlPose)
        n.flight_nav=SimpleNamespace()
        n.view=position.ViewControl([0,0,0])
        pose=SimpleNamespace(position=SimpleNamespace(x=1.,y=0.,z=.5),
                             orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
        n.anchor_odom=SimpleNamespace(header=SimpleNamespace(stamp=Stamp(10)),pose=SimpleNamespace(pose=pose))
        n.odom_received=n.last_joy_time=position.time.monotonic()
        n.operator_dynamic=False
        n.allow_joystick=True
        n.view_last=None
        n.rate_hz,n.joy_timeout=40.,.1
        n.latest_axes=[.5,0.]
        n.axis_x,n.axis_y,n.axis_z=0,1,2
        n.scale_x=n.scale_y=n.scale_z=1.
        n.deadzone=.05
        n.view_dual,n.view_require_imu,n.pos_limit=False,False,False
        n.anchor_mode='none'
        n.imu_ready=MagicMock(return_value=False)
        n.view_attitude_limits=[[-.5,.5],[-.5,.5],[-3.,3.]]
        nav=n.view_nav()
        self.assertAlmostEqual(nav.target_vel_y,-.15)
        self.assertEqual(nav.roll_nav_mode,0)
        self.assertEqual(nav.pitch_nav_mode,0)
        n.rotation_pub=MagicMock()
        n.publish_view_attitude()
        msg=n.rotation_pub.publish.call_args.args[0]
        self.assertEqual(msg.header.frame_id,'cog')
        self.assertAlmostEqual(msg.pose.pose.orientation.w,1.)


if __name__ == '__main__':
    unittest.main()
