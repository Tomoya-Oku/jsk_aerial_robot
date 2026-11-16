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

    def test_removed_mki_mapping_is_rejected(self):
        self.rospy.get_param.side_effect = lambda name, default=None: 'distal' if name == '~mapping_mode' else default
        with self.assertRaisesRegex(ValueError, 'Mk-I mappings were removed'):
            self.control.ControlJoints()
        self.rospy.Publisher.assert_not_called()

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


if __name__ == '__main__':
    unittest.main()
