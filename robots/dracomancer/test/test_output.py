#!/usr/bin/env python3
"""Haptic writer exclusion and shape-mode regression checks without ROS."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import yaml
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
HAPTICS = PACKAGE / 'scripts/haptics'
sys.path.insert(0, str(HAPTICS))
from output_lock import OutputLock


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class OutputLockTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        patcher = patch('output_lock.tempfile.gettempdir', return_value=self.directory.name)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_either_topic_conflicts_and_release_allows_restart(self):
        owner = OutputLock(['/current', '/torque'], 'http://master:11311')
        try:
            for topics in (['/current', '/other'], ['/other', '/torque']):
                with self.assertRaises(RuntimeError):
                    OutputLock(topics, 'http://master:11311/')
            other_robot = OutputLock(['/other/current', '/other/torque'], 'http://master:11311')
            other_robot.close()
            other_master = OutputLock(['/current', '/torque'], 'http://other:11311')
            other_master.close()
        finally:
            owner.close()
        replacement = OutputLock(['/current', '/torque'], 'http://master:11311')
        replacement.close()

    def test_process_exit_releases_lock(self):
        program = '''
import sys
sys.path.insert(0, sys.argv[1])
import output_lock
output_lock.tempfile.gettempdir = lambda: sys.argv[2]
lock = output_lock.OutputLock(['/current'], 'http://master:11311')
print('locked', flush=True)
sys.stdin.read(1)
'''
        child = subprocess.Popen([sys.executable, '-B', '-c', program, str(HAPTICS), self.directory.name],
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        try:
            self.assertEqual(child.stdout.readline().strip(), 'locked')
            with self.assertRaises(RuntimeError):
                OutputLock(['/current'], 'http://master:11311')
        finally:
            child.communicate('x', timeout=10)
        self.assertEqual(child.returncode, 0)
        replacement = OutputLock(['/current'], 'http://master:11311')
        replacement.close()


class HapticIntegrationTest(unittest.TestCase):
    def setUp(self):
        names = ['rospy', 'rospkg', 'geometry_msgs', 'geometry_msgs.msg', 'sensor_msgs',
                 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv',
                 'spinal', 'spinal.msg', 'dracomancer', 'dracomancer.msg',
                 'rosgraph_msgs', 'rosgraph_msgs.msg']
        self.modules = {name: MagicMock() for name in names}
        self.modules['rospkg'].RosPack.return_value.get_path.return_value = str(PACKAGE)
        patcher = patch.dict(sys.modules, self.modules)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.rospy = self.modules['rospy']

    def test_shape_disables_general_contact_and_hardware(self):
        params = yaml.safe_load((PACKAGE / 'config/haptics.yaml').read_text())
        params.update(mapping_mode='shape', enable_hardware_output=True, enable_contact_feedback=True)
        self.rospy.get_param.side_effect = lambda name, default=None: params.get(name.lstrip('~'), default)
        module = load_module('output_controller_test', HAPTICS / 'haptic_controller.py')
        with patch.object(module, 'XM430CurrentOutput', return_value=SimpleNamespace(hardware_enabled=False)) as output:
            node = module.HapticController()
        self.assertFalse(node.enable_contact)
        self.assertFalse(output.call_args.args[7])

    def test_legacy_general_contact_and_hardware_are_preserved(self):
        params = yaml.safe_load((PACKAGE / 'config/haptics.yaml').read_text())
        params.update(mapping_mode='distal', enable_hardware_output=True, enable_contact_feedback=True)
        self.rospy.get_param.side_effect = lambda name, default=None: params.get(name.lstrip('~'), default)
        module = load_module('output_controller_test', HAPTICS / 'haptic_controller.py')
        with patch.object(module, 'XM430CurrentOutput', return_value=SimpleNamespace(hardware_enabled=True)) as output:
            node = module.HapticController()
        self.assertTrue(node.enable_contact)
        self.assertTrue(np.isfinite(node.mapping_jacobian).all())
        self.assertTrue(output.call_args.args[7])

    def test_rejected_general_writer_creates_no_publishers_or_shutdown(self):
        module = load_module('output_servo_test', HAPTICS / 'servo_output.py')
        with patch.object(module, 'claim_output', side_effect=RuntimeError('owned')):
            with self.assertRaises(RuntimeError):
                module.XM430CurrentOutput(['joint'], [0], [1.783], [.2], [1],
                                         '/current', '/torque', hardware_enabled=True)
        self.rospy.Publisher.assert_not_called()
        self.rospy.on_shutdown.assert_not_called()

    def test_contact_shape_rejected_before_any_output(self):
        self.rospy.get_param.return_value = 'shape'
        module = load_module('output_contact_test', PACKAGE / 'scripts/control/contact_haptics.py')
        with self.assertRaisesRegex(ValueError, 'shape mode'):
            module.ContactHaptics()
        self.rospy.Publisher.assert_not_called()
        self.rospy.on_shutdown.assert_not_called()

    def test_rejected_contact_writer_creates_no_publishers_or_shutdown(self):
        settings = yaml.safe_load((PACKAGE / 'config/contact.yaml').read_text())['haptics']
        self.rospy.get_param.side_effect = lambda name, default=None: (
            settings if name == '/dracomancer/contact/haptics' else default)
        module = load_module('output_contact_test', PACKAGE / 'scripts/control/contact_haptics.py')
        with patch.object(module, 'claim_output', side_effect=RuntimeError('owned')):
            with self.assertRaises(RuntimeError):
                module.ContactHaptics()
        self.rospy.Publisher.assert_not_called()
        self.rospy.on_shutdown.assert_not_called()

    def test_shape_never_differentiates_legacy_mapping(self):
        from test_contact import load_controller
        controller = load_controller()
        node = controller.__new__(controller)
        node.mapping_mode = 'shape'
        # No publishers, measurement packet or optimizer are available here.
        # Returning without accessing them is required in shape mode.
        node.publish_mapping_state()


if __name__ == '__main__':
    unittest.main()
