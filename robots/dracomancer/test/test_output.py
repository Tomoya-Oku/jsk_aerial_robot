#!/usr/bin/env python3
"""Direct-UART writer exclusion checks without ROS or hardware."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch


PACKAGE = Path(__file__).resolve().parents[1]
HAPTICS = PACKAGE / 'src'
sys.path.insert(0, str(HAPTICS))
from dracomancer.output_lock import OutputLock


class OutputLockTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        patcher = patch('dracomancer.output_lock.tempfile.gettempdir', return_value=self.directory.name)
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
from dracomancer import output_lock
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



if __name__ == '__main__':
    unittest.main()
