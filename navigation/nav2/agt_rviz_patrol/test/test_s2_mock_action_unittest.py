"""Isolated mock FollowRoute test: never inject stub ROS modules into pytest."""
import os
import subprocess
import sys
import unittest
from pathlib import Path


class MockActionIsolated(unittest.TestCase):
    def test_real_workbench_callback_lifecycle_in_subprocess(self):
        script = Path(__file__).parent / '_s2_mock_action_harness.py'
        source = Path(__file__).parent.parent
        env = dict(os.environ)
        env['PYTHONPATH'] = str(source) + os.pathsep + env.get('PYTHONPATH', '')
        proc = subprocess.run([sys.executable, str(script)], capture_output=True,
                              text=True, env=env, timeout=30, check=False)
        self.assertEqual(proc.returncode, 0, proc.stdout+'\n'+proc.stderr)
        self.assertIn('Ran 9 tests', proc.stderr)


if __name__ == '__main__':
    unittest.main()
