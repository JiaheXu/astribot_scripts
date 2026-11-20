"""Exercise the desktop wrapper with a fake power helper; never contact hardware."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class RobotPowerDesktopTests(unittest.TestCase):
    def test_log_keeps_stdout_stderr_and_original_exit_status(self):
        wrapper = Path(__file__).resolve().parents[1]/'start_robot_power_desktop.sh'
        for status in (0, 7):
            with self.subTest(status=status), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                shutil.copy2(wrapper, root/wrapper.name)
                helper = root/'robot_power.sh'
                helper.write_text('#!/usr/bin/env bash\n'
                                  'test "$1" = start && test "$2" = --execute || exit 99\n'
                                  'echo simulated-progress\necho simulated-detail >&2\n'
                                  f'exit {status}\n')
                helper.chmod(0o755)
                result = subprocess.run(['bash', str(root/wrapper.name)], input='',
                                        text=True, capture_output=True, timeout=5,
                                        env={**os.environ, 'XDG_STATE_HOME': str(root/'state')})
                self.assertEqual(result.returncode, status, result.stderr)
                logs = list((root/'state/astribot').glob('robot-start-*.log'))
                self.assertEqual(len(logs), 1)
                self.assertIn('simulated-progress', logs[0].read_text())
                self.assertIn('simulated-detail', logs[0].read_text())
                self.assertIn('successfully' if status == 0 else 'did not complete',
                              result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
