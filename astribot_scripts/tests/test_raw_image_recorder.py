"""Recorder launcher checks without a live camera or ROS graph."""
import json
import fcntl
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class RawImageRecorderTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='astribot recorder ')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)/'SDK with spaces'
        self.scripts = self.root/'astribot_scripts'
        self.scripts.mkdir(parents=True)
        original = Path(__file__).resolve().parents[1]
        self.script = self.scripts/'record_raw_images.sh'
        shutil.copy(original/self.script.name, self.script)
        shutil.copytree(original/'config', self.scripts/'config')
        (self.root/'env.sh').write_text(':\n')
        setup = self.root/'ros_setup.bash'
        setup.write_text(':\n')
        binary = self.root/'bin'
        binary.mkdir()
        ros2 = binary/'ros2'
        ros2.write_text('#!/bin/sh\n[ "$1 $2 $3" = "pkg prefix rosbag2_storage_mcap" ]\n')
        ros2.chmod(0o755)
        (self.scripts/'record_rgbd_mcap.py').write_text(
            'import json,sys\nprint(json.dumps(sys.argv[1:]))\n')
        self.env = {**os.environ, 'ROS_SETUP': str(setup),
                    'XDG_STATE_HOME': str(self.root/'state'),
                    'PATH': str(binary)+os.pathsep+os.environ['PATH']}
        notify = binary/'notify-send'
        notify.write_text('#!/bin/sh\nexit 0\n')
        notify.chmod(0o755)

    def run_script(self, *args):
        return subprocess.run(['bash', str(self.script), *args], env=self.env,
                              capture_output=True, text=True, timeout=10)

    def test_sampler_receives_exact_output_path(self):
        destination = self.root/'bags/run with spaces $(not_executed)'
        result = self.run_script(str(destination))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout.splitlines()[-1]), [str(destination)])

    def test_existing_output_is_not_overwritten(self):
        output = self.root/'existing'
        output.mkdir()
        (output/'metadata.yaml').write_text('keep me')
        result = self.run_script(str(output))
        self.assertEqual(result.returncode, 1)
        self.assertIn('Output already exists', result.stderr)
        self.assertEqual((output/'metadata.yaml').read_text(), 'keep me')

    def test_help_is_available_without_ros_setup(self):
        self.env['ROS_SETUP'] = '/does/not/exist'
        result = self.run_script('--help')
        self.assertEqual(result.returncode, 0)
        self.assertIn('4,000,000,000', result.stdout)
        self.assertEqual(self.run_script('first', 'second').returncode, 2)

    def test_second_recorder_is_rejected_before_creating_a_bag(self):
        state = self.root/'state/astribot'
        state.mkdir(parents=True)
        with (state/'raw-images-record.lock').open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            output = self.root/'second_bag'
            result = self.run_script(str(output))
        self.assertEqual(result.returncode, 1)
        self.assertIn('already running', result.stderr)
        self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()
