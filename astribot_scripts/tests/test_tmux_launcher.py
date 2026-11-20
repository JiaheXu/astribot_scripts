"""Exercise the real tmux lifecycle with inert sensor/ROS command doubles."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
import uuid


@unittest.skipUnless(shutil.which('tmux'), 'tmux is not installed')
class TmuxLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='astribot tmux ')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'sdk with spaces $(false)'
        scripts = self.root / 'astribot_scripts'
        (scripts / 'config').mkdir(parents=True)
        original = Path(__file__).resolve().parents[1]
        self.socket = 'astribot-test-' + uuid.uuid4().hex
        self.script = scripts / 'launch_all_tmux.sh'
        self.script.write_text((original / self.script.name).read_text().replace(
            'tmux -L astribot-lio', 'tmux -L ' + self.socket))
        shutil.copy(original / 'config/astribot.tmux.conf', scripts / 'config')
        self.addCleanup(lambda: subprocess.run(['tmux', '-L', self.socket, 'kill-server'],
                                               capture_output=True))
        self.log = self.root / 'commands.jsonl'
        binary = self.root / 'bin'
        binary.mkdir()
        package = self.root / 'fake package'
        (package / 'lib/fast_lio').mkdir(parents=True)
        mapping = package / 'lib/fast_lio/fastlio_mapping'
        mapping.touch()
        mapping.chmod(0o755)
        # These stand-ins never import the SDK or create a ROS node.
        stub = '''#!__PYTHON__
import json, os, signal, sys
from pathlib import Path
args = sys.argv[1:]
if args[:2] == ['pkg', 'prefix']:
    print(os.environ['TEST_PREFIX'])
    sys.exit(0)
if '--help' in args or '--show-args' in args:
    sys.exit(0)
def record(event):
    with open(os.environ['TEST_LOG'], 'a') as output:
        output.write(json.dumps({'event': event, 'args': args,
                                 'env': os.environ.get('TEST_SOURCED')}) + '\\n')
def stop(signum, frame):
    record('stop')
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
record('start')
while True:
    signal.pause()
'''.replace('__PYTHON__', sys.executable)
        for name in ('ros2', 'python3'):
            path = binary / name
            path.write_text(stub)
            path.chmod(0o755)
        setup = self.root / 'ros_setup.bash'
        setup.write_text('export TEST_SOURCED=yes\n')
        (self.root / 'env.sh').write_text(':\n')
        workspace = self.root / 'workspace'
        (workspace / 'install').mkdir(parents=True)
        (workspace / 'install/local_setup.bash').write_text(':\n')
        self.env = {**os.environ, 'PATH': str(binary) + os.pathsep + os.environ['PATH'],
                    'ROS_SETUP': str(setup), 'ROS2_WS': str(workspace),
                    'TEST_PREFIX': str(package), 'TEST_LOG': str(self.log)}
        self.env.pop('DISPLAY', None)
        self.env.pop('WAYLAND_DISPLAY', None)

    def run_launcher(self, *args, check=True):
        return subprocess.run(['bash', str(self.script), *args], env=self.env,
                              capture_output=True, text=True, check=check, timeout=25)

    def records(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def test_headless_start_duplicate_prevention_and_graceful_stop(self):
        self.run_launcher('--headless', '--detach')
        deadline = time.monotonic() + 5
        while len(self.records()) < 3 and time.monotonic() < deadline:
            time.sleep(0.05)
        records = self.records()
        self.assertEqual(len(records), 3, records)
        self.assertTrue(all(r['env'] == 'yes' for r in records))
        sensor = next(r['args'] for r in records if r['args'][0].endswith('publish_robot_data.py'))
        self.assertEqual(sensor[1:], ['--observe-only', '--cameras', 'head_rgbd', 'torso_rgbd',
                                     'left_wrist_rgbd', 'right_wrist_rgbd', '--no-depth',
                                     '--color-encoding', 'rgb8', '--image-rate', '3', '--image-size', '640', '480'])
        lio = next(r['args'] for r in records if any('astribot_fast_lio.launch.py' in a for a in r['args']))
        self.assertIn('sensor:=rear', lio)
        self.assertIn('rviz:=false', lio)
        model = next(r['args'] for r in records if any('astribot_lio_rviz.launch.py' in a for a in r['args']))
        self.assertIn('rviz:=false', model)
        self.assertEqual(self.run_launcher('status').stdout.count('dead=0'), 3)
        self.assertIn('already exists', self.run_launcher('--headless', '--detach').stdout)
        self.assertEqual(len(self.records()), 3)
        self.run_launcher('stop')
        self.assertEqual(sum(r['event'] == 'stop' for r in self.records()), 3)
        self.assertNotEqual(self.run_launcher('status', check=False).returncode, 0)

    def test_check_and_missing_display_never_start_processes(self):
        self.assertIn('checks passed', self.run_launcher('check').stdout)
        result = self.run_launcher('--detach', check=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('No desktop display', result.stderr)
        self.assertEqual(self.records(), [])


if __name__ == '__main__':
    unittest.main()
