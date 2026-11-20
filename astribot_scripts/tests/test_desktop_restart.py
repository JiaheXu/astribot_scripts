"""Desktop lifecycle ownership and click serialization, without robot access."""
import fcntl
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

from astribot_scripts import restart_astribot as restart
from astribot_scripts import install_desktop_launcher as install
from astribot_scripts.check_stack_ready import EXPECTED, missing_streams


class DesktopRestartTests(unittest.TestCase):
    def test_counts_only_session_descendants(self):
        rows = '''10 1 bash
11 10 python3 /sdk/astribot_scripts/publish_robot_data.py --observe-only
20 1 bash
21 20 ros2 launch mapping
22 21 /ros/lib/fast_lio/fastlio_mapping --ros-args
30 1 bash
31 30 ros2 launch visualization
32 31 /ros/lib/rviz2/rviz2 -d config.rviz
40 1 /ros/lib/rviz2/rviz2
41 1 python3 /elsewhere/publish_robot_data.py
'''
        self.assertEqual(restart.stack_processes({10, 20, 30}, rows),
                         {'sensors': 1, 'mapping': 1, 'rviz': 1})

    def test_second_click_does_not_stop_or_launch_anything(self):
        with tempfile.TemporaryDirectory() as directory:
            state = Path(directory)/'astribot'
            state.mkdir()
            with (state/'restart.lock').open('w') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                with patch.dict('os.environ', {'XDG_STATE_HOME': directory}), \
                        patch.object(restart, 'notify') as notify, \
                        patch.object(restart.subprocess, 'run') as run:
                    self.assertEqual(restart.main(), 0)
                    run.assert_not_called()
                    notify.assert_called_once()

    def test_desktop_notification_timeout_is_nonfatal(self):
        with patch.object(restart.shutil, 'which', return_value='/usr/bin/notify-send'), \
                patch.object(restart.subprocess, 'run', side_effect=subprocess.TimeoutExpired('notify-send', 10)):
            restart.notify('test', 'test')

    def test_stop_does_not_require_ros_display_or_load_tmux_yaml(self):
        with tempfile.TemporaryDirectory() as directory, \
                patch.dict('os.environ', {'XDG_STATE_HOME': directory}), \
                patch.object(restart, 'notify'), \
                patch.object(restart.shutil, 'which', return_value='/usr/bin/tmux'), \
                patch.object(restart, 'stop_stack') as stop, \
                patch.object(restart, 'wait_started') as start, \
                patch.object(restart.subprocess, 'run') as run:
            self.assertEqual(restart.main('stop'), 0)
            stop.assert_called_once()
            start.assert_not_called()
            run.assert_not_called()
            self.assertIn('Stop complete', (Path(directory)/'astribot/stop.log').read_text())

    def test_stop_waits_for_an_in_progress_start_instead_of_ignoring_click(self):
        with tempfile.TemporaryDirectory() as directory, \
                patch.dict('os.environ', {'XDG_STATE_HOME': directory}), \
                patch.object(restart.fcntl, 'flock', side_effect=[BlockingIOError(), None]) as lock, \
                patch.object(restart, 'notify') as notify, \
                patch.object(restart.shutil, 'which', return_value='/usr/bin/tmux'), \
                patch.object(restart, 'stop_stack') as stop:
            self.assertEqual(restart.main('stop'), 0)
            self.assertEqual(lock.call_args_list[1].args[1], fcntl.LOCK_EX)
            self.assertEqual(notify.call_args_list[0].args[0], 'Astribot Stop queued')
            stop.assert_called_once()

    def test_stop_verifies_owned_processes_and_leaves_unrelated_nodes(self):
        before = '10 1 bash\n11 10 /ros/lib/fast_lio/fastlio_mapping\n20 1 /other/rviz2\n'
        after = '20 1 /other/rviz2\n'
        run = Mock()
        with patch.object(restart.subprocess, 'run', side_effect=[
                Mock(returncode=0, stdout='10\n'), Mock(returncode=1)]), \
                patch.object(restart.subprocess, 'check_output', side_effect=[before, after]):
            restart.stop_stack(run)
        run.assert_called_once_with(['bash', str(restart.SCRIPT_DIR/'launch_all_tmux.sh'), 'stop'], timeout=25)

    def test_stop_fails_if_a_descendant_survives_after_its_pane_exits(self):
        before = '10 1 bash\n11 10 /ros/lib/fast_lio/fastlio_mapping\n'
        after = '11 1 /ros/lib/fast_lio/fastlio_mapping\n'
        with patch.object(restart.subprocess, 'run', side_effect=[
                Mock(returncode=0, stdout='10\n'), Mock(returncode=1)]), \
                patch.object(restart.subprocess, 'check_output', side_effect=[before, after]), \
                patch.object(restart.time, 'monotonic', side_effect=[0, 6]):
            with self.assertRaisesRegex(RuntimeError, r'processes have not exited: \[11\]'):
                restart.stop_stack(Mock())

    def test_installer_adds_observation_recording_and_separate_power_buttons(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scripts = root/'sdk/astribot_scripts'
            desktop = root/'desktop'
            desktop.mkdir()
            old = desktop/'Restart Astribot.desktop'
            old.write_text(f'Exec=/usr/bin/python3 "{scripts / "restart_astribot.py"}"\n')
            obs = desktop/'OBS Studio.desktop'
            obs.write_text('unrelated')
            with patch.object(install, '__file__', str(scripts/'install_desktop_launcher.py')), \
                    patch.object(install.subprocess, 'check_output', return_value=str(desktop)+'\n'), \
                    patch.object(install.subprocess, 'run') as run:
                install.main()
            self.assertFalse(old.exists())
            self.assertEqual(obs.read_text(), 'unrelated')
            self.assertEqual(run.call_count, 10)
            for action in ('start', 'stop'):
                launcher = desktop/f'{action.title()} Astribot.desktop'
                self.assertTrue(launcher.stat().st_mode & 0o111)
                self.assertIn(f'Name={action.title()} Astribot', launcher.read_text())
                self.assertIn(f'{action}_astribot.py', launcher.read_text())
                self.assertIn(f'astribot-{action}.svg', launcher.read_text())
                self.assertTrue(any(call.args[0] == ['gio', 'set', str(launcher), 'metadata::trusted', 'true']
                                    for call in run.call_args_list))
            recorder = desktop/'Record RGB MCAP.desktop'
            self.assertTrue(recorder.stat().st_mode & 0o111)
            self.assertIn('Name=Record RGB-D MCAP', recorder.read_text())
            self.assertIn('Terminal=true', recorder.read_text())
            self.assertIn('record_raw_images.sh', recorder.read_text())
            self.assertIn('astribot-record.svg', recorder.read_text())
            self.assertTrue(any(call.args[0] == ['gio', 'set', str(recorder), 'metadata::trusted', 'true']
                                for call in run.call_args_list))
            power = desktop/'Power On & Home Robot.desktop'
            self.assertTrue(power.stat().st_mode & 0o111)
            self.assertIn('Terminal=true', power.read_text())
            self.assertIn('start_robot_power_desktop.sh', power.read_text())
            self.assertIn('astribot-power.svg', power.read_text())
            self.assertTrue(any(call.args[0] == ['gio', 'set', str(power), 'metadata::trusted', 'true']
                                for call in run.call_args_list))
            guide = desktop/'Astribot Button Guide.desktop'
            self.assertTrue(guide.stat().st_mode & 0o111)
            self.assertIn('Terminal=false', guide.read_text())
            self.assertIn('/usr/bin/xdg-open', guide.read_text())
            self.assertIn('docs/desktop-buttons.html', guide.read_text())
            self.assertIn('astribot-guide.svg', guide.read_text())
            self.assertTrue(any(call.args[0] == ['gio', 'set', str(guide), 'metadata::trusted', 'true']
                                for call in run.call_args_list))

    def test_live_readiness_rejects_missing_stale_and_single_messages(self):
        received = dict.fromkeys(EXPECTED, 10.0)
        counts = dict.fromkeys(EXPECTED, 2)
        self.assertEqual(missing_streams(received, counts, 11.0), [])
        received.pop('head_rgbd')
        received['odometry'] = 1.0
        counts['joints'] = 1
        self.assertEqual(set(missing_streams(received, counts, 11.0)),
                         {'head_rgbd', 'odometry', 'joints'})


if __name__ == '__main__':
    unittest.main()
