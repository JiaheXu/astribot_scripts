"""Offline startup checks; no real SDK connection or motion is created."""
import ast
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch


class StartupTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1] / 'startup.py'
        self.assertTrue(path.exists(), 'startup script must exist')
        spec = importlib.util.spec_from_file_location('startup_under_test', path)
        self.module = importlib.util.module_from_spec(spec)
        self.robot = Mock()
        self.robot.get_control_rights_status.return_value = True
        self.robot.astribot_interface.is_alive.return_value = True
        self.robot.move_to_home.return_value = 'move success'
        self.ros = Mock()
        sdk = Mock()
        sdk.Astribot.return_value = self.robot
        with patch.dict(sys.modules, {
            'rclpy': self.ros,
            'astribot_sdk.core.astribot_api.astribot_client': sdk,
        }):
            spec.loader.exec_module(self.module)

    def test_home_constants_match_sdk_calibration(self):
        path = Path(__file__).resolve().parents[2] / 'astribot_sdk/core/astribot_api/astribot_client.py'
        tree = ast.parse(path.read_text())
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'move_to_home')
        values = {}
        for n in method.body:
            if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name):
                if n.targets[0].id in ('joints_name_list', 'joints_command_list'):
                    values[n.targets[0].id] = ast.literal_eval(n.value)
        self.assertEqual(self.module.HOME, dict(zip(values['joints_name_list'], values['joints_command_list'])))

    def test_prints_done_only_after_measured_home_is_reached(self):
        target = list(self.module.HOME.values())
        self.robot.get_current_joints_position.side_effect = [
            [[value + 0.2 for value in row] for row in target], target]
        with patch.object(self.module.time, 'sleep'), contextlib.redirect_stdout(io.StringIO()) as output:
            result = self.module.main()
        self.assertEqual(result, 0)
        self.robot.move_to_home.assert_called_once_with(duration=10.0)
        self.assertEqual(self.robot.get_current_joints_position.call_count, 2)
        self.assertIn('Start up process is done', output.getvalue())
        self.ros.shutdown.assert_called_once()

    def test_no_control_rights_prevents_motion_and_success_log(self):
        self.robot.get_control_rights_status.return_value = False
        with contextlib.redirect_stdout(io.StringIO()) as output:
            result = self.module.main()
        self.assertEqual(result, 1)
        self.robot.move_to_home.assert_not_called()
        self.assertNotIn('Start up process is done', output.getvalue())

    def test_sdk_rejection_never_prints_done(self):
        self.robot.move_to_home.return_value = 'move failed'
        with contextlib.redirect_stdout(io.StringIO()) as output:
            result = self.module.main()
        self.assertEqual(result, 1)
        self.assertNotIn('Start up process is done', output.getvalue())

    def test_timeout_stops_motion_and_does_not_print_done(self):
        self.robot.get_current_joints_position.return_value = list(self.module.HOME.values())
        with patch.object(self.module.time, 'monotonic', side_effect=[0.0, 10.1]), \
                contextlib.redirect_stdout(io.StringIO()) as output:
            result = self.module.main()
        self.assertEqual(result, 1)
        self.robot.stop_robot.assert_called_once()
        self.assertNotIn('Start up process is done', output.getvalue())

    def test_invalid_joint_values_are_not_home(self):
        target = list(self.module.HOME.values())
        self.assertFalse(self.module.at_home([]))
        self.assertFalse(self.module.at_home([[float('nan') for _ in row] for row in target]))
        self.assertFalse(self.module.at_home([[float('inf') for _ in row] for row in target]))


if __name__ == '__main__':
    unittest.main()
