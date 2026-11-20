"""Command validation and SDK dispatch checks; no ROS install or robot needed."""
import contextlib
import io
import math
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from astribot_scripts import control_joints, control_mobile_base
from astribot_scripts.common import control


def robot_fixture():
    robot = Mock(robot_type='S1', chassis_name='astribot_chassis')
    robot.get_control_rights_status.return_value = True
    robot.astribot_interface.is_alive.return_value = True
    robot.get_desired_joints_position.side_effect = lambda names: [
        [1.0, 2.0, 0.5] if name == robot.chassis_name else
        [0.1] * control_joints.BODY_DOFS[name] for name in names]
    robot.get_joints_position_limit.return_value = (
        [[-1.0] * size for size in control_joints.BODY_DOFS.values()],
        [[1.0] * size for size in control_joints.BODY_DOFS.values()])
    return robot


def twist(x=0.0, y=0.0, yaw=0.0):
    return SimpleNamespace(linear=SimpleNamespace(x=x, y=y, z=0.0),
                           angular=SimpleNamespace(x=0.0, y=0.0, z=yaw))


def joint_command(names, positions, velocity=(), effort=()):
    return SimpleNamespace(name=names, position=positions, velocity=velocity, effort=effort)


class MobileBaseTests(unittest.TestCase):
    def setUp(self):
        self.robot = robot_fixture()
        self.receiver = control_mobile_base.MobileBaseReceiver(self.robot)

    def send(self, message, now=10.0):
        with patch.object(control_mobile_base.time, 'monotonic', return_value=now):
            self.receiver.receive(message)

    def test_idle_does_not_command_or_read_positions(self):
        self.assertEqual(self.receiver.command(10.0, 0.004), ([], []))
        self.robot.get_desired_joints_position.assert_not_called()

    def test_local_velocity_integrates_without_world_rotation_or_mutating_sdk_state(self):
        original = [[1.0, 2.0, 0.5]]
        self.robot.get_desired_joints_position.side_effect = None
        self.robot.get_desired_joints_position.return_value = original
        self.send(twist(0.1, -0.2, 0.3))
        names, rows = self.receiver.command(10.0, 0.004)
        self.assertEqual(names, ['astribot_chassis'])
        for actual, expected in zip(rows[0], [1.0004, 1.9992, 0.5012]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(original, [[1.0, 2.0, 0.5]])

    def test_planar_magnitude_and_yaw_are_limited(self):
        self.send(twist(3.0, 4.0, -2.0))
        _, rows = self.receiver.command(10.0, 0.004)
        self.assertAlmostEqual(rows[0][0], 1.0 + 0.18 * 0.004)
        self.assertAlmostEqual(rows[0][1], 2.0 + 0.24 * 0.004)
        self.assertAlmostEqual(rows[0][2], 0.5 - 0.6 * 0.004)

    def test_timeout_holds_then_fresh_command_resumes(self):
        self.send(twist(0.1))
        held = self.receiver.command(10.0, 0.004)
        self.assertEqual(self.receiver.command(10.5, 0.004), held)
        self.assertEqual(self.receiver.command(11.0, 0.004), held)
        self.send(twist(-0.1), 12.0)
        _, rows = self.receiver.command(12.0, 0.004)
        self.assertAlmostEqual(rows[0][0], 1.0)

    def test_expired_first_command_does_not_start_motion(self):
        self.send(twist(0.1))
        self.assertEqual(self.receiver.command(10.5, 0.004), ([], []))

    def test_invalid_command_cancels_previous_velocity(self):
        for component, axis, value in [('linear', 'x', math.nan), ('angular', 'z', math.inf),
                                        ('linear', 'z', 0.1), ('angular', 'x', 0.1),
                                        ('angular', 'y', math.nan)]:
            with self.subTest(component=component, axis=axis, value=value):
                self.send(twist(0.1))
                held = self.receiver.command(10.0, 0.004)
                invalid = twist(0.1)
                setattr(getattr(invalid, component), axis, value)
                self.send(invalid, 10.1)
                self.assertEqual(self.receiver.command(10.1, 0.004), held)


class JointTests(unittest.TestCase):
    def setUp(self):
        self.robot = robot_fixture()
        self.receiver = control_joints.JointReceiver(self.robot)

    def test_idle_and_partial_named_commands_preserve_other_joints(self):
        self.assertEqual(self.receiver.command(0.0, 0.004), ([], []))
        self.receiver.receive(joint_command(['astribot_head_joint_2'], [-0.2]))
        self.assertEqual(self.receiver.command(0.0, 0.004), (['astribot_head'], [[0.1, -0.2]]))
        self.receiver.receive(joint_command(['astribot_head_joint_1'], [0.3]))
        expected = (['astribot_head'], [[0.3, -0.2]])
        self.assertEqual(self.receiver.command(0.0, 0.004), expected)
        self.assertEqual(self.receiver.command(100.0, 0.004), expected)
        self.robot.get_desired_joints_position.assert_called_once_with(names=['astribot_head'])

    def test_out_of_order_names_are_grouped_in_sdk_order(self):
        self.receiver.receive(joint_command(
            ['astribot_head_joint_2', 'astribot_arm_right_joint_7', 'astribot_torso_joint_3'],
            [-0.2, 0.3, 0.4]))
        names, rows = self.receiver.command(0.0, 0.004)
        self.assertEqual(names, ['astribot_torso', 'astribot_arm_right', 'astribot_head'])
        self.assertEqual(rows, [[0.1, 0.1, 0.4, 0.1], [0.1] * 6 + [0.3], [0.1, -0.2]])

    def test_invalid_messages_are_rejected_atomically(self):
        head = 'astribot_head_joint_1'
        invalid = [joint_command([], []), joint_command([head], []),
                   joint_command([head, head], [0.1, 0.2]),
                   joint_command([head, 'unknown'], [0.2, 0.1]),
                   joint_command(['astribot_chassis_x'], [0.1]),
                   joint_command([head], [math.nan]), joint_command([head], [math.inf]),
                   joint_command([head], [1.01]), joint_command([head], [-1.01]),
                   joint_command([head], [0.1], velocity=[0.0]),
                   joint_command([head], [0.1], effort=[0.0])]
        for message in invalid:
            with self.subTest(message=message):
                self.receiver.receive(message)
                self.assertEqual(self.receiver.command(0.0, 0.004), ([], []))
        self.robot.get_desired_joints_position.assert_not_called()

    def test_valid_pending_update_survives_later_invalid_message(self):
        self.receiver.receive(joint_command(['astribot_head_joint_1'], [0.3]))
        self.receiver.receive(joint_command(['astribot_head_joint_1', 'bad'], [0.8, 0.1]))
        self.assertEqual(self.receiver.command(0.0, 0.004), (['astribot_head'], [[0.3, 0.1]]))

    def test_bad_sdk_limits_or_positions_fail_before_dispatch(self):
        self.robot.get_joints_position_limit.return_value = ([], [])
        with self.assertRaisesRegex(ValueError, 'limits'):
            control_joints.JointReceiver(self.robot)
        self.receiver.receive(joint_command(['astribot_head_joint_1'], [0.3]))
        self.robot.get_desired_joints_position.side_effect = lambda names: [[math.nan, 0.1]]
        with self.assertRaisesRegex(ValueError, 'finite'):
            self.receiver.command(0.0, 0.004)


class RuntimeTests(unittest.TestCase):
    def run_loop(self, robot, factories, clock=None, ok=None):
        ros, sdk = Mock(), Mock()
        ros.ok.side_effect = [True, False, True] if ok is None else ok
        sdk.Astribot.return_value = robot
        with patch.dict(sys.modules, {
            'rclpy': ros, 'astribot_sdk.core.astribot_api.astribot_client': sdk,
        }), patch.object(control.time, 'sleep'), patch.object(control.signal, 'signal'), \
                patch.object(control.time, 'monotonic', side_effect=clock,
                             return_value=10.0), contextlib.redirect_stderr(io.StringIO()):
            result = control.run_receivers(250.0, factories)
        return result, ros, sdk

    def test_unavailable_stdin_declines_sdk_takeover_instead_of_raising(self):
        sdk = Mock()
        sdk.Astribot.side_effect = lambda **kwargs: input()
        for error in (EOFError, OSError):
            with patch.dict(sys.modules, {'astribot_sdk.core.astribot_api.astribot_client': sdk}), \
                    patch('builtins.input', side_effect=error) as original_input:
                self.assertEqual(control.connect_robot(250.0), '')
                self.assertIs(input, original_input)

    def test_subscription_types_topics_qos_callbacks_and_cleanup(self):
        robot = robot_fixture()
        twist_type, joint_type = type('Twist', (), {}), type('JointState', (), {})
        qos_module = SimpleNamespace(
            QoSProfile=lambda **kwargs: SimpleNamespace(**kwargs),
            ReliabilityPolicy=SimpleNamespace(BEST_EFFORT='best_effort'),
            DurabilityPolicy=SimpleNamespace(VOLATILE='volatile'))
        modules = {'geometry_msgs.msg': SimpleNamespace(Twist=twist_type),
                   'sensor_msgs.msg': SimpleNamespace(JointState=joint_type),
                   'rclpy.qos': qos_module}
        receivers = [control_mobile_base.MobileBaseReceiver(robot, topic='/base/input'),
                     control_joints.JointReceiver(robot, topic='/body/input')]
        node = robot.astribot_interface.node
        node.create_subscription.side_effect = ['base_sub', 'joint_sub']
        with patch.dict(sys.modules, modules):
            for receiver in receivers:
                receiver.start()
        calls = node.create_subscription.call_args_list
        for call, kind, topic, receiver in zip(
                calls, [twist_type, joint_type], ['/base/input', '/body/input'], receivers):
            self.assertEqual(call.args[:3], (kind, topic, receiver.receive))
            self.assertEqual(vars(call.args[3]), {
                'depth': 1, 'reliability': 'best_effort', 'durability': 'volatile'})
        for receiver in receivers:
            receiver.close()
            receiver.close()
        self.assertEqual([call.args[0] for call in node.destroy_subscription.call_args_list],
                         ['base_sub', 'joint_sub'])

    def test_combined_receivers_use_one_connection_and_one_scoped_setter(self):
        robot = robot_fixture()
        base = control_mobile_base.MobileBaseReceiver(robot)
        joints = control_joints.JointReceiver(robot)
        with patch.object(control_mobile_base.time, 'monotonic', return_value=10.0):
            base.receive(twist(0.1))
        joints.receive(joint_command(['astribot_head_joint_2'], [0.2]))
        base.start = Mock()
        joints.start = Mock()
        result, ros, sdk = self.run_loop(robot, [lambda r: base, lambda r: joints],
                                         clock=[10.0, 10.004, 10.004])
        self.assertEqual(result, 0)
        sdk.Astribot.assert_called_once_with(freq=250.0, node_name='astribot_command_receiver')
        robot.set_joints_position.assert_called_once()
        args, kwargs = robot.set_joints_position.call_args
        self.assertEqual(args[0], ['astribot_chassis', 'astribot_head'])
        self.assertAlmostEqual(args[1][0][0], 1.0004)
        self.assertEqual(args[1][1], [0.1, 0.2])
        self.assertEqual(kwargs, {'control_way': 'filter', 'use_wbc': False, 'add_default_torso': False})
        robot.move_to_home.assert_not_called()
        robot.stop_robot.assert_called_once()
        ros.shutdown.assert_called_once()

    def test_loop_stall_never_becomes_a_large_integration_step(self):
        robot = robot_fixture()
        receiver = Mock()
        receiver.command.return_value = ([], [])
        result, _, _ = self.run_loop(robot, [lambda r: receiver], clock=[1.0, 100.0, 100.0])
        self.assertEqual(result, 0)
        receiver.command.assert_called_once_with(100.0, 0.004)
        robot.set_joints_position.assert_not_called()
        robot.stop_robot.assert_not_called()

    def test_control_denial_and_feedback_loss_prevent_dispatch(self):
        for control_rights, alive in [(False, True), (True, False)]:
            robot = robot_fixture()
            robot.get_control_rights_status.return_value = control_rights
            robot.astribot_interface.is_alive.return_value = alive
            receiver = Mock()
            result, _, _ = self.run_loop(robot, [lambda r: receiver], ok=[True] * 3)
            self.assertEqual(result, 1)
            robot.set_joints_position.assert_not_called()
            receiver.command.assert_not_called()

    def test_dispatch_failure_stops_robot_and_cleans_up_receiver(self):
        robot = robot_fixture()
        robot.set_joints_position.side_effect = RuntimeError('send failed')
        receiver = Mock()
        receiver.command.return_value = (['astribot_head'], [[0.0, 0.0]])
        result, ros, _ = self.run_loop(robot, [lambda r: receiver], ok=[True, True])
        self.assertEqual(result, 1)
        robot.stop_robot.assert_called_once()
        receiver.close.assert_called_once()
        ros.shutdown.assert_called_once()

    def test_control_transfer_does_not_stop_the_new_owner(self):
        robot = robot_fixture()
        robot.get_control_rights_status.side_effect = [True, True, False, False]
        receiver = Mock()
        receiver.command.return_value = (['astribot_head'], [[0.0, 0.0]])
        result, ros, _ = self.run_loop(robot, [lambda r: receiver], ok=[True, True, True])
        self.assertEqual(result, 1)
        robot.set_joints_position.assert_called_once()
        robot.stop_robot.assert_not_called()
        receiver.close.assert_called_once()
        ros.shutdown.assert_called_once()

    def test_interrupt_stops_motion_before_ros_shutdown(self):
        robot = robot_fixture()
        robot.set_joints_position.side_effect = KeyboardInterrupt
        receiver = Mock()
        receiver.command.return_value = (['astribot_head'], [[0.0, 0.0]])
        result, ros, _ = self.run_loop(robot, [lambda r: receiver], ok=[True, True])
        self.assertEqual(result, 130)
        robot.stop_robot.assert_called_once()
        receiver.close.assert_called_once()
        ros.shutdown.assert_called_once()

    def test_receiver_close_failure_still_shuts_down_ros(self):
        robot = robot_fixture()
        receiver = Mock()
        receiver.command.return_value = ([], [])
        receiver.close.side_effect = RuntimeError('context already closed')
        result, ros, _ = self.run_loop(robot, [lambda r: receiver])
        self.assertEqual(result, 1)
        ros.shutdown.assert_called_once()

    def test_help_and_invalid_options_require_no_ros_or_sdk(self):
        for module in (control_mobile_base, control_joints):
            result = subprocess.run([sys.executable, '-m', module.__name__, '--help'],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('--topic', result.stdout)
            for value in ('0', '-1', 'nan', 'inf'):
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
                    module.main(['--rate', value])
                self.assertEqual(exc.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
