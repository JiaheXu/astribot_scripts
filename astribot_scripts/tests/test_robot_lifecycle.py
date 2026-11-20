"""Sequence and protocol checks with simulated diagnostics; never contact a robot."""
import contextlib
import io
import json
import time
import unittest
from unittest.mock import Mock, patch

from astribot_scripts.common.robot_lifecycle import (
    Status, Sequencer, LifecycleError, RobotWeb, diagnostic_type, decode_status,
)
from astribot_scripts import robot_power


class SimulatedRobot:
    def __init__(self, power=False, driver=False):
        self.power, self.driver = power, driver
        self.now = 0.0
        self.events = []
        self.fault = None
        self.ignore = None
        self.freeze = False
        self.disconnected = False
        self.source = 'WEB'
        self.owners = ['/web_astribot_42'] if driver else []
        self.keep_owner = False

    def clock(self):
        return self.now

    def read(self, timeout=3):
        self.now += .25
        if self.disconnected:
            raise LifecycleError('No fresh DSM diagnostics')
        codes = [14000000]
        if self.power:
            codes.append(12000000)
        if self.driver:
            codes.append(13000000)
        if self.fault:
            codes.append(self.fault)
        return Status(tuple(codes), 1 if self.freeze else int(self.now*1e9), self.now)

    def command(self, module, enabled):
        self.events.append((module, enabled, self.now))
        if (module, enabled) != self.ignore:
            setattr(self, module, enabled)
            if module == 'driver':
                self.owners = ['/web_astribot_42'] if enabled else []

    def control_source(self):
        return self.source

    def control_owners(self):
        return list(self.owners)

    def select_pc_control(self):
        self.events.append(('source', 'PC', self.now))
        if self.ignore != 'handoff':
            self.source = 'PC'
            if not self.keep_owner:
                self.owners = []

    def home(self):
        self.events.append(('home', True, self.now))
        return 0

    def sequencer(self, **kwargs):
        return Sequencer(self, self, self.home, timeout=2, stable_seconds=.5, cooldown=1,
                         clock=self.clock, report=lambda _: None, **kwargs)


class RobotLifecycleTests(unittest.TestCase):
    def test_start_confirms_power_then_activation_then_home(self):
        robot = SimulatedRobot()
        robot.sequencer().start()
        self.assertEqual([(key, enabled) for key, enabled, _ in robot.events],
                         [('power', True), ('driver', True), ('source', 'PC'), ('home', True)])
        self.assertGreaterEqual(robot.events[0][2], 1)
        self.assertGreaterEqual(robot.events[1][2]-robot.events[0][2], .5)
        self.assertGreaterEqual(robot.events[2][2]-robot.events[1][2], 5.5)
        self.assertGreaterEqual(robot.events[3][2]-robot.events[2][2], .5)

    def test_already_active_skips_power_and_activation_commands(self):
        robot = SimulatedRobot(True, True)
        robot.sequencer().start()
        self.assertEqual([event[0] for event in robot.events], ['source', 'home'])

    def test_existing_pc_source_without_owner_needs_no_handoff_command(self):
        robot = SimulatedRobot(True, True)
        robot.source, robot.owners = 'PC', []
        robot.sequencer().start()
        self.assertEqual([event[0] for event in robot.events], ['home'])

    def test_other_controllers_and_teleoperation_are_not_taken_over(self):
        for source, owners in [('PC', ['/use_astribot_99']),
                               ('WEB', ['/web_astribot_42', '/another_sdk']),
                               ('VR_CONTROL', ['/web_astribot_42']),
                               ('DARG', []), ('JOYSTICK', [])]:
            with self.subTest(source=source, owners=owners):
                robot = SimulatedRobot(True, True)
                robot.source, robot.owners = source, owners
                with self.assertRaises(LifecycleError):
                    robot.sequencer().start()
                self.assertEqual(robot.events, [])

    def test_handoff_ack_without_pc_source_or_release_never_homes(self):
        for field, value in [('ignore', 'handoff'), ('keep_owner', True)]:
            with self.subTest(field=field):
                robot = SimulatedRobot(True, True)
                setattr(robot, field, value)
                with self.assertRaisesRegex(LifecycleError, 'Timed out'):
                    robot.sequencer().start()
                self.assertEqual([event[:2] for event in robot.events], [('source', 'PC')])

    def test_lost_activation_during_settling_blocks_handoff_and_home(self):
        robot = SimulatedRobot(True, True)
        original_read = robot.read
        def read(timeout=3):
            if robot.now >= 1:
                robot.driver = False
            return original_read(timeout)
        robot.read = read
        with self.assertRaisesRegex(LifecycleError, 'activation was lost'):
            robot.sequencer().start()
        self.assertEqual(robot.events, [])

    def test_control_state_read_failure_blocks_power_and_home(self):
        robot = SimulatedRobot()
        robot.control_source = Mock(side_effect=LifecycleError('control-state unavailable'))
        with self.assertRaisesRegex(LifecycleError, 'control-state unavailable'):
            robot.sequencer().start()
        self.assertEqual(robot.events, [])

    def test_shutdown_waits_for_deactivation_before_power_off(self):
        robot = SimulatedRobot(True, True)
        robot.sequencer().stop()
        self.assertEqual([(key, enabled) for key, enabled, _ in robot.events],
                         [('driver', False), ('power', False)])
        self.assertGreaterEqual(robot.events[1][2]-robot.events[0][2], .5)

    def test_already_off_does_not_send_commands(self):
        robot = SimulatedRobot()
        robot.sequencer().stop()
        self.assertEqual(robot.events, [])

    def test_http_ack_without_power_feedback_never_activates_or_moves(self):
        robot = SimulatedRobot()
        robot.ignore = ('power', True)
        with self.assertRaisesRegex(LifecycleError, 'Timed out'):
            robot.sequencer().start()
        self.assertEqual([event[:2] for event in robot.events], [('power', True)])

    def test_failed_deactivation_never_powers_off(self):
        robot = SimulatedRobot(True, True)
        robot.ignore = ('driver', False)
        with self.assertRaisesRegex(LifecycleError, 'Timed out'):
            robot.sequencer().stop()
        self.assertEqual([event[:2] for event in robot.events], [('driver', False)])

    def test_lost_or_duplicate_diagnostics_never_count_as_off(self):
        for field in ('disconnected', 'freeze'):
            robot = SimulatedRobot(True, True)
            setattr(robot, field, True)
            with self.subTest(field=field), self.assertRaises(LifecycleError):
                robot.sequencer().stop()
            self.assertEqual(robot.events, [])

    def test_estop_or_control_fault_blocks_start(self):
        for code in (12200100, 13200001, 16200000, 18200002):
            robot = SimulatedRobot(True, True)
            robot.fault = code
            with self.subTest(code=code), self.assertRaises(LifecycleError):
                robot.sequencer().start()
            self.assertEqual(robot.events, [])

    def test_home_failure_does_not_force_deactivation_or_power_off(self):
        robot = SimulatedRobot(True, True)
        robot.home = Mock(return_value=1)
        with self.assertRaisesRegex(LifecycleError, 'Home was not confirmed'):
            robot.sequencer().start()
        self.assertEqual([event[:2] for event in robot.events], [('source', 'PC')])

    def test_rejected_handoff_never_runs_home_or_retries(self):
        robot = SimulatedRobot(True, True)
        robot.select_pc_control = Mock(side_effect=LifecycleError('handoff rejected'))
        with self.assertRaisesRegex(LifecycleError, 'handoff rejected'):
            robot.sequencer().start()
        robot.select_pc_control.assert_called_once()
        self.assertEqual(robot.events, [])

    def test_controller_appearing_during_settling_blocks_handoff(self):
        robot = SimulatedRobot(True, True)
        robot.control_owners = Mock(side_effect=[['/web_astribot_42'], ['/another_sdk']])
        with self.assertRaisesRegex(LifecycleError, 'another_sdk'):
            robot.sequencer().start()
        self.assertEqual(robot.events, [])

    def test_verified_descriptor_decodes_source_time_and_codes(self):
        message = diagnostic_type()()
        now = time.time()
        message.header.time_pub = int(now * 1e9)
        message.header.seq = 12
        message.itcs.extend([12000000, 13000000, 14000000])
        state = decode_status(message.SerializeToString(), 2.0, now)
        self.assertEqual((state.state('power'), state.state('driver'), state.seq), ('on', 'on', 12))
        with self.assertRaisesRegex(LifecycleError, 'stale'):
            decode_status(message.SerializeToString(), 2.0, now+10)
        with self.assertRaisesRegex(LifecycleError, 'clocks disagree'):
            decode_status(message.SerializeToString(), 2.0, now-10)
        message.itcs.append(12345678)
        with self.assertRaisesRegex(LifecycleError, 'Unrecognized'):
            decode_status(message.SerializeToString(), 2.0, now)

    def test_exact_command_envelopes_and_no_retry_after_rejection(self):
        web = RobotWeb('127.0.0.1', 5141)
        response = Mock()
        response.__enter__ = Mock(return_value=response)
        response.__exit__ = Mock(return_value=False)
        response.read.return_value = b'{"suc":true,"data":{}}'
        web.opener = Mock()
        web.opener.open.return_value = response
        for module, enabled, msg_id in [('power', True, 'robot_power'), ('driver', False, 'robot_driver')]:
            web.command(module, enabled)
            request = web.opener.open.call_args.args[0]
            self.assertEqual(request.full_url, 'http://127.0.0.1:5141/msg')
            self.assertEqual(json.loads(request.data), {'msg_id': msg_id, 'data': {'start': enabled}})
        response.read.return_value = b'{"suc":false,"reason":"busy"}'
        web.opener.open.reset_mock()
        with self.assertRaises(LifecycleError):
            web.command('power', False)
        web.opener.open.assert_called_once()
        with self.assertRaises(ValueError):
            web.command('brake', True)

    def test_exact_pc_handoff_envelope_and_rejection_has_no_retry(self):
        web = RobotWeb('127.0.0.1', 5141)
        response = Mock()
        response.__enter__ = Mock(return_value=response)
        response.__exit__ = Mock(return_value=False)
        response.read.return_value = b'{"suc":true}'
        web.opener = Mock()
        web.opener.open.return_value = response
        web.select_pc_control()
        request = web.opener.open.call_args.args[0]
        self.assertEqual(request.full_url, 'http://127.0.0.1:5141/msg')
        self.assertEqual(json.loads(request.data), {'msg_id': 'set_control_source', 'data': {'source': 'PC'}})
        response.read.return_value = b'{"suc":false}'
        web.opener.open.reset_mock()
        with self.assertRaisesRegex(LifecycleError, 'handoff command rejected'):
            web.select_pc_control()
        web.opener.open.assert_called_once()

    def test_passive_control_source_parser_rejects_bad_data(self):
        for source in ('WEB', 'PC', 'VR_CONTROL'):
            self.assertEqual(RobotWeb.parse_control_source(json.dumps([
                {'msg_id': 'control_source', 'data': {'source': source}}])), source)
        self.assertIsNone(RobotWeb.parse_control_source('[]'))
        for payload in ('{}', '[{"msg_id":"control_source","data":{}}]',
                        '[{"msg_id":"control_source","data":{"source":"unknown"}}]'):
            with self.subTest(payload=payload), self.assertRaises(LifecycleError):
                RobotWeb.parse_control_source(payload)

    def test_cli_preview_never_sends_commands_or_launches_sdk(self):
        for action in ('start', 'stop', 'status'):
            with self.subTest(action=action), patch.object(robot_power, 'DiagnosticMonitor') as monitor, \
                    patch.object(robot_power, 'RobotWeb') as web, \
                    patch.object(robot_power.Sequencer, 'wait', return_value=Status((12000000,13000000),1,1)), \
                    patch.object(robot_power, 'run_home') as home, \
                    contextlib.redirect_stdout(io.StringIO()):
                monitor.return_value.control_owners.return_value = []
                self.assertEqual(robot_power.main([action]), 0)
                web.return_value.command.assert_not_called()
                web.return_value.select_pc_control.assert_not_called()
                home.assert_not_called()
                monitor.return_value.close.assert_called_once()

    def test_sdk_exit_zero_without_home_confirmation_is_a_failure(self):
        with patch.object(robot_power.subprocess, 'Popen') as process:
            process.return_value.wait.return_value = 0
            with self.assertRaisesRegex(LifecycleError, 'without measured-home confirmation'):
                robot_power.run_home(123)
        self.assertEqual(process.call_args.kwargs['pass_fds'], (123,))
        self.assertEqual(process.call_args.kwargs['stdin'], robot_power.subprocess.DEVNULL)

    def test_home_timeout_interrupts_helper_instead_of_forcing_power_off(self):
        with patch.object(robot_power.subprocess, 'Popen') as process:
            process.return_value.wait.side_effect = [robot_power.subprocess.TimeoutExpired('home',45),130]
            process.return_value.poll.return_value = None
            with self.assertRaisesRegex(LifecycleError, 'interrupted or timed out'):
                robot_power.run_home(123)
            process.return_value.send_signal.assert_called_once_with(robot_power.signal.SIGINT)


if __name__ == '__main__':
    unittest.main()
