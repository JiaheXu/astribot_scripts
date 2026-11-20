#!/usr/bin/env python3
"""Move an already powered/activated robot home and verify completion within 10s.

The SDK retains its control-rights prompt and home-path collision checks.
Its home trajectory is capped at about 5s. The deadline below is checked after
the blocking SDK call returns; it is not an independent hardware watchdog.
"""

import math
import time
import importlib
import json
from pathlib import Path
from unittest.mock import patch

import rclpy
from astribot_sdk.core.astribot_api.astribot_client import Astribot


COMPLETION_TIMEOUT = 10.0
# Match the SDK home pose for feedback verification, not for issuing a new path.
HOME = {
    'astribot_torso': [0.29, -0.58, 0.29, 0.0],
    'astribot_arm_left': [0.159, -0.022, -1.42, 1.66, -0.345, 0.115, 0.125],
    'astribot_gripper_left': [0.0],
    'astribot_arm_right': [-0.159, -0.022, 1.42, 1.66, 0.345, 0.115, -0.125],
    'astribot_gripper_right': [0.0],
    'astribot_head': [0.0, 0.0],
}


def at_home(positions):
    if positions is None or len(positions) != len(HOME):
        return False
    for (name, target), measured in zip(HOME.items(), positions):
        tolerance = 2.0 if 'gripper' in name else 0.02
        if len(measured) != len(target):
            return False
        if not all(math.isfinite(value) and abs(value - goal) <= tolerance
                   for value, goal in zip(measured, target)):
            return False
    return True


def connect_home_robot(non_interactive=False):
    if not non_interactive:
        return Astribot()
    # The SDK catches input/connection exceptions and may claim control anyway.
    # Do not feed EOF or automatically answer "yes" to that takeover path.
    sdk = importlib.import_module('astribot_sdk.core.astribot_api.astribot_client')
    from astribot_msgs.srv import RawRequest
    from rclpy.callback_groups import ReentrantCallbackGroup

    class UnownedControlOnly(sdk.AstribotInterface):
        def acquire_control_rights(self, high_control_rights=False):
            self.have_control_rights = False
            client = self.node.create_client(RawRequest, '/astribot/control_rights')
            try:
                if client.wait_for_service(timeout_sec=2.0):
                    self.node.get_logger().error('Another SDK controller owns the robot. '
                                                 'Automation will not seize its control rights.')
                    return
            finally:
                self.node.destroy_client(client)
            self._control_rights_srv = self.node.create_service(
                RawRequest, '/astribot/control_rights', self.transfer_control_rights,
                callback_group=ReentrantCallbackGroup())
            self.have_control_rights = True

    with patch.object(sdk, 'AstribotInterface', UnownedControlOnly):
        return Astribot()


def main(non_interactive=False, completion_file=None):
    robot = None
    motion_requested = False
    try:
        print('[STARTUP] Connecting; robot must already be powered on and activated.', flush=True)
        robot = connect_home_robot(non_interactive)
        if not robot.get_control_rights_status():
            raise RuntimeError('Control rights are required to move home; no motion was requested.')
        if not robot.astribot_interface.is_alive():
            raise RuntimeError('Robot interface is not alive.')
        print('[STARTUP] Moving home (opens grippers); completion window is 10 seconds. '
              'SDK home-motion duration is capped at about 5 seconds.', flush=True)
        deadline = time.monotonic() + COMPLETION_TIMEOUT
        motion_requested = True
        result = robot.move_to_home(duration=COMPLETION_TIMEOUT)
        print(f'[STARTUP] SDK home result: {result}', flush=True)
        if result is None or result is False or any(word in str(result).lower() for word in ('fail', 'error', 'reject')):
            raise RuntimeError(f'SDK did not accept/complete home motion: {result}')
        while rclpy.ok():
            if time.monotonic() >= deadline:
                raise TimeoutError('Home position was not confirmed within 10 seconds.')
            if not robot.get_control_rights_status() or not robot.astribot_interface.is_alive():
                raise RuntimeError('Lost control rights or robot feedback while moving home.')
            measured = robot.get_current_joints_position(names=list(HOME))
            if at_home(measured):
                if completion_file is not None:
                    Path(completion_file).write_text(json.dumps({
                        'stage': 'home', 'success': True, 'completed_at': time.time(),
                        'measured_joints': dict(zip(HOME, measured)),
                    }, allow_nan=False))
                print('[STARTUP] Start up process is done. Home position confirmed.', flush=True)
                return 0
            time.sleep(0.1)
        raise RuntimeError('ROS shut down before home position was confirmed.')
    except (Exception, KeyboardInterrupt) as exc:
        print(f'[STARTUP] ERROR: {exc or "Interrupted"}', flush=True)
        if motion_requested:
            try:
                if robot.get_control_rights_status():
                    robot.stop_robot()
            except Exception as stop_error:
                print(f'[STARTUP] Could not confirm stop: {stop_error}', flush=True)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--non-interactive', action='store_true',
                        help='claim only unowned SDK control; never prompt or take over another controller')
    parser.add_argument('--completion-file', help=argparse.SUPPRESS)
    args = parser.parse_args()
    raise SystemExit(main(args.non_interactive, args.completion_file))
