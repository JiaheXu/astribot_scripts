"""Shared SDK loop for ROS command receivers; importing this does not connect."""
import math
import signal
import sys
import time


def finite_vector(values, size, label):
    if values is None or len(values) != size:
        raise ValueError(f'{label} must contain {size} values')
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f'{label} must contain only finite values')
    return result


def desired_positions(robot, names, dofs):
    rows = robot.get_desired_joints_position(names=names)
    if rows is None or len(rows) != len(names):
        raise ValueError('Incomplete desired joint positions from SDK')
    return [finite_vector(row, dof, name) for name, dof, row in zip(names, dofs, rows)]


def command_qos():
    # Accept both reliable and best-effort publishers, without a stale backlog
    # or commands retained from before this receiver started.
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def connect_robot(rate):
    from astribot_sdk.core.astribot_api.astribot_client import Astribot
    from unittest.mock import patch

    read_input = input

    def control_answer(*args, **kwargs):
        try:
            return read_input(*args, **kwargs)
        except (EOFError, OSError):
            # The SDK's broad exception handler otherwise treats input failure
            # as an absent control owner and can grant itself control.
            return ''

    with patch('builtins.input', control_answer):
        return Astribot(freq=rate, node_name='astribot_command_receiver')


def run_receivers(rate, factories):
    """Run all receivers with one SDK owner and one filtered command per tick."""
    import rclpy

    robot, receivers = None, []
    previous_handlers = {}
    motion_requested = False
    result = 0

    def interrupt(signum, frame):
        # Keep ROS usable until the finally block sends the SDK stop command.
        raise KeyboardInterrupt

    try:
        robot = connect_robot(rate)
        if robot.robot_type != 'S1':
            raise ValueError('These command receivers require an Astribot S1')
        if not robot.get_control_rights_status():
            raise RuntimeError('SDK control rights are required; no commands were sent')
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, interrupt)
        for factory in factories:
            receiver = factory(robot)
            receivers.append(receiver)
            receiver.start()
        period = 1.0 / rate
        previous = time.monotonic()
        # The SDK already spins this node in its background executor. Keep SDK
        # setters on this thread, outside subscription callbacks.
        while rclpy.ok():
            started = time.monotonic()
            dt = min(max(started - previous, 0.0), period)
            previous = started
            if not robot.get_control_rights_status():
                raise RuntimeError('Lost SDK control rights')
            if not robot.astribot_interface.is_alive():
                raise RuntimeError('SDK connection lost')
            names, positions = [], []
            for receiver in receivers:
                parts, rows = receiver.command(started, dt)
                names.extend(parts)
                positions.extend(rows)
            if names:
                motion_requested = True
                robot.set_joints_position(names, positions, control_way='filter',
                                          use_wbc=False, add_default_torso=False)
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        result = 130
    except Exception as exc:
        print(f'[CONTROL] ERROR: {exc}', file=sys.stderr, flush=True)
        result = 1
    finally:
        if robot is not None and motion_requested:
            try:
                if robot.get_control_rights_status():
                    robot.stop_robot()
            except Exception as exc:
                print(f'[CONTROL] Could not confirm SDK stop: {exc}', file=sys.stderr, flush=True)
                result = 1
        for receiver in receivers:
            try:
                receiver.close()
            except Exception as exc:
                print(f'[CONTROL] Could not close receiver: {exc}', file=sys.stderr, flush=True)
                result = 1
        try:
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)
    return result
