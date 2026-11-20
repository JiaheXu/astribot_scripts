#!/usr/bin/env python3
"""Receive local-frame ROS Twist commands and control the S1 mobile base.

Uses the position integration from examples/202-chassis_joy_control_local.py.
linear.x/y are m/s; angular.z is rad/s. Run --with-joints to also receive body
joint positions through the same SDK connection.
"""
import argparse
import math
import threading
import time

from astribot_scripts.common.cli import positive_rate
from astribot_scripts.common.control import command_qos, desired_positions, finite_vector, run_receivers


class MobileBaseReceiver:
    def __init__(self, robot, topic='/cmd_vel', timeout=0.5,
                 max_linear_speed=0.3, max_angular_speed=0.6):
        self.robot = robot
        self.node = robot.astribot_interface.node
        self.topic = topic
        self.timeout = timeout
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self._lock = threading.Lock()
        self._latest = None
        self._position = None
        self._timed_out = False
        self.subscription = None

    def start(self):
        from geometry_msgs.msg import Twist
        self.subscription = self.node.create_subscription(
            Twist, self.topic, self.receive, command_qos())
        self.node.get_logger().info(
            f'Receiving base Twist on {self.topic} in the local chassis frame; '
            f'command timeout {self.timeout:g}s')

    def receive(self, message):
        try:
            values = finite_vector([
                message.linear.x, message.linear.y, message.angular.z,
                message.linear.z, message.angular.x, message.angular.y,
            ], 6, 'Twist')
            if any(value != 0.0 for value in values[3:]):
                raise ValueError('Only linear.x, linear.y and angular.z are supported')
            vx, vy, wz = values[:3]
            speed = math.hypot(vx, vy)
            if speed > self.max_linear_speed:
                vx, vy = vx / speed * self.max_linear_speed, vy / speed * self.max_linear_speed
            wz = max(-self.max_angular_speed, min(wz, self.max_angular_speed))
        except (TypeError, ValueError) as exc:
            with self._lock:
                self._latest = None
            self.node.get_logger().warning(f'Rejected base command: {exc}', throttle_duration_sec=1.0)
            return
        with self._lock:
            self._latest = ([vx, vy, wz], time.monotonic())

    def command(self, now, dt):
        with self._lock:
            latest = self._latest
        fresh = latest is not None and 0.0 <= now - latest[1] < self.timeout
        if self._position is None:
            if not fresh:
                return [], []
            self._position = desired_positions(self.robot, [self.robot.chassis_name], [3])[0]
        if fresh:
            velocity = latest[0]
            self._timed_out = False
        else:
            velocity = [0.0, 0.0, 0.0]
            if not self._timed_out:
                self.node.get_logger().warning('Base command expired or invalid; holding the base target')
                self._timed_out = True
        # Local axes follow the vendor example: do not apply a world-frame
        # rotation. Never integrate a long scheduling pause into a position jump.
        self._position = [position + speed * dt
                          for position, speed in zip(self._position, velocity)]
        return [self.robot.chassis_name], [self._position.copy()]

    def close(self):
        if self.subscription is not None:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/cmd_vel', help='input geometry_msgs/msg/Twist topic')
    parser.add_argument('--rate', type=positive_rate, default=250.0, help='SDK control rate in Hz')
    parser.add_argument('--timeout', type=positive_rate, default=0.5,
                        help='hold the base target after this many seconds without a valid command')
    parser.add_argument('--max-linear-speed', type=positive_rate, default=0.3,
                        help='maximum planar speed magnitude in m/s (default: 0.3)')
    parser.add_argument('--max-angular-speed', type=positive_rate, default=0.6,
                        help='maximum absolute yaw rate in rad/s (default: 0.6)')
    parser.add_argument('--with-joints', action='store_true',
                        help='also receive body joint positions using this SDK connection')
    parser.add_argument('--joint-topic', default='/astribot/joint_cmd',
                        help='JointState position command topic for --with-joints')
    args = parser.parse_args(argv)
    factories = [lambda robot: MobileBaseReceiver(
        robot, args.topic, args.timeout, args.max_linear_speed, args.max_angular_speed)]
    if args.with_joints:
        from astribot_scripts.control_joints import JointReceiver
        factories.append(lambda robot: JointReceiver(robot, args.joint_topic))
    return run_receivers(args.rate, factories)


if __name__ == '__main__':
    raise SystemExit(main())
