#!/usr/bin/env python3
"""Receive named ROS JointState position targets for the S1 torso, arms and head.

Uses filtered set_joints_position commands from examples/104-set_joint_position.py.
Positions are radians. Targets persist until replaced; unnamed joints in a newly
commanded body part retain their SDK desired position. No home motion is issued.
"""
import argparse
import threading

from astribot_scripts.common.cli import positive_rate
from astribot_scripts.common.control import command_qos, desired_positions, finite_vector, run_receivers


BODY_DOFS = {'astribot_torso': 4, 'astribot_arm_left': 7,
             'astribot_arm_right': 7, 'astribot_head': 2}


class JointReceiver:
    def __init__(self, robot, topic='/astribot/joint_cmd'):
        self.robot = robot
        self.node = robot.astribot_interface.node
        self.topic = topic
        self._lock = threading.Lock()
        self._pending = {}
        self._targets = {}
        self.subscription = None
        names = list(BODY_DOFS)
        lower, upper = robot.get_joints_position_limit(names=names)
        if len(lower) != len(names) or len(upper) != len(names):
            raise ValueError('Incomplete joint position limits from SDK')
        self.joints = {}
        for row, (part, dof) in enumerate(BODY_DOFS.items()):
            lows = finite_vector(lower[row], dof, f'{part} lower limits')
            highs = finite_vector(upper[row], dof, f'{part} upper limits')
            for index, (low, high) in enumerate(zip(lows, highs)):
                if low > high:
                    raise ValueError(f'Inverted joint position limits for {part}')
                self.joints[f'{part}_joint_{index + 1}'] = (part, index, low, high)

    def start(self):
        from sensor_msgs.msg import JointState
        self.subscription = self.node.create_subscription(
            JointState, self.topic, self.receive, command_qos())
        self.node.get_logger().info(
            f'Receiving JointState position targets (rad) on {self.topic}; '
            'targets remain active until replaced')

    def receive(self, message):
        try:
            names = list(message.name)
            if not names or len(names) != len(set(names)):
                raise ValueError('Joint names must be nonempty and unique')
            if len(message.velocity) or len(message.effort):
                raise ValueError('This receiver accepts position only; leave velocity and effort empty')
            positions = finite_vector(message.position, len(names), 'JointState.position')
            updates = {}
            for name, position in zip(names, positions):
                if name not in self.joints:
                    raise ValueError(f'Unknown body joint: {name}')
                _, _, low, high = self.joints[name]
                if not low <= position <= high:
                    raise ValueError(f'{name} position {position:g} is outside [{low:g}, {high:g}] rad')
                updates[name] = position
        except (TypeError, ValueError) as exc:
            self.node.get_logger().warning(f'Rejected joint command: {exc}', throttle_duration_sec=1.0)
            return
        # Validate the entire message before applying any of its joints.
        with self._lock:
            self._pending.update(updates)

    def command(self, now, dt):
        with self._lock:
            pending, self._pending = self._pending, {}
        new_parts = [part for part in BODY_DOFS if part not in self._targets
                     and any(self.joints[name][0] == part for name in pending)]
        if new_parts:
            rows = desired_positions(self.robot, new_parts, [BODY_DOFS[part] for part in new_parts])
            self._targets.update(zip(new_parts, rows))
        for name, position in pending.items():
            part, index, _, _ = self.joints[name]
            self._targets[part][index] = position
        names = [part for part in BODY_DOFS if part in self._targets]
        return names, [self._targets[part].copy() for part in names]

    def close(self):
        if self.subscription is not None:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/astribot/joint_cmd',
                        help='input sensor_msgs/msg/JointState topic')
    parser.add_argument('--rate', type=positive_rate, default=250.0, help='SDK control rate in Hz')
    args = parser.parse_args(argv)
    return run_receivers(args.rate, [lambda robot: JointReceiver(robot, args.topic)])


if __name__ == '__main__':
    raise SystemExit(main())
