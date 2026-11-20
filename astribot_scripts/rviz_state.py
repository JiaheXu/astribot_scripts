#!/usr/bin/env python3
"""Publish S1 current joints and current/desired Cartesian axes for RViz2."""
import argparse
import time
from unittest.mock import patch

import rclpy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray

from astribot_scripts.common.cli import positive_rate
from astribot_scripts.common.state import (
    FRAME, MODEL_JOINTS, POSE_PARTS, joint_message, pose_markers, robot_description,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rate', type=positive_rate, default=10.0, help='polling Hz (default: 10)')
    args = parser.parse_args()
    # Keep model loading and offline checks independent of SDK initialization.
    from astribot_sdk.core.astribot_api.astribot_client import Astribot

    publishers = []
    node = None
    try:
        with patch('builtins.input', return_value=''):
            robot = Astribot()
        if robot.robot_type != 'S1':
            raise ValueError('This visualization uses the bundled S1 model')
        node = robot.astribot_interface.node
        for message_type, name in [(JointState, 'joint_states'),
                                   (MarkerArray, 'current_poses'), (MarkerArray, 'desired_poses')]:
            publishers.append(node.create_publisher(message_type, f'/astribot_viz/{name}', 10))
        node.get_logger().info('Publishing S1 joints and chassis-frame poses for RViz. '
                               'Timestamps are polling time; SDK getters are not synchronized.')
        while rclpy.ok():
            started = time.monotonic()
            if not robot.astribot_interface.is_alive():
                raise RuntimeError('SDK connection lost')
            try:
                stamp = node.get_clock().now().to_msg()
                messages = [joint_message(robot.get_current_joints_position(names=list(MODEL_JOINTS)), stamp)]
                for state in ('current', 'desired'):
                    poses = getattr(robot, f'get_{state}_cartesian_pose')(names=POSE_PARTS, frame='chassis')
                    messages.append(pose_markers(poses, stamp, state, max(1.0, 3.0 / args.rate)))
            except (ValueError, TypeError) as exc:
                node.get_logger().warning(f'Skipping invalid state: {exc}', throttle_duration_sec=5.0)
            else:
                for publisher, message in zip(publishers, messages):
                    publisher.publish(message)
            # The SDK already spins this node on its background executor.
            time.sleep(max(0.0, 1.0 / args.rate - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        for publisher in publishers:
            node.destroy_publisher(publisher)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
