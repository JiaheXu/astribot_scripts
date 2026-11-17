#!/usr/bin/env python3
"""Publish S1 current joints and current/desired Cartesian axes for RViz2."""
import argparse
import math
from pathlib import Path
import time
from unittest.mock import patch
import xml.etree.ElementTree as ET

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
import yaml

FRAME = 'astribot_viz_chassis'
MODEL_JOINTS = {
    f'astribot_{part}': [f'astribot_{part}_joint_{i}' for i in range(1, count + 1)]
    for part, count in [('torso', 4), ('arm_left', 7), ('arm_right', 7), ('head', 2)]
}
POSE_PARTS = ['astribot_chassis', *MODEL_JOINTS]


def robot_description():
    config = Path(__file__).resolve().parents[1] / 'astribot_config/robot_config/astribot_s1'
    root = ET.parse(config / 'model/astribot_whole_body_with_head.urdf').getroot()
    for mesh in root.findall('.//mesh'):
        path = config / 'meshes' / mesh.attrib['filename']
        if not path.is_file():
            raise FileNotFoundError(path)
        mesh.set('filename', path.resolve().as_uri())
    mount = yaml.safe_load((config / 'astribot_torso.yaml').read_text())['transform']['weld_to_base_pose']
    if len(mount) != 6 or not all(math.isfinite(v) for v in mount):
        raise ValueError('Invalid torso-to-chassis transform')
    ET.SubElement(root, 'link', name=FRAME)
    joint = ET.SubElement(root, 'joint', name='astribot_viz_base_joint', type='fixed')
    ET.SubElement(joint, 'parent', link=FRAME)
    ET.SubElement(joint, 'child', link='astribot_torso_base')
    ET.SubElement(joint, 'origin', xyz=' '.join(map(str, mount[:3])), rpy=' '.join(map(str, mount[3:])))
    return ET.tostring(root, encoding='unicode')


def joint_message(values, stamp):
    if values is None or len(values) != len(MODEL_JOINTS):
        raise ValueError('Incomplete current joint data')
    message = JointState()
    message.header.stamp = stamp
    for (part, names), row in zip(MODEL_JOINTS.items(), values):
        if row is None or len(row) != len(names) or not all(math.isfinite(v) for v in row):
            raise ValueError(f'Invalid joints for {part}')
        message.name.extend(names)
        message.position.extend(float(v) for v in row)
    return message


def pose_markers(poses, stamp, state, lifetime):
    if poses is None or len(poses) != len(POSE_PARTS):
        raise ValueError(f'Incomplete {state} Cartesian poses')
    result = MarkerArray()
    length, alpha = (0.15, 1.0) if state == 'current' else (0.10, 0.4)
    for index, (part, pose) in enumerate(zip(POSE_PARTS, poses)):
        if pose is None or len(pose) != 7 or not all(math.isfinite(v) for v in pose):
            raise ValueError(f'Invalid {state} pose for {part}')
        norm = math.hypot(*pose[3:])
        if not math.isfinite(norm) or norm < 1e-8:
            raise ValueError(f'Invalid {state} quaternion for {part}')
        for axis in range(4):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = FRAME, stamp
            marker.ns, marker.id = state, index * 4 + axis
            marker.action = Marker.ADD
            # Follow the latest chassis TF; SDK polling and LiDAR clocks are not synchronized.
            marker.frame_locked = True
            marker.lifetime = Duration(seconds=lifetime).to_msg()
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, pose[:3])
            marker.color.a = alpha
            if axis < 3:
                marker.type = Marker.ARROW
                q = marker.pose.orientation
                q.x, q.y, q.z, q.w = (float(v / norm) for v in pose[3:])
                marker.points = [Point(), Point(**{('x', 'y', 'z')[axis]: length})]
                marker.scale.x, marker.scale.y, marker.scale.z = 0.008, 0.018, 0.025
                setattr(marker.color, ('r', 'g', 'b')[axis], 1.0)
            else:
                marker.type = Marker.TEXT_VIEW_FACING
                marker.pose.orientation.w = 1.0
                marker.pose.position.z += 0.19 if state == 'current' else 0.25
                marker.scale.z = 0.035
                marker.color.r = marker.color.g = marker.color.b = 1.0
                marker.text = f'{state} {part.removeprefix("astribot_")}'
            result.markers.append(marker)
    return result


def positive_rate(value):
    rate = float(value)
    if not math.isfinite(rate) or rate <= 0:
        raise argparse.ArgumentTypeError('rate must be finite and positive')
    return rate


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
