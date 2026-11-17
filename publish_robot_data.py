#!/usr/bin/env python3
"""Publish S1 cameras, depth, LiDAR, three IMUs, joints and Cartesian poses.

Uses one SDK connection and no GUI. Drivers remain active on exit.
Sensor headers are preserved; joint/pose stamps are SDK polling time.
"""
import argparse
import json
import math
import time
from unittest.mock import patch

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from get_lidar_and_img import CAMERAS, SensorPublisher, add_sensor_arguments
from rviz_state import FRAME, MODEL_JOINTS, POSE_PARTS, joint_message, pose_markers, positive_rate, robot_description

IMU_TOPICS = {
    'front': ('/livox/imu_front', '/sdk/astribot_lidar/front_imu'),
    'back': ('/livox/imu_back', '/sdk/astribot_lidar/back_imu'),
    'chassis': ('/astribot_whole_body/imu_sensor/astribot_chassis_imu_site', '/sdk/astribot_chassis/imu'),
}
STATE_TOPICS = {
    '/astribot_viz/joint_states': JointState,
    '/sdk/desired_joint_states': JointState,
    '/sdk/robot_state': String,
}
for _state in ('current', 'desired'):
    STATE_TOPICS[f'/astribot_viz/{_state}_poses'] = MarkerArray
    STATE_TOPICS[f'/sdk/chassis/{_state}_pose'] = PoseStamped
    for _part in POSE_PARTS:
        STATE_TOPICS[f'/sdk/cartesian/{_state}/{_part}'] = PoseStamped


def pose_message(values, stamp, frame):
    if values is None or len(values) != 7 or not all(math.isfinite(v) for v in values):
        raise ValueError('Expected a finite [x, y, z, qx, qy, qz, qw] pose')
    norm = math.hypot(*values[3:])
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid pose quaternion')
    message = PoseStamped()
    message.header.stamp, message.header.frame_id = stamp, frame
    p, q = message.pose.position, message.pose.orientation
    p.x, p.y, p.z = map(float, values[:3])
    q.x, q.y, q.z, q.w = (float(v / norm) for v in values[3:])
    return message


def stamp_dict(stamp):
    return {'sec': stamp.sec, 'nanosec': stamp.nanosec}


def read_state(robot, lifetime):
    """Validate a polling cycle before publishing; retain non-URDF SDK values too."""
    node = robot.astribot_interface.node
    stamp = node.get_clock().now().to_msg()
    names, dofs = robot.whole_body_names, robot.whole_body_dofs
    if len(names) != len(dofs) or not set(POSE_PARTS).issubset(names):
        raise ValueError('Incomplete S1 body metadata')
    snapshot = {'schema_version': 1, 'timestamp_basis': 'sdk_polling_time',
                'poll_started': stamp_dict(stamp), 'states': {}}
    messages = {}
    for state in ('current', 'desired'):
        joints = {}
        for quantity in ('position', 'velocity', 'torque'):
            rows = getattr(robot, f'get_{state}_joints_{quantity}')(names=names)
            if rows is None or len(rows) != len(names):
                raise ValueError(f'Incomplete {state} joint {quantity}')
            joints[quantity] = {}
            for part, dof, row in zip(names, dofs, rows):
                if row is None or len(row) != dof or not all(math.isfinite(v) for v in row):
                    raise ValueError(f'Invalid {state} joint {quantity} for {part}')
                joints[quantity][part] = list(map(float, row))
        snapshot['states'][state] = joints
        message = joint_message([joints['position'][part] for part in MODEL_JOINTS], stamp)
        message.header.frame_id = FRAME
        for quantity, field in [('velocity', 'velocity'), ('torque', 'effort')]:
            # Reuse the model's length/finite checks for all three arrays.
            checked = joint_message([joints[quantity][part] for part in MODEL_JOINTS], stamp)
            setattr(message, field, checked.position)
        topic = '/astribot_viz/joint_states' if state == 'current' else '/sdk/desired_joint_states'
        messages[topic] = message
        getter = getattr(robot, f'get_{state}_cartesian_pose')
        poses = getter(names=POSE_PARTS, frame='chassis')
        messages[f'/astribot_viz/{state}_poses'] = pose_markers(poses, stamp, state, lifetime)
        for part, pose in zip(POSE_PARTS, poses):
            messages[f'/sdk/cartesian/{state}/{part}'] = pose_message(pose, stamp, FRAME)
        world = getter(names=['astribot_chassis'], frame='world')
        if world is None or len(world) != 1:
            raise ValueError(f'Incomplete {state} chassis world pose')
        messages[f'/sdk/chassis/{state}_pose'] = pose_message(world[0], stamp, 'astribot_sdk_world')
    snapshot['poll_finished'] = stamp_dict(node.get_clock().now().to_msg())
    messages['/sdk/robot_state'] = String(data=json.dumps(snapshot, allow_nan=False, separators=(',', ':')))
    return messages


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_sensor_arguments(parser, default_cameras=list(CAMERAS))
    parser.add_argument('--rate', type=positive_rate, default=50.0, help='joint/pose polling Hz (default: 50)')
    args = parser.parse_args()
    if any(value <= 0 for value in args.head_depth_size):
        parser.error('--head-depth-size values must be positive')
    description = robot_description()
    from astribot_sdk.core.astribot_api.astribot_client import Astribot

    sensors, node = None, None
    publishers = {}
    try:
        with patch('builtins.input', return_value=''):
            robot = Astribot()
        if robot.robot_type != 'S1':
            raise ValueError('This publisher uses the bundled S1 model')
        node = robot.astribot_interface.node
        sensors = SensorPublisher(robot, args.cameras, args.head_depth_size, args.depth_big_endian, IMU_TOPICS)
        sensors.start()
        for topic, kind in STATE_TOPICS.items():
            publishers[topic] = node.create_publisher(kind, topic, 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publishers['/sdk/robot_description'] = node.create_publisher(String, '/sdk/robot_description', qos)
        publishers['/sdk/robot_description'].publish(String(data=description))
        node.get_logger().info('Publishing all robot data. Joint/pose stamps are polling time, '
                               'not synchronized measurement time. No motion commands are sent.')
        last_report, samples = time.monotonic() - 1.0, 0
        while rclpy.ok():
            started = time.monotonic()
            if not robot.astribot_interface.is_alive():
                raise RuntimeError('SDK connection lost')
            try:
                messages = read_state(robot, max(1.0, 3.0 / args.rate))
            except (ValueError, TypeError) as exc:
                node.get_logger().warning(f'Skipping invalid state: {exc}', throttle_duration_sec=5.0)
            else:
                for topic, message in messages.items():
                    publishers[topic].publish(message)
                samples += 1
            if started - last_report >= 1.0:
                sensors.report()
                print(f'Published joint/pose samples: {samples}', flush=True)
                last_report = started
            # The SDK already spins its node in a background executor.
            time.sleep(max(0.0, 1.0 / args.rate - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        if sensors is not None:
            sensors.close()
        for publisher in publishers.values():
            node.destroy_publisher(publisher)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
