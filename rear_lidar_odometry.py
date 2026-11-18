#!/usr/bin/env python3
"""Convert existing rear KISS-ICP poses to SDK chassis odometry; no SDK connection."""
import argparse
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

BASE_FRAME = 'astribot_viz_chassis'
ODOM_FRAME = 'kiss_rear_odom'


def load_calibration(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or data.get('base_from_cloud') is None:
        raise ValueError(f'{path}: base_from_cloud is missing; supply the calibrated cloud-to-chassis transform')
    matrix = np.asarray(data['base_from_cloud'], dtype=float)
    frame = data.get('cloud_frame')
    if not isinstance(frame, str) or not frame or frame.startswith('/') or frame in (BASE_FRAME, ODOM_FRAME):
        raise ValueError('cloud_frame must be a distinct, nonempty TF frame without a leading slash')
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('base_from_cloud must be a finite 4x4 matrix')
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6, rtol=0)):
        raise ValueError('base_from_cloud must be a rigid transform with rotation determinant +1')
    return frame, matrix


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    values = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    norm = np.linalg.norm(values[3:])
    if not np.isfinite(values).all() or not np.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid odometry position or quaternion')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(values[3:] / norm).as_matrix()
    result[:3, 3] = values[:3]
    return result


def set_pose(pose, matrix):
    pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
    q = pose.orientation
    q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(matrix[:3, :3]).as_quat())


def convert(source, base_from_cloud, cloud_frame):
    if source.header.frame_id != ODOM_FRAME or source.child_frame_id != cloud_frame:
        raise ValueError(f'Expected {ODOM_FRAME} -> {cloud_frame}, got '
                         f'{source.header.frame_id} -> {source.child_frame_id}')
    odom_from_cloud = pose_matrix(source.pose.pose)
    cloud_from_base = np.linalg.inv(base_from_cloud)
    result = Odometry()
    result.header = source.header
    result.child_frame_id = BASE_FRAME
    # Keep KISS's map origin: changing only the child frame uses right multiplication.
    set_pose(result.pose.pose, odom_from_cloud @ cloud_from_base)
    covariance = np.asarray(source.pose.covariance).reshape(6, 6)
    if not np.isfinite(covariance).all():
        raise ValueError('Invalid input pose covariance')
    x, y, z = odom_from_cloud[:3, :3] @ cloud_from_base[:3, 3]
    jacobian = np.eye(6)
    # Fixed-axis orientation errors in the parent frame move the offset base origin.
    jacobian[:3, 3:] = -np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    result.pose.covariance = (jacobian @ covariance @ jacobian.T).ravel().tolist()
    # KISS-ICP 1.3.0 publishes no velocity estimate; zero must not imply certainty.
    result.twist.covariance = (np.eye(6) * 1e12).ravel().tolist()
    return result


class RearBaseOdometry(Node):
    def __init__(self, cloud_frame, base_from_cloud):
        super().__init__('rear_lidar_base_odometry')
        self.cloud_frame, self.base_from_cloud = cloud_frame, base_from_cloud
        self.publisher = self.create_publisher(Odometry, '/kiss/base/odometry', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        mount = TransformStamped()
        mount.header.frame_id, mount.child_frame_id = BASE_FRAME, cloud_frame
        mount.header.stamp = self.get_clock().now().to_msg()
        t, q = mount.transform.translation, mount.transform.rotation
        t.x, t.y, t.z = map(float, base_from_cloud[:3, 3])
        q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(base_from_cloud[:3, :3]).as_quat())
        self.static_tf.sendTransform(mount)
        self.subscription = self.create_subscription(
            Odometry, '/kiss/rear/odometry', self.receive, qos_profile_sensor_data)
        self.get_logger().info('Publishing /kiss/base/odometry in kiss_rear_odom; pose only, no velocity estimate.')

    def receive(self, source):
        try:
            message = convert(source, self.base_from_cloud, self.cloud_frame)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        transform = TransformStamped()
        transform.header, transform.child_frame_id = message.header, message.child_frame_id
        p = message.pose.pose.position
        t = transform.transform.translation
        t.x, t.y, t.z = p.x, p.y, p.z
        transform.transform.rotation = message.pose.pose.orientation
        self.publisher.publish(message)
        self.tf.sendTransform(transform)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', default=str(Path(__file__).resolve().parent / 'config/rear_lidar_to_base.yaml'))
    args, ros_args = parser.parse_known_args()
    try:
        frame, matrix = load_calibration(args.calibration)
    except (ValueError, OSError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    # Keep visualization TF separate from the driver's tree, including standalone use.
    rclpy.init(args=[*ros_args, '--ros-args', '-r', '/tf:=/astribot_viz/tf',
                     '-r', '/tf_static:=/astribot_viz/tf_static'])
    node = RearBaseOdometry(frame, matrix)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
