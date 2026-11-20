#!/usr/bin/env python3
"""Convert rear LIO to chassis odometry and publish its display-time TF.

Chassis odometry/path retain source stamps. Only visualization TF is retimed;
source IMU odometry, clouds and sensor TF remain unchanged.
"""
from copy import deepcopy
import math
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster

from astribot_scripts.common.fast_lio_viz import rear_imu_from_sdk_chassis
from astribot_scripts.common.state import FRAME


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    norm = math.hypot(*values[3:])
    if not all(math.isfinite(v) for v in values) or not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid FAST-LIO pose')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(np.asarray(values[3:]) / norm).as_matrix()
    matrix[:3, 3] = values[:3]
    return matrix


def set_pose(pose, matrix):
    pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
    q = pose.orientation
    q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(matrix[:3, :3]).as_quat())


def chassis_odometry(source, imu_from_chassis):
    """Change the tracked body by right multiplication; keep the world origin."""
    if source.header.frame_id != 'map' or source.child_frame_id != 'body':
        raise ValueError('Expected rear FAST-LIO map -> body odometry')
    world_from_imu = pose_matrix(source.pose.pose)
    result = Odometry()
    result.header = deepcopy(source.header)
    result.child_frame_id = FRAME
    set_pose(result.pose.pose, world_from_imu @ imu_from_chassis)
    covariance = np.asarray(source.pose.covariance).reshape(6, 6)
    if not np.isfinite(covariance).all():
        raise ValueError('Invalid FAST-LIO covariance')
    # Fixed-axis orientation uncertainty moves the offset chassis origin.
    x, y, z = world_from_imu[:3, :3] @ imu_from_chassis[:3, 3]
    jacobian = np.eye(6)
    jacobian[:3, 3:] = -np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    result.pose.covariance = (jacobian @ covariance @ jacobian.T).ravel().tolist()
    # Upstream FAST-LIO does not populate a measured twist.
    result.twist.covariance = (np.eye(6) * 1e12).ravel().tolist()
    return result


def chassis_path(source, imu_from_chassis):
    if source.header.frame_id != 'map':
        raise ValueError('Expected FAST-LIO path in map')
    result = Path()
    result.header = deepcopy(source.header)
    for item in source.poses:
        if item.header.frame_id != 'map':
            raise ValueError('Expected FAST-LIO path pose in map')
        pose = PoseStamped()
        pose.header = deepcopy(item.header)
        set_pose(pose.pose, pose_matrix(item.pose) @ imu_from_chassis)
        result.poses.append(pose)
    return result


def display_transform(source, stamp):
    if source.header.frame_id != 'map' or source.child_frame_id != FRAME:
        raise ValueError(f'Expected converted chassis odometry map -> {FRAME}')
    p, q = source.pose.pose.position, source.pose.pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    norm = math.hypot(*values[3:])
    if not all(math.isfinite(v) for v in values) or not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid FAST-LIO pose')
    transform = TransformStamped()
    transform.header.frame_id, transform.header.stamp = 'map', stamp
    transform.child_frame_id = FRAME
    t, r = transform.transform.translation, transform.transform.rotation
    t.x, t.y, t.z = p.x, p.y, p.z
    r.x, r.y, r.z, r.w = (v / norm for v in values[3:])
    return transform


class DisplayTF(Node):
    def __init__(self):
        super().__init__('fast_lio_display_tf')
        self.latest, self.received = None, 0.0
        self.imu_from_chassis = rear_imu_from_sdk_chassis()
        self.tf = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, '/fast_lio/chassis/odometry', 10)
        self.path_publisher = self.create_publisher(Path, '/fast_lio/chassis/path', 10)
        self.subscription = self.create_subscription(
            Odometry, '/fast_lio/rear/odometry', self.receive, qos_profile_sensor_data)
        self.path_subscription = self.create_subscription(
            Path, '/fast_lio/rear/path', self.receive_path, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.publish)
        self.get_logger().info(f'Robot base: {FRAME}; chassis odometry/path preserve source stamps. '
                               'Only visualization TF uses display time.')

    def receive(self, message):
        try:
            converted = chassis_odometry(message, self.imu_from_chassis)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        self.odom_publisher.publish(converted)
        self.latest, self.received = converted, time.monotonic()

    def receive_path(self, message):
        try:
            converted = chassis_path(message, self.imu_from_chassis)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        self.path_publisher.publish(converted)

    def publish(self):
        if self.latest is not None and time.monotonic() - self.received < 2.0:
            self.tf.sendTransform(display_transform(self.latest, self.get_clock().now().to_msg()))


def main():
    rclpy.init()
    node = DisplayTF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
