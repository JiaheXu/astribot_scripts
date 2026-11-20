#!/usr/bin/env python3
"""Adapt rear FAST-LIO feedback and gate Nav2 velocity output for the SDK.

No SDK connection is created here. The existing control_mobile_base receiver
consumes /astribot/nav_cmd_vel. Use launch/astribot_nav2.launch.py on ROS Humble.
"""
from copy import deepcopy
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from scipy.spatial.transform import Rotation
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from astribot_scripts.common.control import command_qos, finite_vector
from astribot_scripts.common.fast_lio_viz import rear_imu_from_sdk_chassis
from astribot_scripts.common.nav2 import (BASE_FRAME, IMU_FRAME, LIDAR_FRAME, ODOM_FRAME,
                                         FeedbackGate, OdometryProjector, rigid_matrix)
from astribot_scripts.common.nav2_goals import RvizGoals


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def stamp_message(value):
    sec, nanosec = divmod(int(value), 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def set_pose(pose, matrix):
    pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
    q = pose.orientation
    q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(matrix[:3, :3]).as_quat())


def transform(parent, child, matrix, stamp):
    result = TransformStamped()
    result.header.frame_id, result.header.stamp = parent, stamp
    result.child_frame_id = child
    t, q = result.transform.translation, result.transform.rotation
    t.x, t.y, t.z = map(float, matrix[:3, 3])
    q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(matrix[:3, :3]).as_quat())
    return result


def load_mounts(path=''):
    if path:
        values = json.loads(Path(path).read_text())
        return rigid_matrix(values['base_from_imu']), rigid_matrix(values['imu_from_lidar'])
    # Same nominal geometry as the current rear-LIO profile, not per-robot calibration.
    base_from_imu = np.linalg.inv(rear_imu_from_sdk_chassis())
    imu_from_lidar = np.eye(4)
    imu_from_lidar[:3, 3] = [-0.011, -0.02329, 0.04412]
    return base_from_imu, imu_from_lidar


class Nav2Bridge(Node):
    def __init__(self):
        super().__init__('astribot_nav2_bridge')
        self.offset = float(self.declare_parameter('stamp_offset', 0.0).value)
        timeout = float(self.declare_parameter('feedback_timeout', 0.5).value)
        self.command_timeout = float(self.declare_parameter('command_timeout', 0.3).value)
        self.enable_output = bool(self.declare_parameter('enable_output', False).value)
        mount_file = self.declare_parameter('mount_file', '').value
        if not np.isfinite([self.offset, timeout, self.command_timeout]).all() or min(timeout, self.command_timeout) <= 0:
            raise ValueError('Timeouts must be finite/positive and stamp_offset must be finite')
        base_from_imu, imu_from_lidar = load_mounts(mount_file)
        self.projector = OdometryProjector(np.linalg.inv(base_from_imu))
        self.gate = FeedbackGate(timeout=timeout)
        self.latest_command = None
        self.last_cloud_stamp = None
        self.last_reason = None
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        stamp = self.get_clock().now().to_msg()
        self.static_tf.sendTransform([
            transform(BASE_FRAME, IMU_FRAME, base_from_imu, stamp),
            transform(IMU_FRAME, LIDAR_FRAME, imu_from_lidar, stamp)])
        self.odom_pub = self.create_publisher(Odometry, '/astribot_nav/odom', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/astribot_nav/points', qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/astribot/nav_cmd_vel', 1)
        self.status_pub = self.create_publisher(String, '/astribot_nav/status', 1)
        self.subscriptions_ = [
            self.create_subscription(Odometry, '/fast_lio/rear/odometry', self.receive_odometry,
                                     qos_profile_sensor_data),
            self.create_subscription(PointCloud2, '/fast_lio/rear/cloud_registered_body',
                                     self.receive_cloud, qos_profile_sensor_data),
            self.create_subscription(Twist, '/astribot_nav/cmd_vel', self.receive_command, command_qos())]
        self.rviz_goals = RvizGoals(self, self.projector, self.feedback_reason)
        self.timer = self.create_timer(0.05, self.publish_command)
        self.get_logger().info(
            f'Nav2 feedback bridge: {ODOM_FRAME} -> {BASE_FRAME}; stamp_offset={self.offset:g}s. '
            'Source timestamps are preserved apart from this explicit constant offset. '
            + ('Using supplied mounts.' if mount_file else 'Using existing nominal rear sensor mounts.'))

    def corrected_stamp(self, stamp):
        value = stamp_ns(stamp) + round(self.offset * 1e9)
        age = (self.get_clock().now().nanoseconds - value) * 1e-9
        if value <= 0 or not -self.gate.future_tolerance <= age < self.gate.timeout:
            raise ValueError(f'Sensor stamp age {age:.3f}s; synchronize clocks or set a verified stamp_offset')
        return value

    def reject(self, stream, exc):
        self.gate.invalidate(stream)
        self.latest_command = None
        self.get_logger().warning(f'Rejected {stream}: {exc}', throttle_duration_sec=2.0)

    def receive_odometry(self, message):
        try:
            if message.header.frame_id != 'map' or message.child_frame_id != 'body':
                raise ValueError('Expected rear FAST-LIO map -> body odometry')
            stamp = self.corrected_stamp(message.header.stamp)
            p, q = message.pose.pose.position, message.pose.pose.orientation
            pose, twist = self.projector.project([p.x, p.y, p.z], [q.x, q.y, q.z, q.w], stamp)
        except (ValueError, TypeError) as exc:
            self.reject('odom', exc)
            return
        result = Odometry()
        result.header.frame_id, result.header.stamp = ODOM_FRAME, stamp_message(stamp)
        result.child_frame_id = BASE_FRAME
        set_pose(result.pose.pose, pose)
        # Uncertainty is not calibrated here. Do not pass through upstream zero
        # twist or its one-message-late pose covariance as precise measurements.
        result.pose.covariance = (np.eye(6) * 1e6).ravel().tolist()
        result.twist.covariance = (np.eye(6) * 1e6).ravel().tolist()
        if twist is not None:
            linear, angular = result.twist.twist.linear, result.twist.twist.angular
            linear.x, linear.y, linear.z = map(float, twist[:3])
            angular.x, angular.y, angular.z = map(float, twist[3:])
            self.gate.mark('odom', stamp * 1e-9, time.monotonic())
        else:
            self.gate.invalidate('odom')
        self.odom_pub.publish(result)
        self.tf.sendTransform(transform(ODOM_FRAME, BASE_FRAME, pose, result.header.stamp))

    def receive_cloud(self, message):
        try:
            if message.header.frame_id != 'body':
                raise ValueError('Expected FAST-LIO body-frame cloud, not raw /livox/lidar_back')
            stamp = self.corrected_stamp(message.header.stamp)
            if self.last_cloud_stamp is not None and stamp <= self.last_cloud_stamp:
                raise ValueError('Cloud timestamp did not advance')
            fields = {field.name for field in message.fields}
            if (not {'x', 'y', 'z'}.issubset(fields) or message.width * message.height == 0
                    or message.point_step <= 0 or message.row_step < message.width * message.point_step
                    or len(message.data) < message.row_step * message.height):
                raise ValueError('Empty or malformed obstacle cloud')
        except (ValueError, TypeError) as exc:
            self.reject('cloud', exc)
            return
        output = deepcopy(message)
        # This is an exact frame alias: body is the rear IMU in this FAST-LIO
        # output. The static transform above defines its navigation counterpart.
        output.header.frame_id, output.header.stamp = IMU_FRAME, stamp_message(stamp)
        self.cloud_pub.publish(output)
        self.last_cloud_stamp = stamp
        self.gate.mark('cloud', stamp * 1e-9, time.monotonic())

    def feedback_reason(self):
        return self.gate.reason(self.get_clock().now().nanoseconds * 1e-9, time.monotonic())

    def receive_command(self, message):
        if not self.enable_output or self.feedback_reason() is not None:
            self.latest_command = None
            return
        try:
            values = finite_vector([message.linear.x, message.linear.y, message.angular.z,
                                    message.linear.z, message.angular.x, message.angular.y], 6, 'Nav2 Twist')
            if any(values[3:]):
                raise ValueError('Nav2 command must be planar')
        except (TypeError, ValueError) as exc:
            self.latest_command = None
            self.get_logger().warning(str(exc), throttle_duration_sec=2.0)
            return
        self.latest_command = (deepcopy(message), time.monotonic())

    def publish_command(self):
        reason = self.feedback_reason()
        if reason is not None:
            self.latest_command = None
        output = Twist()
        if self.latest_command is not None:
            message, received = self.latest_command
            if time.monotonic() - received < self.command_timeout:
                output = message
            else:
                self.latest_command = None
        state = reason or ('feedback ready' if self.enable_output else 'feedback ready; preview output disabled')
        if state != self.last_reason:
            self.get_logger().info(f'Nav2 command gate: {state}')
            self.last_reason = state
        self.status_pub.publish(String(data=state))
        if self.enable_output:
            self.cmd_pub.publish(output)


def main():
    rclpy.init()
    node = None
    try:
        node = Nav2Bridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok() and node.enable_output:
                node.cmd_pub.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
