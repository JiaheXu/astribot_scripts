#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) 2024, Astribot Co., Ltd.
# All rights reserved. License: BSD 3-Clause License
"""Activate camera/lidar drivers and publish standard ROS 2 Image/PointCloud2 topics.

The SDK Enter prompt is answered automatically. Images and scans are not saved.
Use --cameras to select color streams; head_rgbd also publishes raw depth.
No GUI or display server is used.
Press Ctrl-C to stop; both drivers remain running.
"""

import argparse
import threading
import time
from unittest.mock import patch

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, Imu, PointCloud2


CAMERAS = ("head_rgbd", "torso_rgbd", "left_wrist_rgbd", "right_wrist_rgbd", "head_stereo")

LIDAR_TOPICS = {"front": "/livox/lidar_front", "back": "/livox/lidar_back"}


def to_image_message(header, array):
    """Pack decoded BGR pixels while preserving the source header."""
    if (not isinstance(array, np.ndarray) or array.dtype != np.uint8
            or array.ndim != 3 or array.shape[2] != 3
            or array.shape[0] == 0 or array.shape[1] == 0):
        raise ValueError("Expected a nonempty uint8 BGR image with shape (height, width, 3)")
    message = Image()
    message.header = header
    message.height, message.width = array.shape[:2]
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = message.width * 3
    message.data = array.tobytes()
    return message


def to_depth_message(source, width, height, bigendian=False):
    """Wrap the driver's raw uint16 depth bytes without scaling or colorizing."""
    if source.format.strip().lower() != 'raw':
        raise ValueError(f'Expected raw uint16 depth, received format={source.format!r}')
    if width <= 0 or height <= 0:
        raise ValueError('Depth dimensions must be positive')
    expected = width * height * 2
    if len(source.data) != expected:
        raise ValueError(f'Depth frame has {len(source.data)} bytes; expected {expected} for {width}x{height} uint16')
    message = Image()
    message.header = source.header
    message.width = width
    message.height = height
    message.encoding = '16UC1'
    message.is_bigendian = int(bigendian)
    message.step = width * 2
    message.data = source.data
    return message


def add_sensor_arguments(parser, default_cameras=None):
    parser.add_argument("--cameras", nargs="+", choices=CAMERAS, default=default_cameras or ["head_rgbd"])
    parser.add_argument("--headless", action="store_true", help="compatibility option; always runs without a GUI")
    parser.add_argument("--head-depth-size", nargs=2, type=int, default=[640, 576], metavar=("WIDTH", "HEIGHT"))
    parser.add_argument("--depth-big-endian", action="store_true")


class SensorPublisher:
    """Share camera, depth, cloud and optional IMU publishing on one SDK connection."""
    def __init__(self, robot, cameras, depth_size=(640, 576), depth_big_endian=False, imu_topics=None):
        if any(value <= 0 for value in depth_size):
            raise ValueError('Depth dimensions must be positive')
        self.robot, self.node = robot, robot.astribot_interface.node
        self.cameras = list(dict.fromkeys(cameras))
        self.depth_size, self.depth_big_endian = depth_size, depth_big_endian
        self.imu_topics = imu_topics or {}
        self.counts = dict.fromkeys(self.cameras, 0)
        self.scan_counts = dict.fromkeys(LIDAR_TOPICS, 0)
        self.point_counts = dict.fromkeys(LIDAR_TOPICS, 0)
        self.imu_counts = dict.fromkeys(self.imu_topics, 0)
        self.lock = threading.Lock()
        self.subscriptions = []
        self.image_publishers, self.lidar_publishers, self.imu_publishers = {}, {}, {}
        self.depth_publisher = None
        self.depth_count = 0
        self.depth_error = None

    def image_callback(self, topic_name, msg, width, height, array):
        camera = topic_name.split('/')[2]
        try:
            image = to_image_message(msg.header, array)
        except ValueError as exc:
            self.node.get_logger().error(f'{camera}: {exc}', throttle_duration_sec=5.0)
            return
        self.image_publishers[camera].publish(image)
        with self.lock:
            self.counts[camera] += 1

    def lidar_callback(self, name, msg):
        self.lidar_publishers[name].publish(msg)
        with self.lock:
            self.scan_counts[name] += 1
            self.point_counts[name] += msg.width * msg.height

    def imu_callback(self, name, msg):
        self.imu_publishers[name].publish(msg)
        with self.lock:
            self.imu_counts[name] += 1

    def depth_callback(self, msg):
        try:
            image = to_depth_message(msg, *self.depth_size, self.depth_big_endian)
        except ValueError as exc:
            error = str(exc)
            if error != self.depth_error:
                self.node.get_logger().error(error)
            self.depth_error = error
            return
        self.depth_publisher.publish(image)
        self.depth_error = None
        with self.lock:
            self.depth_count += 1

    def start(self):
        if not self.robot.activate_camera():
            raise RuntimeError('Camera activation failed')
        if not self.robot.activate_lidar():
            raise RuntimeError('LiDAR activation failed')
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        for name, topic in LIDAR_TOPICS.items():
            output = f'/sdk/astribot_lidar/{name}_scan'
            self.lidar_publishers[name] = self.node.create_publisher(PointCloud2, output, qos)
            print(f'Publishing PointCloud2: {output}', flush=True)
            self.subscriptions.append(self.node.create_subscription(
                PointCloud2, topic, lambda msg, name=name: self.lidar_callback(name, msg), qos))
        for camera in self.cameras:
            output = f'/sdk/astribot_camera/{camera}/color_image'
            self.image_publishers[camera] = self.node.create_publisher(Image, output, qos)
            print(f'Publishing Image (bgr8): {output}', flush=True)
            subscription = self.robot.register_image_callback(camera, 'color', self.image_callback, need_decode=True)
            if subscription is None:
                raise RuntimeError(f'Could not subscribe to {camera}')
            self.subscriptions.append(subscription)
        if 'head_rgbd' in self.cameras:
            output = '/sdk/astribot_camera/head_rgbd/depth_image'
            self.depth_publisher = self.node.create_publisher(Image, output, qos)
            print(f'Publishing Image (16UC1): {output}', flush=True)
            # SDK depth registration would overwrite its color callback.
            self.subscriptions.append(self.node.create_subscription(
                CompressedImage, '/astribot_camera/head_rgbd/depth_compress', self.depth_callback, qos))
        for name, (source, output) in self.imu_topics.items():
            self.imu_publishers[name] = self.node.create_publisher(Imu, output, qos)
            print(f'Publishing Imu: {source} -> {output}', flush=True)
            self.subscriptions.append(self.node.create_subscription(
                Imu, source, lambda msg, name=name: self.imu_callback(name, msg), qos))

    def report(self):
        with self.lock:
            lidar = ', '.join(f'{name}: {self.scan_counts[name]} scans, {self.point_counts[name]} points'
                              for name in LIDAR_TOPICS)
            print(f'Received image frames: {self.counts}; head depth: {self.depth_count}; '
                  f'LiDAR: {lidar}; IMU: {self.imu_counts}', flush=True)

    def close(self):
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.subscriptions.clear()
        for publishers in (self.image_publishers, self.lidar_publishers, self.imu_publishers):
            for publisher in publishers.values():
                self.node.destroy_publisher(publisher)
            publishers.clear()
        if self.depth_publisher is not None:
            self.node.destroy_publisher(self.depth_publisher)
            self.depth_publisher = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_sensor_arguments(parser)
    args = parser.parse_args()
    if any(value <= 0 for value in args.head_depth_size):
        parser.error("--head-depth-size values must be positive")
    from astribot_sdk.core.astribot_api.astribot_client import Astribot

    sensors = None
    try:
        with patch("builtins.input", return_value=""):
            robot = Astribot()
        sensors = SensorPublisher(robot, args.cameras, args.head_depth_size, args.depth_big_endian)
        sensors.start()
        last_report = time.monotonic() - 1.0
        while rclpy.ok():
            if time.monotonic() - last_report >= 1.0:
                sensors.report()
                last_report = time.monotonic()
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        if sensors is not None:
            sensors.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
