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
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from astribot_sdk.core.astribot_api.astribot_client import Astribot


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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cameras", nargs="+", choices=CAMERAS, default=["head_rgbd"])
    parser.add_argument("--headless", action="store_true", help="compatibility option; this script always runs without a GUI")
    parser.add_argument("--head-depth-size", nargs=2, type=int, default=[640, 576], metavar=("WIDTH", "HEIGHT"),
                        help="raw uint16 head depth dimensions (default: 640 576)")
    parser.add_argument("--depth-big-endian", action="store_true", help="declare depth bytes big-endian (default: little-endian)")
    args = parser.parse_args()
    if any(value <= 0 for value in args.head_depth_size):
        parser.error("--head-depth-size values must be positive")
    cameras = list(dict.fromkeys(args.cameras))
    counts = dict.fromkeys(cameras, 0)
    scan_counts = dict.fromkeys(LIDAR_TOPICS, 0)
    point_counts = dict.fromkeys(LIDAR_TOPICS, 0)
    lock = threading.Lock()
    subscriptions = []
    image_publishers = {}
    lidar_publishers = {}
    depth_publisher = None
    depth_count = 0
    depth_error = None
    astribot = None

    def image_callback(topic_name, msg, width, height, array):
        # msg.header.stamp is the source timestamp; decoded color arrays are BGR.
        camera = topic_name.split("/")[2]
        image_publishers[camera].publish(to_image_message(msg.header, array))
        with lock:
            counts[camera] += 1

    def lidar_callback(name, msg):
        # Relay the complete standard message, preserving Livox fields and timing.
        lidar_publishers[name].publish(msg)
        with lock:
            scan_counts[name] += 1
            point_counts[name] += msg.width * msg.height

    def depth_callback(msg):
        nonlocal depth_count, depth_error
        try:
            image = to_depth_message(msg, *args.head_depth_size, args.depth_big_endian)
        except ValueError as exc:
            error = str(exc)
            if error != depth_error:
                node.get_logger().error(error)
            depth_error = error
            return
        depth_publisher.publish(image)
        depth_error = None
        with lock:
            depth_count += 1

    try:
        # Answer the SDK control-rights prompt as Enter, only during startup.
        with patch("builtins.input", return_value=""):
            astribot = Astribot()
        if not astribot.activate_camera():
            raise RuntimeError("Camera activation failed")
        if not astribot.activate_lidar():
            raise RuntimeError("LiDAR activation failed")
        node = astribot.astribot_interface.node
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        for name, topic in LIDAR_TOPICS.items():
            output_topic = f"/sdk/astribot_lidar/{name}_scan"
            lidar_publishers[name] = node.create_publisher(PointCloud2, output_topic, qos)
            print(f"Publishing PointCloud2: {output_topic}", flush=True)
            subscriptions.append(node.create_subscription(
                PointCloud2, topic,
                lambda msg, name=name: lidar_callback(name, msg), qos,
            ))
        for camera in cameras:
            output_topic = f"/sdk/astribot_camera/{camera}/color_image"
            image_publishers[camera] = node.create_publisher(Image, output_topic, qos)
            print(f"Publishing Image (bgr8): {output_topic}", flush=True)
            subscription = astribot.register_image_callback(
                camera, "color", image_callback, need_decode=True
            )
            if subscription is None:
                raise RuntimeError(f"Could not subscribe to {camera}")
            subscriptions.append(subscription)
        if 'head_rgbd' in cameras:
            output_topic = '/sdk/astribot_camera/head_rgbd/depth_image'
            depth_publisher = node.create_publisher(Image, output_topic, qos)
            print(f'Publishing Image (16UC1): {output_topic}', flush=True)
            # Subscribe directly: registering depth via the SDK overwrites its color callback.
            subscriptions.append(node.create_subscription(
                CompressedImage, '/astribot_camera/head_rgbd/depth_compress', depth_callback, qos,
            ))
        last_report = time.monotonic() - 1.0
        while rclpy.ok():
            with lock:
                received = dict(counts)
                scans = dict(scan_counts)
                points = dict(point_counts)
                depths = depth_count
            if time.monotonic() - last_report >= 1.0:
                lidar = ", ".join(
                    f"{name}: {scans[name]} scans, {points[name]} points"
                    for name in LIDAR_TOPICS
                )
                print(f"Received image frames: {received}; head depth: {depths}; LiDAR: {lidar}", flush=True)
                last_report = time.monotonic()
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        if astribot is not None:
            for subscription in subscriptions:
                astribot.astribot_interface.node.destroy_subscription(subscription)
            for publisher in (*image_publishers.values(), *lidar_publishers.values()):
                astribot.astribot_interface.node.destroy_publisher(publisher)
            if depth_publisher is not None:
                astribot.astribot_interface.node.destroy_publisher(depth_publisher)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
