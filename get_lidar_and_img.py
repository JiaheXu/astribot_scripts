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
import time
from unittest.mock import patch

import rclpy

from astribot_scripts.common.sensors import (
    CAMERAS, LIDAR_TOPICS, SensorPublisher, add_sensor_arguments,
    to_depth_message, to_image_message,
)


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
