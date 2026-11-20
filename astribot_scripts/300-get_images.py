#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) 2024, Astribot Co., Ltd.
# All rights reserved. License: BSD 3-Clause License
"""Receive live color images without saving files or activating cameras by default.

Select --cameras, use --headless for counts only, or --activate to start cameras.
Press q in an image window or Ctrl-C to stop. Camera drivers are left running.
"""

import argparse
import threading
import time
from unittest.mock import patch

import cv2
import rclpy
from astribot_sdk.core.astribot_api.astribot_client import Astribot


CAMERAS = ("head_rgbd", "torso_rgbd", "left_wrist_rgbd", "right_wrist_rgbd", "head_stereo")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cameras", nargs="+", choices=CAMERAS, default=["head_rgbd"])
    parser.add_argument("--headless", action="store_true", help="print counts without decoding/displaying images")
    parser.add_argument("--activate", action="store_true", help="request camera activation before listening")
    args = parser.parse_args()
    cameras = list(dict.fromkeys(args.cameras))
    counts = dict.fromkeys(cameras, 0)
    latest = {}
    lock = threading.Lock()
    subscriptions = []
    astribot = None

    def image_callback(topic_name, msg, width, height, array):
        # msg.header.stamp is the source timestamp; decoded color arrays are BGR.
        camera = topic_name.split("/")[2]
        with lock:
            counts[camera] += 1
            if not args.headless:
                latest[camera] = array

    try:
        # Answer the SDK control-rights prompt as Enter, only during startup.
        with patch("builtins.input", return_value=""):
            astribot = Astribot()
        if args.activate and not astribot.activate_camera():
            raise RuntimeError("Camera activation failed")
        for camera in cameras:
            subscription = astribot.register_image_callback(
                camera, "color", image_callback, need_decode=not args.headless
            )
            if subscription is None:
                raise RuntimeError(f"Could not subscribe to {camera}")
            subscriptions.append(subscription)
        last_report = time.monotonic() - 1.0
        while rclpy.ok():
            with lock:
                frames = dict(latest)
                latest.clear()
                received = dict(counts)
            # OpenCV windows run on the main thread, outside the SDK callback.
            if not args.headless:
                for camera, frame in frames.items():
                    cv2.imshow(camera, frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            if time.monotonic() - last_report >= 1.0:
                print(f"Received image frames: {received}", flush=True)
                last_report = time.monotonic()
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        if astribot is not None:
            for subscription in subscriptions:
                astribot.astribot_interface.node.destroy_subscription(subscription)
        if not args.headless:
            cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
