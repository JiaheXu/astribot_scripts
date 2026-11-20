#!/usr/bin/env python3
"""Republish the color topics from 300-get_images.py as raw RGB images.

The camera driver must already be active (for example, call
Astribot().activate_camera() in the application that controls the robot).
This node subscribes directly to the SDK's ROS topics and does not acquire
robot control rights or activate/deactivate hardware.

From the SDK directory:
    source ./env.sh
    python3 examples/304-publish_rgb_images.py
    python3 examples/304-publish_rgb_images.py --cameras head_rgbd torso_rgbd

Input:  /astribot_camera/<camera>/color_compress/compressed (CompressedImage)
Output: /astribot_camera/<camera>/color/image_raw (Image, rgb8)

In another terminal, source ./env.sh and run rviz2. Add an Image display,
select an output topic, choose raw transport and Best Effort reliability.
The Image display does not need CameraInfo or TF. Images retain their
original resolution, timestamp and frame_id. The head_stereo stream is
published as received, without splitting the stereo image.
"""

import argparse
from functools import partial
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CompressedImage, Image


CAMERAS = (
    "head_rgbd",
    "torso_rgbd",
    "left_wrist_rgbd",
    "right_wrist_rgbd",
    "head_stereo",
)


class RgbImageRepublisher(Node):
    """Decode compressed camera frames and publish RViz-compatible images."""

    def __init__(self, cameras):
        super().__init__("astribot_rgb_image_republisher")
        self.image_publishers = {}
        self.image_subscriptions = []
        self.last_received = {}
        self.input_topics = {}
        for camera in dict.fromkeys(cameras):
            source = f"/astribot_camera/{camera}/color_compress/compressed"
            target = f"/astribot_camera/{camera}/color/image_raw"
            self.image_publishers[camera] = self.create_publisher(
                Image, target, qos_profile_sensor_data,
            )
            self.last_received[camera] = time.monotonic()
            self.input_topics[camera] = source
            self.image_subscriptions.append(self.create_subscription(
                CompressedImage, source, partial(self.image_callback, camera),
                qos_profile_sensor_data,
            ))
            self.get_logger().info(f"{source} -> {target} (rgb8)")
        self.create_timer(5.0, self.check_streams)

    def image_callback(self, camera, compressed):
        try:
            bgr = cv2.imdecode(
                np.frombuffer(compressed.data, dtype=np.uint8), cv2.IMREAD_COLOR,
            )
            if bgr is None:
                raise ValueError("image decoder returned no image")
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        except (cv2.error, ValueError) as error:
            self.get_logger().warning(
                f"Cannot decode {camera} image ({compressed.format!r}): {error}",
                throttle_duration_sec=5.0,
            )
            return

        image = Image()
        image.header = compressed.header
        image.height, image.width = rgb.shape[:2]
        image.encoding = "rgb8"
        image.is_bigendian = 0  # Each channel is a single byte.
        image.step = image.width * 3
        image.data = rgb.tobytes()
        self.image_publishers[camera].publish(image)
        self.last_received[camera] = time.monotonic()

    def check_streams(self):
        now = time.monotonic()
        for camera, last in self.last_received.items():
            if now - last >= 5.0:
                self.get_logger().warning(
                    f"No decoded image on {self.input_topics[camera]} for "
                    f"{now - last:.0f}s; check that the camera driver is active."
                )


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cameras", nargs="+", choices=CAMERAS, default=CAMERAS,
                        help="Cameras to republish (default: all five).")
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    rclpy.init(args=args)
    node = None
    try:
        node = RgbImageRepublisher(options.cameras)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
