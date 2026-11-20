"""Prepare each camera's rectified RGB, CameraInfo and registered metric depth."""
from copy import deepcopy
from functools import partial
from pathlib import Path
import json
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import yaml

from astribot_scripts.common.rgbd import FramePairs, Projector, camera_model, image_array


class CameraInputs(Node):
    def __init__(self):
        super().__init__('rgbd_inputs')
        root = Path(__file__).resolve().parents[2]
        path = self.declare_parameter('calibration', str(root / 'config/rgbd_pointcloud.yaml')).value
        configs = yaml.safe_load(Path(path).read_text())['cameras']
        self.bridge = CvBridge()
        self.counts = {}
        self.last_received = {}
        self.registration_errors = {}
        for camera in ('head_rgbd', 'torso_rgbd'):
            self.add_camera(camera, configs[camera])
        self.status_pub = self.create_publisher(String, '/foundationpose/input_status', 2)
        self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        now = time.monotonic()
        status = {'counts': self.counts, 'registration_errors': self.registration_errors,
                  'seconds_since_input': {key: now - value for key, value in self.last_received.items()}}
        self.status_pub.publish(String(data=json.dumps(status)))

    def add_camera(self, camera, config):
        size, k, distortion = camera_model(config['color'])
        maps = cv2.initUndistortRectifyMap(k, distortion, None, k, size, cv2.CV_16SC2)
        info = CameraInfo(width=size[0], height=size[1], distortion_model='plumb_bob')
        info.k, info.r, info.p = k.ravel(), np.eye(3).ravel(), np.c_[k, np.zeros(3)].ravel()
        info.d = [0.] * 5
        try:
            projector = Projector(config, stride=1)
        except (KeyError, TypeError, ValueError) as exc:
            projector = None
            self.registration_errors[camera] = str(exc)
            self.get_logger().warning(f'{camera}: registered depth paused: {exc}')
        pairs = FramePairs(slop=0.06)
        prefix = f'/foundationpose/{camera}'
        color_pub = self.create_publisher(Image, prefix + '/rgb/image_rect_color', 2)
        info_pub = self.create_publisher(CameraInfo, prefix + '/rgb/camera_info', 2)
        depth_pub = self.create_publisher(Image, prefix + '/depth_registered/image_rect', 2)
        self.counts[camera] = dict(color=0, depth=0, pairs=0, registered=0)

        def receive(kind, msg):
            try:
                arr = image_array(msg)
                expected_size = config[kind]['width'], config[kind]['height']
                expected_encodings = ('rgb8', 'bgr8') if kind == 'color' else ('16UC1', '32FC1')
                if (msg.width, msg.height) != expected_size or msg.encoding not in expected_encodings:
                    raise ValueError(f'Unexpected {msg.width}x{msg.height} {msg.encoding}')
                expected_frame = config['optical_frame'] if kind == 'color' else config.get('depth_input_frame')
                if msg.header.frame_id and msg.header.frame_id != expected_frame:
                    raise ValueError(f'Unexpected {kind} frame {msg.header.frame_id!r}, expected {expected_frame!r}')
                counts = self.counts[camera]
                counts[kind] += 1
                self.last_received[camera + '/' + kind] = time.monotonic()
                if counts[kind] == 1:
                    self.get_logger().info(f'{camera} received {kind}: {msg.width}x{msg.height} {msg.encoding}')
                if kind == 'color':
                    rect = cv2.remap(arr, *maps, cv2.INTER_LINEAR)
                    image = self.bridge.cv2_to_imgmsg(rect, 'rgb8')
                    image.header = deepcopy(msg.header)
                    image.header.frame_id = config['optical_frame']
                    info.header = image.header
                    color_pub.publish(image)
                    info_pub.publish(info)
                pair = pairs.add(kind, msg, time.monotonic())
                if pair is None:
                    return
                counts['pairs'] += 1
                if projector is not None:
                    color, depth = pair
                    aligned = self.bridge.cv2_to_imgmsg(projector.rectified_depth(image_array(depth)), '32FC1')
                    aligned.header = deepcopy(color.header)
                    aligned.header.frame_id = config['optical_frame']
                    depth_pub.publish(aligned)
                    counts['registered'] += 1
            except (ValueError, cv2.error) as exc:
                self.get_logger().warning(f'{camera} {kind}: {exc}', throttle_duration_sec=5)

        # The routed camera link offers reliable delivery for the large images.
        for kind in ('color', 'depth'):
            self.create_subscription(Image, f'/sdk/astribot_camera/{camera}/{kind}_image',
                                     partial(receive, kind), 2)


def main():
    cv2.setNumThreads(1)
    rclpy.init()
    node = CameraInputs()
    previous = {}
    def report():
        if node.counts != previous:
            node.get_logger().info(f'Input frame totals: {node.counts}')
            previous.update(deepcopy(node.counts))
    node.create_timer(10.0, report)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
