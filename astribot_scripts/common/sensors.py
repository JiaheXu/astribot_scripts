# Copyright (c) 2024, Astribot Co., Ltd.
# All rights reserved. License: BSD 3-Clause License
"""Camera, depth, LiDAR and IMU message conversion and publishing."""
import threading
import time
import math

import cv2
import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, Imu, PointCloud2
from std_srvs.srv import SetBool
from astribot_scripts.common.cli import positive_int, positive_rate


CAMERAS = ("head_rgbd", "torso_rgbd", "left_wrist_rgbd", "right_wrist_rgbd", "head_stereo")

LIDAR_TOPICS = {"front": "/livox/lidar_front", "back": "/livox/lidar_back"}


def activate_sensor_service(node, service, timeout):
    """Use one persistent client while DDS discovers a sensor-only service.

    The SDK already spins this node in its own executor. Never spin it again.
    These two SetBool services start cameras/lidars; no robot control is taken.
    """
    if service not in ('/astribot_camera_control', '/astribot_lidar_control'):
        raise ValueError('Expected a camera or lidar activation service')
    client = node.create_client(SetBool, service)
    try:
        node.get_logger().info(f'Waiting up to {timeout:g}s for {service}')
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'Sensor service {service} unavailable after {timeout:g}s')
        request = SetBool.Request()
        request.data = True
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            future.cancel()
            raise RuntimeError(f'Sensor service {service} did not respond within {timeout:g}s')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f'Sensor service {service} rejected activation: '
                               f'{getattr(response, "message", "no response")}')
        node.get_logger().info(f'Sensor service {service}: {response.message}')
    finally:
        node.destroy_client(client)


def to_image_message(header, array, color_encoding='bgr8', image_size=None):
    """Pack decoded SDK BGR pixels as raw BGR/RGB, preserving the header."""
    if color_encoding not in ('bgr8', 'rgb8'):
        raise ValueError('Color encoding must be bgr8 or rgb8')
    if (not isinstance(array, np.ndarray) or array.dtype != np.uint8
            or array.ndim != 3 or array.shape[2] != 3
            or array.shape[0] == 0 or array.shape[1] == 0):
        raise ValueError("Expected a nonempty uint8 BGR image with shape (height, width, 3)")
    if image_size is not None:
        width, height = image_size
        if width <= 0 or height <= 0:
            raise ValueError('Color dimensions must be positive')
        if (array.shape[1], array.shape[0]) != (width, height):
            interpolation = (cv2.INTER_AREA if width <= array.shape[1] and height <= array.shape[0]
                             else cv2.INTER_LINEAR)
            array = cv2.resize(array, (width, height), interpolation=interpolation)
    message = Image()
    message.header = header
    message.height, message.width = array.shape[:2]
    message.encoding = color_encoding
    message.is_bigendian = 0
    message.step = message.width * 3
    message.data = (array[:, :, ::-1] if color_encoding == 'rgb8' else array).tobytes()
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
    parser.add_argument("--no-depth", action="store_true", help="publish color images only; do not subscribe to head depth")
    parser.add_argument("--color-encoding", choices=['bgr8', 'rgb8'], default='bgr8',
                        help="uncompressed Image channel order (default: bgr8)")
    parser.add_argument('--image-rate', type=positive_rate, default=None,
                        help='maximum raw color publication Hz per camera; decode only the newest frame per tick')
    parser.add_argument('--image-size', nargs=2, type=positive_int, default=None,
                        metavar=('WIDTH', 'HEIGHT'), help='resize raw color images to these exact dimensions')


class SensorPublisher:
    """Share camera, depth, cloud and optional IMU publishing on one SDK connection."""
    def __init__(self, robot, cameras, depth_size=(640, 576), depth_big_endian=False, imu_topics=None,
                 enable_depth=True, color_encoding='bgr8', image_rate=None, image_size=None):
        if any(value <= 0 for value in depth_size):
            raise ValueError('Depth dimensions must be positive')
        self.robot, self.node = robot, robot.astribot_interface.node
        self.cameras = list(dict.fromkeys(cameras))
        self.depth_size, self.depth_big_endian = depth_size, depth_big_endian
        if color_encoding not in ('bgr8', 'rgb8'):
            raise ValueError('Color encoding must be bgr8 or rgb8')
        self.enable_depth, self.color_encoding = enable_depth, color_encoding
        if image_rate is not None and (not math.isfinite(image_rate) or image_rate <= 0):
            raise ValueError('image_rate must be finite and positive')
        self.image_rate = image_rate
        if image_size is not None and (len(image_size) != 2 or any(
                not isinstance(value, int) or value <= 0 for value in image_size)):
            raise ValueError('image_size must contain positive integer width and height')
        self.image_size = tuple(image_size) if image_size is not None else None
        self.latest_compressed = {}
        self.image_timer = None
        self.image_receive_group = ReentrantCallbackGroup()
        self.image_publish_group = MutuallyExclusiveCallbackGroup()
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

    def compressed_callback(self, camera, message):
        # Bound memory/latency to one compressed frame per camera.
        with self.lock:
            self.latest_compressed[camera] = message

    def publish_latest_images(self):
        with self.lock:
            latest = self.latest_compressed
            self.latest_compressed = {}
        for camera, message in latest.items():
            try:
                if not any(fmt in message.format.lower() for fmt in ('jpeg', 'jpg', 'png')):
                    raise ValueError(f'Expected compressed color image, got {message.format!r}')
                bgr = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
                if bgr is None:
                    raise ValueError('Could not decode compressed color image')
                self.image_callback(f'/astribot_camera/{camera}/color_compress/compressed',
                                    message, bgr.shape[1], bgr.shape[0], bgr)
            except (ValueError, cv2.error) as exc:
                self.node.get_logger().error(f'{camera}: {exc}', throttle_duration_sec=5.0)

    def image_callback(self, topic_name, msg, width, height, array):
        camera = topic_name.split('/')[2]
        try:
            image = to_image_message(msg.header, array, self.color_encoding, self.image_size)
        except (ValueError, cv2.error) as exc:
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

    def start(self, activation_timeout=0.0):
        if activation_timeout > 0:
            activate_sensor_service(self.node, '/astribot_camera_control', activation_timeout)
            activate_sensor_service(self.node, '/astribot_lidar_control', activation_timeout)
        else:
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
            size = f'{self.image_size[0]}x{self.image_size[1]}' if self.image_size else 'source dimensions'
            print(f'Publishing Image ({self.color_encoding}, {size}): {output}', flush=True)
            if self.image_rate is None:
                subscription = self.robot.register_image_callback(camera, 'color', self.image_callback, need_decode=True)
            else:
                source = f'/astribot_camera/{camera}/color_compress/compressed'
                subscription = self.node.create_subscription(
                    CompressedImage, source, lambda msg, name=camera: self.compressed_callback(name, msg),
                    QoSProfile(depth=1), callback_group=self.image_receive_group)
                print(f'Compressed color input: {source}; raw output at {self.image_rate:g} Hz', flush=True)
            if subscription is None:
                raise RuntimeError(f'Could not subscribe to {camera}')
            self.subscriptions.append(subscription)
        if self.image_rate is not None:
            self.image_timer = self.node.create_timer(1.0 / self.image_rate, self.publish_latest_images,
                                                      callback_group=self.image_publish_group)
        if self.enable_depth and 'head_rgbd' in self.cameras:
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
            depth = self.depth_count if self.enable_depth else 'disabled'
            print(f'Received image frames: {self.counts}; head depth: {depth}; '
                  f'LiDAR: {lidar}; IMU: {self.imu_counts}', flush=True)

    def close(self):
        if self.image_timer is not None:
            self.node.destroy_timer(self.image_timer)
            self.image_timer = None
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
