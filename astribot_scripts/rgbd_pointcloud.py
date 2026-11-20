#!/usr/bin/env python3
"""Head/torso RGB-D clouds and a frozen map-frame overlay for combined RViz."""
from copy import deepcopy
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from astribot_scripts.common.rgbd import FramePairs, Projector, image_array, transform_points

CONFIG = Path(__file__).resolve().parent / 'config/rgbd_pointcloud.yaml'


def depth_image(source, config):
    """Wrap the driver's raw uint16 CompressedImage; never resize depth."""
    if isinstance(source, Image):
        if source.encoding not in ('16UC1', '32FC1'):
            raise ValueError('Depth Image must be 16UC1 or 32FC1')
        image_array(source)  # Validate row stride, payload and dimensions.
        return source
    width, height = config['depth']['width'], config['depth']['height']
    if source.format.strip().lower() != 'raw' or len(source.data) != width * height * 2:
        raise ValueError(f'Expected raw uint16 depth payload of {width}x{height}')
    result = Image()
    result.header = deepcopy(source.header)
    result.width, result.height = width, height
    result.encoding, result.step = '16UC1', width * 2
    result.is_bigendian = int(config.get('depth_bigendian', False))
    result.data = source.data
    return result


def cloud_message(points, frame, stamp):
    result = PointCloud2()
    result.header.frame_id, result.header.stamp = frame, deepcopy(stamp)
    result.height, result.width = 1, len(points)
    result.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                     for i, name in enumerate(('x', 'y', 'z', 'rgb'))]
    # Packed 0x00RRGGBB bits in the PCL-compatible FLOAT32 rgb field.
    result.is_bigendian, result.is_dense = False, True
    result.point_step, result.row_step = 16, 16 * len(points)
    result.data = points.tobytes()
    return result


def transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    values = np.asarray([t.x, t.y, t.z, q.x, q.y, q.z, q.w])
    if not np.isfinite(values).all() or np.linalg.norm(values[3:]) < 1e-8:
        raise ValueError('Invalid camera TF')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    matrix[:3, 3] = values[:3]
    return matrix


class RGBDPointCloud(Node):
    def __init__(self):
        super().__init__('astribot_rgbd_pointcloud')
        config_path = self.declare_parameter('config', str(CONFIG)).value
        rate = float(self.declare_parameter('rate', 3.0).value)
        stride = self.declare_parameter('stride', 2).value
        near = float(self.declare_parameter('min_depth', 0.2).value)
        far = float(self.declare_parameter('max_depth', 5.0).value)
        slop = float(self.declare_parameter('sync_slop', 0.06).value)
        self.max_age = float(self.declare_parameter('max_pair_age', 1.0).value)
        self.max_tf_age = float(self.declare_parameter('max_tf_age', 1.0).value)
        self.tf_mode = self.declare_parameter('tf_time_mode', 'latest').value
        self.fixed_frame = self.declare_parameter('fixed_frame', 'map').value
        if (not all(np.isfinite(v) and v > 0 for v in (rate, self.max_age, self.max_tf_age))
                or self.tf_mode not in ('latest', 'measurement') or not self.fixed_frame
                or type(stride) is not int or stride < 1 or not 0 < near < far or not np.isfinite(far)):
            raise ValueError('Invalid RGB-D rate/range/TF parameters')
        self.configs = yaml.safe_load(Path(config_path).read_text())['cameras']
        if set(self.configs) != {'head_rgbd', 'torso_rgbd'}:
            raise ValueError('Configure exactly head_rgbd and torso_rgbd; wrist clouds are not enabled')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pairs, self.projectors, self.pending, self.latest_depth = {}, {}, {}, {}
        self.publishers_, self.subscriptions_, self.counts = {}, [], {}
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        for camera, config in self.configs.items():
            self.pairs[camera] = FramePairs(slop, self.max_age)
            self.counts[camera] = dict(color=0, depth=0, pairs=0, clouds=0, map_clouds=0)
            try:
                self.projectors[camera] = Projector(config, stride, near, far)
            except (ValueError, KeyError, TypeError) as exc:
                self.projectors[camera] = None
                self.get_logger().error(f'{camera}: cloud disabled: {exc}. Edit {config_path} and restart. '
                                        'Sampled depth images remain available.')
            for kind in ('color', 'depth'):
                msg_type = (CompressedImage if kind == 'depth' and config['depth_transport'] == 'raw_compressed'
                            else Image)
                if config['depth_transport'] not in ('raw_compressed', 'image'):
                    raise ValueError('depth_transport must be raw_compressed or image')
                self.subscriptions_.append(self.create_subscription(
                    msg_type, config[f'{kind}_topic'],
                    lambda msg, c=camera, k=kind: self.receive(c, k, msg), qos))
            topics = {'depth': f'/sdk/astribot_camera/{camera}/depth_image',
                      'cloud': f'/sdk/astribot_camera/{camera}/points',
                      'map': f'/astribot_viz/{camera}/points'}
            if config['depth_topic'] == topics['depth']:
                # A standard Image input can already be the sampled depth output.
                topics.pop('depth')
            self.publishers_[camera] = {
                kind: self.create_publisher(Image if kind == 'depth' else PointCloud2, topic, qos)
                for kind, topic in topics.items()}
        self.timer = self.create_timer(1.0 / rate, self.publish)
        self.report_timer = self.create_timer(10.0, self.report)
        self.get_logger().info(f'Head/torso RGB-D: {rate:g} Hz, stride {stride}, {near:g}–{far:g} m; '
                               f'{self.tf_mode} TF to {self.fixed_frame}. '
                               'Latest TF uses nominal mounts for an approximate display overlay.')

    def receive(self, camera, kind, message):
        now = time.monotonic()
        config = self.configs[camera]
        expected = config['optical_frame'] if kind == 'color' else config.get('depth_input_frame', '')
        if message.header.frame_id and message.header.frame_id != expected:
            self.get_logger().warning(f'{camera} {kind}: unexpected frame {message.header.frame_id!r}; '
                                      f'expected {expected!r}', throttle_duration_sec=5)
            return
        self.counts[camera][kind] += 1
        if kind == 'depth':
            self.latest_depth[camera] = (now, message)
        try:
            pair = self.pairs[camera].add(kind, message, now)
        except ValueError as exc:
            self.get_logger().warning(f'{camera}: {exc}', throttle_duration_sec=5)
            return
        if pair is not None:
            self.pending[camera] = (now, pair)
            self.counts[camera]['pairs'] += 1

    def publish(self):
        now = time.monotonic()
        for camera, config in self.configs.items():
            pending = self.pending.pop(camera, None)
            sample = self.latest_depth.pop(camera, None)
            if pending is not None and now - pending[0] <= self.max_age:
                sample = (pending[0], pending[1][1])
            else:
                pending = None
            if sample is None or now - sample[0] > self.max_age:
                continue
            try:
                depth = depth_image(sample[1], config)
                publishers = self.publishers_[camera]
                if 'depth' in publishers:
                    publishers['depth'].publish(depth)
                if pending is None or self.projectors[camera] is None:
                    continue
                points = self.projectors[camera].project(image_array(pending[1][0]), image_array(depth))
                publishers['cloud'].publish(cloud_message(points, config['optical_frame'], depth.header.stamp))
                self.counts[camera]['clouds'] += 1
                when = Time() if self.tf_mode == 'latest' else Time.from_msg(depth.header.stamp)
                tf = self.buffer.lookup_transform(self.fixed_frame, config['optical_frame'], when)
                stamp = self.get_clock().now()
                if self.tf_mode == 'latest' and (tf.header.stamp.sec or tf.header.stamp.nanosec):
                    age = (stamp.nanoseconds - Time.from_msg(tf.header.stamp).nanoseconds) / 1e9
                    if not -0.1 <= age <= self.max_tf_age:
                        raise ValueError(f'Camera TF is stale/future ({age:.3f}s)')
                frozen = transform_points(points, transform_matrix(tf.transform))
                publishers['map'].publish(cloud_message(frozen, self.fixed_frame,
                                                        stamp.to_msg() if self.tf_mode == 'latest' else depth.header.stamp))
                self.counts[camera]['map_clouds'] += 1
            except (ValueError, cv2.error, TransformException) as exc:
                self.get_logger().warning(f'{camera}: {exc}', throttle_duration_sec=5)

    def report(self):
        self.get_logger().info(f'RGB-D received/published: {self.counts}')
        for camera, counts in self.counts.items():
            if counts['color'] and counts['depth'] and not counts['pairs']:
                self.get_logger().warning(f'{camera}: no timestamp-matched pairs; check camera clocks and sync_slop')


def main():
    cv2.setNumThreads(1)
    rclpy.init()
    node = None
    try:
        node = RGBDPointCloud()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
