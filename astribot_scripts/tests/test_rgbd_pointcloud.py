"""Numerical projection, registration, binary layout and stream failure tests."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np
import yaml

from astribot_scripts.common.rgbd import (FramePairs, Projector, image_array,
                                          transform_points, POINT_DTYPE)

ROOT = Path(__file__).resolve().parents[1]


def config():
    model = {'width': 4, 'height': 3, 'camera_matrix': [2, 0, 1, 0, 2, 1, 0, 0, 1],
             'distortion_model': 'plumb_bob', 'distortion_coefficients': [0.] * 5}
    return {'color': deepcopy(model), 'depth': deepcopy(model),
            'depth_unit_m': .001, 'color_from_depth': np.eye(4).tolist()}


def message(timestamp, **kwargs):
    return NS(header=NS(frame_id='', stamp=NS(sec=timestamp // 10**9, nanosec=timestamp % 10**9)), **kwargs)


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.color = np.arange(36, dtype=np.uint8).reshape(3, 4, 3)
        self.depth = np.full((3, 4), 1000, dtype=np.uint16)

    def test_millimeters_to_meters_optical_axes_and_exact_rgb(self):
        points = Projector(config(), stride=1).project(self.color, self.depth)
        self.assertEqual(len(points), 12)
        np.testing.assert_allclose([points[0][k] for k in ('x', 'y', 'z')], [-.5, -.5, 1])
        np.testing.assert_allclose([points[5][k] for k in ('x', 'y', 'z')], [0, 0, 1])
        self.assertEqual(int(points[5]['rgb']), (15 << 16) | (16 << 8) | 17)

    def test_depth_to_color_baseline_changes_pixel_and_output_coordinates(self):
        calibration = config()
        calibration['color_from_depth'][0][3] = .5
        points = Projector(calibration, stride=1).project(self.color, self.depth)
        self.assertEqual(len(points), 9)  # Rightmost depth column projects outside RGB.
        np.testing.assert_allclose([points[0][k] for k in ('x', 'y', 'z')], [0, -.5, 1])
        self.assertEqual(int(points[0]['rgb']), (3 << 16) | (4 << 8) | 5)

    def test_distinct_depth_and_rgb_resolutions_and_intrinsics(self):
        calibration = config()
        calibration['depth'].update(width=2, height=1, camera_matrix=[1, 0, 0, 0, 1, 0, 0, 0, 1])
        points = Projector(calibration, stride=1).project(self.color, self.depth[:1, :2])
        np.testing.assert_allclose(points['x'], [0, 1])
        np.testing.assert_allclose(points['y'], [0, 0])
        self.assertEqual(int(points[0]['rgb']), (15 << 16) | (16 << 8) | 17)
        self.assertEqual(int(points[1]['rgb']), (21 << 16) | (22 << 8) | 23)

    def test_float_depth_is_already_meters_and_invalid_depth_is_dropped(self):
        depth = self.depth.astype(np.float32) / 1000
        depth[0] = [0, np.nan, np.inf, 9]
        depth[1, 0] = -.5
        points = Projector(config(), stride=1).project(self.color, depth)
        self.assertEqual(len(points), 7)
        np.testing.assert_allclose(points['z'], 1)
        depth[:] = 0
        self.assertEqual(len(Projector(config()).project(self.color, depth)), 0)

    def test_color_distortion_changes_registered_pixel(self):
        calibration = config()
        calibration['color']['width'] = 8
        calibration['color']['distortion_coefficients'][0] = 1.
        color = np.zeros((3, 8, 3), dtype=np.uint8)
        color[1, 5] = [255, 0, 0]
        depth = np.zeros_like(self.depth)
        depth[1, 3] = 1000  # x/z=1 -> distorted x=2 -> color u=5.
        points = Projector(calibration, stride=1).project(color, depth)
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(float(points['x'][0]), 1)
        self.assertEqual(int(points['rgb'][0]), 0xFF0000)

    def test_occluded_points_keep_nearest_color_surface(self):
        calibration = config()
        calibration['color']['camera_matrix'][0] = .1  # Several depth pixels share a color pixel.
        depth = np.zeros_like(self.depth)
        depth[1, 1:3] = [2000, 1000]
        points = Projector(calibration, stride=1).project(self.color, depth)
        self.assertEqual(len(points), 1)
        self.assertEqual(float(points['z'][0]), 1)

    def test_rectified_depth_z_buffer_uses_rgb_raster_and_meters(self):
        calibration = config()
        calibration['color'].update(width=5, camera_matrix=[.1, 0, 2, 0, 2, 1, 0, 0, 1])
        calibration['color_from_depth'][2][3] = .25
        depth = np.zeros_like(self.depth)
        depth[1, 1:3] = [1000, 2000]
        result = Projector(calibration, stride=1).rectified_depth(depth)
        expected = np.zeros((3, 5), dtype=np.float32)
        expected[1, 2] = 1.25
        self.assertEqual(result.dtype, np.float32)
        np.testing.assert_array_equal(result, expected)

    def test_map_transform_preserves_color_and_source_points(self):
        points = Projector(config(), stride=1).project(self.color, self.depth)
        matrix = np.array([[0, -1, 0, 3], [1, 0, 0, 4], [0, 0, 1, 5], [0, 0, 0, 1]])
        result = transform_points(points, matrix)
        np.testing.assert_allclose([result[0][k] for k in ('x', 'y', 'z')], [3.5, 3.5, 6])
        np.testing.assert_array_equal(result['rgb'], points['rgb'])
        self.assertEqual(float(points['z'][0]), 1)

    def test_missing_calibration_units_and_invalid_rigid_transform_fail(self):
        for field in ('depth_unit_m', 'color_from_depth'):
            calibration = config()
            calibration[field] = None
            with self.assertRaises(ValueError):
                Projector(calibration)
        calibration = config()
        calibration['depth']['camera_matrix'] = None
        with self.assertRaises(ValueError):
            Projector(calibration)
        calibration = config()
        calibration['color_from_depth'][0][0] = 2
        with self.assertRaises(ValueError):
            Projector(calibration)

    def test_stride_and_mismatched_image_sizes(self):
        projector = Projector(config(), stride=2)
        self.assertEqual(len(projector.project(self.color, self.depth)), 4)
        with self.assertRaises(ValueError):
            projector.project(self.color[:, :2], self.depth)
        with self.assertRaises(ValueError):
            projector.project(self.color, self.depth[:2])


class ImageAndPairTests(unittest.TestCase):
    def test_padded_big_endian_depth_rows(self):
        image = NS(encoding='16UC1', is_bigendian=1, width=2, height=2, step=6,
                   data=struct.pack('>HHH', 1000, 2000, 999) + struct.pack('>HHH', 3000, 4000, 999))
        np.testing.assert_array_equal(image_array(image), [[1000, 2000], [3000, 4000]])

    def test_padded_bgr_becomes_rgb(self):
        image = NS(encoding='bgr8', is_bigendian=0, width=1, height=2, step=4,
                   data=bytes([1, 2, 3, 99, 4, 5, 6, 99]))
        np.testing.assert_array_equal(image_array(image), [[[3, 2, 1]], [[6, 5, 4]]])
        image.data = image.data[:-1]
        with self.assertRaises(ValueError):
            image_array(image)

    def test_delayed_color_matches_nearest_source_depth_once(self):
        pairs = FramePairs(slop=.02)
        first, nearest, color = message(1_000_000_000), message(1_030_000_000), message(1_025_000_000)
        pairs.add('depth', first, 1)
        pairs.add('depth', nearest, 1.03)
        self.assertEqual(pairs.add('color', color, 1.3), (color, nearest))
        self.assertIsNone(pairs.add('color', color, 1.31))
        self.assertIsNone(pairs.add('depth', nearest, 1.32))

    def test_color_first_and_unsynchronized_frames(self):
        pairs = FramePairs(slop=.02)
        color, depth = message(2_000_000_000), message(2_005_000_000)
        self.assertIsNone(pairs.add('color', color, 1))
        self.assertEqual(pairs.add('depth', depth, 1.01), (color, depth))
        pairs.add('color', message(3_000_000_000), 2)
        self.assertIsNone(pairs.add('depth', message(3_500_000_000), 2.01))
        with self.assertRaises(ValueError):
            pairs.add('color', message(0), 3)

    def test_stale_queues_expire_and_memory_is_bounded(self):
        pairs = FramePairs(max_age=.2, queue_size=3)
        pairs.add('color', message(1_000_000_000), 1)
        self.assertIsNone(pairs.add('depth', message(1_000_000_000), 2))
        for i in range(20):
            pairs.add('depth', message(2_000_000_000 + i), 2 + i * .001)
        self.assertEqual(len(pairs.queues['depth']), 3)
        self.assertEqual(len(pairs.queues['color']), 0)


class Image:
    pass


class Cloud:
    def __init__(self):
        self.header = NS()


class Field(NS):
    FLOAT32 = 7


def node_module():
    modules = {'rclpy': Mock(), 'rclpy.node': NS(Node=object),
               'rclpy.executors': NS(ExternalShutdownException=RuntimeError),
               'rclpy.qos': NS(QoSProfile=Mock(), ReliabilityPolicy=Mock()),
               'rclpy.time': NS(Time=Mock()),
               'sensor_msgs.msg': NS(CompressedImage=object, Image=Image, PointCloud2=Cloud, PointField=Field),
               'tf2_ros': NS(Buffer=Mock(), TransformListener=Mock(), TransformException=RuntimeError)}
    spec = importlib.util.spec_from_file_location('rgbd_node_under_test', ROOT / 'rgbd_pointcloud.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class MessageTests(unittest.TestCase):
    def test_pcl_binary_layout_rgb_bits_and_source_stamp(self):
        module = node_module()
        points = np.array([(1, 2, 3, 0x12AB34)], dtype=POINT_DTYPE)
        stamp = NS(sec=42, nanosec=123)
        msg = module.cloud_message(points, 'optical', stamp)
        self.assertEqual((msg.header.frame_id, msg.header.stamp.sec, msg.header.stamp.nanosec), ('optical', 42, 123))
        self.assertIsNot(msg.header.stamp, stamp)
        self.assertEqual((msg.point_step, msg.row_step, len(msg.data)), (16, 16, 16))
        self.assertEqual(struct.unpack('<fffI', msg.data), (1, 2, 3, 0x12AB34))
        self.assertEqual([(f.name, f.offset, f.datatype) for f in msg.fields],
                         [('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('rgb', 12, 7)])

    def test_raw_depth_preserves_header_dimensions_units_and_rejects_bad_payload(self):
        module = node_module()
        source = message(1_000_000_000, format='raw', data=bytes(24))
        image = module.depth_image(source, config())
        self.assertEqual((image.width, image.height, image.step, image.encoding), (4, 3, 8, '16UC1'))
        self.assertEqual(image.header, source.header)
        self.assertEqual(image.data, source.data)
        source.data = bytes(20)
        with self.assertRaises(ValueError):
            module.depth_image(source, config())

    def test_rviz_clouds_are_rgb8_in_map_with_short_decay_and_no_wrist_clouds(self):
        view = yaml.safe_load((ROOT / 'config/astribot_fast_lio.rviz').read_text())['Visualization Manager']
        group = next(d for d in view['Displays'] if d['Name'] == 'Head and torso RGB-D clouds')
        self.assertEqual({d['Topic']['Value'] for d in group['Displays']},
                         {'/astribot_viz/head_rgbd/points', '/astribot_viz/torso_rgbd/points'})
        for display in group['Displays']:
            self.assertTrue(display['Enabled'])
            self.assertEqual(display['Color Transformer'], 'RGB8')
            self.assertEqual(display['Decay Time'], 1)
        self.assertEqual(view['Global Options']['Fixed Frame'], 'map')


class Time:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = nanoseconds

    @classmethod
    def from_msg(cls, stamp):
        return cls(stamp.sec * 10**9 + stamp.nanosec)

    def to_msg(self):
        return NS(sec=self.nanoseconds // 10**9, nanosec=self.nanoseconds % 10**9)


class PublishingTests(unittest.TestCase):
    def setUp(self):
        self.module = node_module()
        self.module.Time = Time
        self.node = object.__new__(self.module.RGBDPointCloud)
        node = self.node
        calibration = config()
        calibration['optical_frame'] = 'color_optical'
        node.configs = {'head_rgbd': calibration}
        source = message(42_000_000_000, format='raw', data=np.full((3, 4), 1000, '<u2').tobytes())
        color = message(42_000_000_000, encoding='rgb8', is_bigendian=0, width=4, height=3,
                        step=12, data=bytes([255, 0, 0] * 12))
        node.pending = {'head_rgbd': (10., (color, source))}
        node.latest_depth = {'head_rgbd': (10., source)}
        node.projectors = {'head_rgbd': Projector(calibration, stride=1)}
        node.publishers_ = {'head_rgbd': {name: Mock() for name in ('depth', 'cloud', 'map')}}
        node.counts = {'head_rgbd': {'clouds': 0, 'map_clouds': 0}}
        node.max_age, node.max_tf_age, node.tf_mode, node.fixed_frame = 1., 1., 'latest', 'map'
        self.tf = NS(header=NS(stamp=NS(sec=123, nanosec=0)),
                     transform=NS(translation=NS(x=3., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))
        node.buffer = Mock()
        node.buffer.lookup_transform.return_value = self.tf
        node.get_clock = lambda: NS(now=lambda: Time(123_000_000_000))
        node.get_logger = Mock()

    def test_source_cloud_keeps_capture_stamp_and_map_cloud_freezes_latest_tf(self):
        with patch.object(self.module.time, 'monotonic', return_value=10.1):
            self.node.publish()
            self.node.publish()  # No repeat of a consumed sample.
        publishers = self.node.publishers_['head_rgbd']
        for publisher in publishers.values():
            publisher.publish.assert_called_once()
        source = publishers['cloud'].publish.call_args.args[0]
        display = publishers['map'].publish.call_args.args[0]
        self.assertEqual((source.header.frame_id, source.header.stamp.sec), ('color_optical', 42))
        self.assertEqual((display.header.frame_id, display.header.stamp.sec), ('map', 123))
        self.assertAlmostEqual(struct.unpack_from('<f', display.data)[0], 2.5)
        self.assertEqual(self.node.buffer.lookup_transform.call_args.args[2].nanoseconds, 0)

    def test_measurement_mode_uses_capture_time_and_stale_latest_tf_is_rejected(self):
        self.node.tf_mode = 'measurement'
        with patch.object(self.module.time, 'monotonic', return_value=10.1):
            self.node.publish()
        self.assertEqual(self.node.buffer.lookup_transform.call_args.args[2].nanoseconds, 42_000_000_000)
        self.assertEqual(self.node.publishers_['head_rgbd']['map'].publish.call_args.args[0].header.stamp.sec, 42)
        self.setUp()
        self.tf.header.stamp.sec = 120
        with patch.object(self.module.time, 'monotonic', return_value=10.1):
            self.node.publish()
        self.node.publishers_['head_rgbd']['cloud'].publish.assert_called_once()
        self.node.publishers_['head_rgbd']['map'].publish.assert_not_called()

    def test_missing_calibration_publishes_depth_only_and_stale_samples_are_not_replayed(self):
        self.node.projectors['head_rgbd'] = None
        with patch.object(self.module.time, 'monotonic', return_value=10.1):
            self.node.publish()
        self.node.publishers_['head_rgbd']['depth'].publish.assert_called_once()
        self.node.publishers_['head_rgbd']['cloud'].publish.assert_not_called()
        self.node.publishers_['head_rgbd']['map'].publish.assert_not_called()
        self.setUp()
        with patch.object(self.module.time, 'monotonic', return_value=12):
            self.node.publish()
        for publisher in self.node.publishers_['head_rgbd'].values():
            publisher.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
