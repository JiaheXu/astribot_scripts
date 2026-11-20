"""Offline tag-center geometry, expiry, frame handling and launch tests."""
import importlib.util
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]


def vector(x=0., y=0., z=0.):
    return NS(x=x, y=y, z=z)


def header(frame='camera', stamp=123):
    return NS(frame_id=frame, stamp=NS(sec=stamp, nanosec=0))


def pose(x=0., y=0., z=1., q=(0., 0., 0., 1.)):
    return NS(position=vector(x, y, z), orientation=NS(**dict(zip(('x', 'y', 'z', 'w'), q))))


class Marker:
    ADD, DELETE, ARROW, TEXT_VIEW_FACING = 0, 2, 0, 9

    def __init__(self):
        self.header, self.pose = header(), pose()
        self.scale, self.color = vector(), NS(r=0., g=0., b=0., a=0.)
        self.points = []
        self.frame_locked = False


class Time:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = nanoseconds

    @classmethod
    def from_msg(cls, stamp):
        return cls(stamp.sec * 10**9 + stamp.nanosec)

    def to_msg(self):
        return NS(sec=self.nanoseconds // 10**9, nanosec=self.nanoseconds % 10**9)


class FakeNode:
    params = {}

    def __init__(self, name):
        self.log = Mock()
        self.now = Time(200 * 10**9)
        self.created_subscriptions = []

    def declare_parameter(self, name, default):
        return NS(value=self.params.get(name, default))

    def get_clock(self):
        return NS(now=lambda: self.now)

    def get_logger(self):
        return self.log

    def create_publisher(self, *args):
        return Mock()

    def create_subscription(self, *args):
        self.created_subscriptions.append(args)
        return args


def detection(ident=7, tag_pose=None):
    return NS(id=str(ident), header=header(), results=[NS(
        hypothesis=NS(class_id=str(ident)), pose=NS(pose=tag_pose or pose()))])


def detections(*items):
    return NS(header=header(), detections=list(items))


def load_modules():
    modules = {
        'rclpy': Mock(), 'rclpy.node': NS(Node=FakeNode),
        'rclpy.duration': NS(Duration=lambda seconds: NS(to_msg=lambda: NS(sec=seconds))),
        'rclpy.time': NS(Time=Time), 'rclpy.executors': NS(ExternalShutdownException=RuntimeError),
        'rclpy.qos': NS(QoSProfile=lambda **kw: kw, ReliabilityPolicy=NS(BEST_EFFORT='best')),
        'geometry_msgs.msg': NS(Point=vector),
        'visualization_msgs.msg': NS(Marker=Marker, MarkerArray=lambda: NS(markers=[])),
        'vision_msgs.msg': NS(Detection2DArray=object),
        'tf2_ros': NS(Buffer=Mock, TransformException=RuntimeError, TransformListener=Mock()),
    }
    helper_spec = importlib.util.spec_from_file_location(
        'astribot_scripts.common.apriltag_markers', ROOT / 'common/apriltag_markers.py')
    helper = importlib.util.module_from_spec(helper_spec)
    node_spec = importlib.util.spec_from_file_location('apriltag_rviz_under_test', ROOT / 'apriltag_rviz.py')
    node = importlib.util.module_from_spec(node_spec)
    with patch.dict(sys.modules, modules):
        helper_spec.loader.exec_module(helper)
        sys.modules[helper_spec.name] = helper
        node_spec.loader.exec_module(node)
    return helper, node


class AxisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.helper, cls.node_module = load_modules()

    def setUp(self):
        self.axes = self.helper.TagAxes('tag36h11', {7: .1, 8: .15})

    def test_axes_start_at_pose_center_with_tag_rotation_and_rgb_colors(self):
        rotation = Rotation.from_euler('z', np.pi / 2)
        message = detections(detection(tag_pose=pose(.2, -.1, 1., rotation.as_quat())))
        markers = self.axes.update('head', message, np.eye(4), 'map', header().stamp).markers
        self.assertEqual(len(markers), 4)
        for i, marker in enumerate(markers[:3]):
            self.assertEqual(vars(marker.pose.position), {'x': .2, 'y': -.1, 'z': 1.})
            self.assertEqual(vars(marker.points[0]), vars(vector()))
            expected = np.eye(3)[i] * .1
            np.testing.assert_allclose(list(vars(marker.points[1]).values()), expected)
            self.assertEqual([marker.color.r, marker.color.g, marker.color.b], list(np.eye(3)[i]))
            q = marker.pose.orientation
            np.testing.assert_allclose(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
                                       rotation.as_matrix(), atol=1e-12)
            self.assertFalse(marker.frame_locked)
            self.assertEqual(marker.lifetime.sec, 1.0)
        self.assertEqual(markers[3].text, 'head: tag36h11 / 7')

    def test_camera_transform_composes_with_tag_center_and_orientation(self):
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_euler('y', np.pi / 2).as_matrix()
        matrix[:3, 3] = [2., 3., 4.]
        result = self.axes.update('head', detections(detection()), matrix, 'map', header().stamp)
        p = result.markers[0].pose.position
        np.testing.assert_allclose([p.x, p.y, p.z], [3., 3., 4.])

    def test_missing_tag_deleted_without_removing_other_cameras(self):
        first = self.axes.update('head', detections(detection(), detection(8)), np.eye(4), 'map', header().stamp)
        wrist = self.axes.update('wrist', detections(detection()), np.eye(4), 'map', header().stamp)
        self.assertNotEqual(first.markers[0].ns, wrist.markers[0].ns)
        result = self.axes.update('head', detections(detection(8)), np.eye(4), 'map', header().stamp)
        deletes = [m for m in result.markers if m.action == Marker.DELETE]
        self.assertEqual({m.id for m in deletes}, {28, 29, 30, 31})
        self.assertTrue(all('/head/' in m.ns for m in deletes))
        self.assertEqual(self.axes.visible['wrist'], {7})
        cleared = self.axes.clear('head', detections(), 'map', header().stamp)
        self.assertEqual(len(cleared.markers), 4)
        self.assertTrue(all(m.action == Marker.DELETE for m in cleared.markers))

    def test_invalid_unconfigured_and_duplicate_detections_are_skipped(self):
        bad_class = detection()
        bad_class.results[0].hypothesis.class_id = '8'
        bad_frame = detection()
        bad_frame.header.frame_id = 'wrong'
        missing_pose = detection()
        missing_pose.results = []
        bad_items = [detection(999), bad_class, bad_frame, missing_pose,
                     detection(tag_pose=pose(z=-1)), detection(tag_pose=pose(x=np.nan)),
                     detection(tag_pose=pose(q=(0, 0, 0, 0)))]
        result = self.axes.update('head', detections(*bad_items, detection(), detection()),
                                  np.eye(4), 'map', header().stamp)
        self.assertEqual(len(result.markers), 4)

    def test_axis_length_and_lifetime_are_validated(self):
        for length, lifetime in ((0, 1), (np.nan, 1), (.1, -1), (.1, np.inf)):
            with self.assertRaises(ValueError):
                self.helper.TagAxes('tag36h11', {7: .1}, length, lifetime)

    def test_shared_configuration_uses_tag36h11_and_metric_lengths(self):
        with tempfile.TemporaryDirectory() as directory:
            cameras, tags = Path(directory) / 'cameras.yaml', Path(directory) / 'tags.yaml'
            cameras.write_text(yaml.safe_dump({'cameras': {
                'head': {'frame_id': 'astribot_viz_head_rgbd_optical', 'detections_topic': '/head/tags'},
                'wrist': {'frame_id': 'astribot_viz_left_wrist_rgbd_optical', 'detections_topic': '/wrist/tags'}}}))
            tags.write_text('family: tag36h11\ntags:\n  - id: 7\n    length_mm: 100.0\n')
            sources, family, lengths = self.helper.load_sources(cameras, tags)
            self.assertEqual(family, 'tag36h11')
            self.assertEqual(lengths, {7: .1})
            self.assertEqual(sources['head'], ('/head/tags', 'astribot_viz_head_rgbd_optical'))
            tags.write_text('family: tag36h11\ntags:\n  - id: 587\n    length_mm: 100.0\n')
            with self.assertRaises(ValueError):
                self.helper.load_sources(cameras, tags)

    def test_aruco_size_and_dictionary_reach_axis_label(self):
        with tempfile.TemporaryDirectory() as directory:
            cameras, tags = Path(directory) / 'cameras.yaml', Path(directory) / 'tags.yaml'
            cameras.write_text('cameras:\n  head:\n    frame_id: camera\n')
            tags.write_text('family: DICT_5X5_100\ntags:\n  - id: 1\n    length_mm: 60.0\n')
            sources, family, lengths = self.helper.load_sources(cameras, tags)
            self.assertEqual(lengths, {1: .06})
            axes = self.helper.TagAxes(family, lengths)
            markers = axes.update('head', detections(detection(1)), np.eye(4), 'map', header().stamp).markers
            self.assertEqual(len(markers), 4)
            self.assertEqual(markers[3].text, 'head: DICT_5X5_100 / 1')
            tags.write_text('family: DICT_5X5_100\ntags:\n  - id: 100\n    length_mm: 60.0\n')
            with self.assertRaises(ValueError):
                self.helper.load_sources(cameras, tags)


class AdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.helper, cls.node_module = load_modules()

    def setUp(self):
        with patch.object(self.node_module, 'load_sources', return_value=({'head': ('/detections', 'camera')},
                                                                         'tag36h11', {7: .1})):
            self.node = self.node_module.AprilTagRviz()
        self.node.buffer.lookup_transform.return_value = NS(
            header=header(stamp=200), transform=NS(translation=vector(2., 3., 4.),
                                                 rotation=pose().orientation))

    def test_latest_display_mode_preserves_original_detection_stamp(self):
        message = detections(detection())
        self.node.receive('head', 'camera', message)
        when = self.node.buffer.lookup_transform.call_args.args[2]
        self.assertEqual(when.nanoseconds, 0)
        marker = self.node.publisher.publish.call_args.args[0].markers[0]
        self.assertEqual(marker.header.frame_id, 'map')
        self.assertEqual(vars(marker.pose.position), {'x': 2., 'y': 3., 'z': 5.})
        self.assertEqual(message.header.stamp.sec, 123)

    def test_measurement_mode_requests_image_time(self):
        self.node.tf_mode = 'measurement'
        self.node.receive('head', 'camera', detections(detection()))
        self.assertEqual(self.node.buffer.lookup_transform.call_args.args[2].nanoseconds, 123 * 10**9)

    def test_stale_or_missing_tf_and_wrong_frame_remove_old_axes(self):
        self.node.receive('head', 'camera', detections(detection()))
        self.node.buffer.lookup_transform.return_value.header.stamp.sec = 190
        self.node.receive('head', 'camera', detections(detection()))
        markers = self.node.publisher.publish.call_args.args[0].markers
        self.assertEqual(len(markers), 4)
        self.assertTrue(all(m.action == Marker.DELETE for m in markers))
        self.node.buffer.lookup_transform.side_effect = RuntimeError('TF missing')
        self.node.receive('head', 'camera', detections(detection()))
        self.assertEqual(self.node.publisher.publish.call_args.args[0].markers, [])
        self.node.buffer.lookup_transform.reset_mock()
        self.node.receive('head', 'wrong', detections(detection()))
        self.node.buffer.lookup_transform.assert_not_called()

    def test_empty_detection_array_clears_without_tf_lookup(self):
        self.node.receive('head', 'camera', detections(detection()))
        self.node.buffer.lookup_transform.reset_mock()
        self.node.receive('head', 'camera', detections())
        self.node.buffer.lookup_transform.assert_not_called()
        self.assertEqual(len(self.node.publisher.publish.call_args.args[0].markers), 4)


class LaunchTests(unittest.TestCase):
    def setUp(self):
        class Action:
            def __init__(self, *args, **kwargs):
                self.args, self.kwargs = args, kwargs

        class Config:
            def __init__(self, name):
                self.name = name

            def perform(self, context):
                return context[self.name]

        self.missing_error = type('PackageNotFoundError', (Exception,), {})
        modules = {
            'ament_index_python.packages': NS(PackageNotFoundError=self.missing_error,
                                               get_package_share_directory=Mock()),
            'launch.actions': NS(**{name: type(name, (Action,), {}) for name in
                                    ('DeclareLaunchArgument', 'ExecuteProcess', 'LogInfo')}),
            'launch.substitutions': NS(LaunchConfiguration=Config),
            'launch_ros.actions': NS(Node=type('Node', (Action,), {})),
        }
        spec = importlib.util.spec_from_file_location('apriltag_launch_under_test', ROOT / 'common/apriltag_launch.py')
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(self.module)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / 'config').mkdir()
        for file in ('cameras.yaml', 'tags.yaml'):
            (self.root / 'config' / file).write_text('{}')
        self.context = {'apriltags': 'true', 'apriltag_root': str(self.root), 'apriltag_cameras': '',
                        'apriltag_tags': '', 'apriltag_tf_time': 'latest', 'apriltag_axis_length': '0.1',
                        'apriltag_start_detector': 'true', 'use_sim_time': 'false'}

    def test_installed_detector_and_axes_use_same_config_and_private_tf(self):
        self.module.get_package_share_directory.return_value = str(self.root)
        detector, axes = self.module.configure(self.context)
        self.assertEqual(detector.kwargs['package'], 'apriltag_utils')
        parameters = detector.kwargs['parameters'][0]
        self.assertFalse(parameters['publish_tf'])
        cmd = axes.kwargs['cmd']
        self.assertIn(f'cameras_config:={parameters["cameras_config"]}', cmd)
        self.assertIn(f'tags_config:={parameters["tags_config"]}', cmd)
        self.assertIn('/tf:=/astribot_viz/tf', cmd)
        self.assertIn('/tf_static:=/astribot_viz/tf_static', cmd)

    def test_existing_detector_mode_only_starts_visualizer(self):
        self.module.get_package_share_directory.return_value = str(self.root)
        self.context['apriltag_start_detector'] = 'false'
        actions = self.module.configure(self.context)
        self.assertEqual(len(actions), 1)
        self.assertIn('astribot_scripts.apriltag_rviz', actions[0].kwargs['cmd'])

    def test_auto_skips_missing_package_and_explicit_enable_reports_error(self):
        self.module.get_package_share_directory.side_effect = self.missing_error()
        with patch.object(self.module.importlib.util, 'find_spec', return_value=None):
            self.context['apriltags'] = 'auto'
            actions = self.module.configure(self.context)
            self.assertEqual(type(actions[0]).__name__, 'LogInfo')
            self.context['apriltags'] = 'true'
            with self.assertRaises(RuntimeError):
                self.module.configure(self.context)
        self.context['apriltags'] = 'false'
        self.assertEqual(self.module.configure(self.context), [])

    def test_source_fallback_passes_pythonpath_and_detector_parameters(self):
        self.module.get_package_share_directory.side_effect = self.missing_error()
        (self.root / 'apriltag_utils').mkdir()
        (self.root / 'apriltag_utils/node.py').write_text('')
        with patch.object(self.module.importlib.util, 'find_spec', return_value=object()):
            detector, axes = self.module.configure(self.context)
        self.assertTrue(detector.kwargs['additional_env']['PYTHONPATH'].startswith(str(self.root)))
        self.assertIn('from apriltag_utils.node import main; main()', detector.kwargs['cmd'])

    def test_visualization_only_does_not_require_native_detector_dependencies(self):
        self.module.get_package_share_directory.side_effect = self.missing_error()
        self.context['apriltag_start_detector'] = 'false'
        with patch.object(self.module.importlib.util, 'find_spec',
                          side_effect=lambda name: object() if name == 'vision_msgs' else None):
            actions = self.module.configure(self.context)
        self.assertEqual(len(actions), 1)

    def test_aruco_source_does_not_require_pupil_apriltags(self):
        self.module.get_package_share_directory.side_effect = self.missing_error()
        (self.root / 'apriltag_utils').mkdir()
        (self.root / 'apriltag_utils/node.py').write_text('')
        (self.root / 'config/tags.yaml').write_text('family: DICT_4X4_50\n')
        with patch.object(self.module.importlib.util, 'find_spec',
                          side_effect=lambda name: None if name == 'pupil_apriltags' else object()):
            actions = self.module.configure(self.context)
        self.assertEqual(len(actions), 2)

    def test_rviz_enables_axis_display_and_combined_launch_uses_auto_mode(self):
        config = yaml.safe_load((ROOT / 'config/astribot_fast_lio.rviz').read_text())['Visualization Manager']
        group = next(item for item in config['Displays'] if item['Name'] == 'AprilTag detections')
        self.assertTrue(group['Enabled'])
        self.assertEqual(group['Displays'][0]['Marker Topic']['Value'], '/astribot_viz/apriltag_axes')
        self.assertIn('configure_apriltags', (ROOT / 'launch/astribot_lio_rviz.launch.py').read_text())
        arguments = self.module.arguments()
        self.assertEqual(arguments[0].kwargs['default_value'], 'auto')


if __name__ == '__main__':
    unittest.main()
