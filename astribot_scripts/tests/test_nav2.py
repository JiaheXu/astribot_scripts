"""Offline Nav2 geometry, freshness, ROS adapter and parameter contract tests."""
from copy import deepcopy
from concurrent.futures import Future
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from astribot_scripts.common.nav2 import (BASE_FRAME, IMU_FRAME, ODOM_FRAME,
                                         FeedbackGate, OdometryProjector, merge_parameters,
                                         pose_matrix, rigid_matrix)

ROOT = Path(__file__).resolve().parents[1]
NS = types.SimpleNamespace


def pose_values(matrix):
    return matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_quat()


class GeometryTests(unittest.TestCase):
    def test_chassis_lever_arm_is_applied_before_differentiation(self):
        base_from_imu = pose_matrix([-.2, -.18, .21], Rotation.from_euler('z', 2.35).as_quat())
        projector = OdometryProjector(np.linalg.inv(base_from_imu))
        for index, yaw in enumerate((0.6, 0.61, 0.62)):
            world_from_base = pose_matrix([3, -2, .4], Rotation.from_euler('z', yaw).as_quat())
            pose, velocity = projector.project(*pose_values(world_from_base @ base_from_imu),
                                               int((100 + index * .1) * 1e9))
            np.testing.assert_allclose(pose[:3, 3], [0, 0, 0], atol=1e-12)
            self.assertAlmostEqual(Rotation.from_matrix(pose[:3, :3]).as_euler('xyz')[2], yaw - .6)
            if index:
                np.testing.assert_allclose(velocity[:3], 0, atol=1e-12)
                self.assertAlmostEqual(velocity[5], .1, places=6)

    def test_velocity_is_in_child_frame_and_yaw_wrap_is_continuous(self):
        projector = OdometryProjector(np.eye(4))
        yaw1, yaw2 = np.pi - .01, -np.pi + .01
        projector.project([0, 0, 0], Rotation.from_euler('z', yaw1).as_quat(), 100_000_000_000)
        _, velocity = projector.project([-.01, 0, 0], Rotation.from_euler('z', yaw2).as_quat(),
                                         100_100_000_000)
        self.assertAlmostEqual(velocity[5], .2)
        self.assertGreater(velocity[0], .09)

    def test_bad_pose_and_reflected_mount_are_rejected(self):
        for value in (np.diag([-1, 1, 1, 1]), np.zeros((4, 4)), [[1]]):
            with self.assertRaises(ValueError):
                rigid_matrix(value)
        for position, q in (([0, 0, np.nan], [0, 0, 0, 1]), ([0, 0, 0], [0, 0, 0, 0])):
            with self.assertRaises(ValueError):
                pose_matrix(position, q)

    def test_clock_reset_and_implausible_jump_latch_failure(self):
        for stamp, position in ((99_000_000_000, [0, 0, 0]), (100_100_000_000, [3, 0, 0]),
                                (102_000_000_000, [0, 0, 0])):
            projector = OdometryProjector(np.eye(4))
            projector.project([0, 0, 0], [0, 0, 0, 1], 100_000_000_000)
            with self.assertRaises(ValueError):
                projector.project(position, [0, 0, 0, 1], stamp)
            with self.assertRaises(ValueError):
                projector.project([0, 0, 0], [0, 0, 0, 1], 100_200_000_000)


class GateTests(unittest.TestCase):
    def test_needs_both_streams_and_real_measurement_freshness(self):
        gate = FeedbackGate()
        self.assertIn('odom', gate.reason(100, 10))
        gate.mark('odom', 100, 10)
        self.assertIn('cloud', gate.reason(100, 10))
        gate.mark('cloud', 100, 10)
        self.assertIsNone(gate.reason(100.1, 10.1))
        self.assertIn('timed out', gate.reason(100.6, 10.6))
        gate.mark('odom', 73, 10.6)
        self.assertIn('stamp age', gate.reason(100.6, 10.6))

    def test_future_skew_and_paused_clock_do_not_bypass_gate(self):
        gate = FeedbackGate()
        gate.mark('odom', 100.2, 10)
        gate.mark('cloud', 100, 10)
        self.assertIn('stamp age', gate.reason(100, 10))
        gate.mark('odom', 100, 10)
        gate.mark('cloud', 99.7, 10)
        self.assertIn('disagree', gate.reason(100, 10))
        gate.mark('cloud', 100, 10)
        self.assertIn('timed out', gate.reason(100, 10.6))


class TimeMessage:
    def __init__(self, sec=0, nanosec=0):
        self.sec, self.nanosec = sec, nanosec


def vector():
    return NS(x=0.0, y=0.0, z=0.0)


class TwistMessage:
    def __init__(self):
        self.linear, self.angular = vector(), vector()


class OdometryMessage:
    def __init__(self):
        self.header = NS(stamp=TimeMessage(), frame_id='map')
        self.child_frame_id = 'body'
        self.pose = NS(pose=NS(position=vector(), orientation=NS(x=0., y=0., z=0., w=1.)),
                       covariance=[0.] * 36)
        self.twist = NS(twist=TwistMessage(), covariance=[0.] * 36)


class TransformMessage:
    def __init__(self):
        self.header = NS(stamp=TimeMessage(), frame_id='')
        self.child_frame_id = ''
        self.transform = NS(translation=vector(), rotation=NS(x=0., y=0., z=0., w=1.))


class PoseStampedMessage:
    def __init__(self):
        self.header = NS(stamp=TimeMessage(), frame_id='map')
        self.pose = NS(position=vector(), orientation=NS(x=0., y=0., z=0., w=1.))


class MarkerMessage:
    ARROW, TEXT_VIEW_FACING, ADD = 0, 9, 0

    def __init__(self):
        self.header = NS(stamp=TimeMessage(), frame_id='')
        self.pose = PoseStampedMessage().pose
        self.scale = vector()
        self.color = NS(r=0., g=0., b=0., a=0.)


class FakeNode:
    def __init__(self, name):
        self.now_ns = 100_000_000_000
        self.publishers = {}
        self.logger = Mock()

    def declare_parameter(self, name, value):
        return NS(value=value)

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=self.now_ns, to_msg=lambda: TimeMessage(self.now_ns // 10**9)))

    def create_publisher(self, kind, topic, qos):
        self.publishers[topic] = Mock()
        return self.publishers[topic]

    def create_subscription(self, *args):
        return args

    def create_timer(self, *args):
        return args

    def create_service(self, *args):
        return args

    def get_logger(self):
        return self.logger


class BridgeFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        modules = {
            'rclpy': Mock(), 'rclpy.node': NS(Node=FakeNode),
            'rclpy.executors': NS(ExternalShutdownException=RuntimeError),
            'rclpy.qos': NS(qos_profile_sensor_data='sensor', QoSProfile=lambda **kwargs: kwargs,
                            ReliabilityPolicy=NS(BEST_EFFORT='best'), DurabilityPolicy=NS(VOLATILE='volatile')),
            'builtin_interfaces.msg': NS(Time=TimeMessage),
            'geometry_msgs.msg': NS(TransformStamped=TransformMessage, Twist=TwistMessage),
            'nav_msgs.msg': NS(Odometry=OdometryMessage, Path=lambda: NS(header=NS(frame_id=''), poses=[])),
            'sensor_msgs.msg': NS(PointCloud2=object), 'std_msgs.msg': NS(String=lambda **kw: NS(**kw)),
            'tf2_ros': NS(StaticTransformBroadcaster=Mock(), TransformBroadcaster=Mock()),
            'rclpy.action': NS(ActionClient=lambda *args: Mock()),
            'nav2_msgs.action': NS(NavigateToPose=NS(Goal=lambda: NS(pose=PoseStampedMessage()))),
            'std_srvs.srv': NS(Trigger=object),
            'visualization_msgs.msg': NS(Marker=MarkerMessage, MarkerArray=lambda: NS(markers=[])),
        }
        modules['geometry_msgs.msg'].PoseStamped = PoseStampedMessage
        modules['geometry_msgs.msg'].PointStamped = object
        modules['rclpy.qos'].DurabilityPolicy.TRANSIENT_LOCAL = 'transient_local'
        spec = importlib.util.spec_from_file_location('nav2_bridge_under_test', ROOT / 'nav2_bridge.py')
        cls.module = importlib.util.module_from_spec(spec)
        cls.modules = modules
        with patch.dict(sys.modules, modules):
            goal_spec = importlib.util.spec_from_file_location(
                'astribot_scripts.common.nav2_goals', ROOT / 'common/nav2_goals.py')
            goal_module = importlib.util.module_from_spec(goal_spec)
            goal_spec.loader.exec_module(goal_module)
            sys.modules[goal_spec.name] = goal_module
            spec.loader.exec_module(cls.module)

    def setUp(self):
        with patch.dict(sys.modules, self.modules), \
                patch.object(self.module, 'load_mounts', return_value=(np.eye(4), np.eye(4))):
            self.node = self.module.Nav2Bridge()
        self.node.enable_output = True
        self.clock = patch.object(self.module.time, 'monotonic', return_value=10.0)
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def odom(self, stamp=100.0, x=0.0):
        message = OdometryMessage()
        message.header.stamp = self.module.stamp_message(round(stamp * 1e9))
        message.pose.pose.position.x = x
        return message

    def cloud(self, stamp=100.1):
        return NS(header=NS(frame_id='body', stamp=self.module.stamp_message(round(stamp * 1e9))),
                  fields=[NS(name=key) for key in ('x', 'y', 'z')], width=1, height=1,
                  point_step=12, row_step=12, data=bytes(12))

    def ready(self):
        self.node.receive_odometry(self.odom())
        self.node.now_ns = 100_100_000_000
        self.node.receive_odometry(self.odom(100.1, .01))
        self.node.receive_cloud(self.cloud())
        self.assertIsNone(self.node.feedback_reason())


class BridgeTests(BridgeFixture):
    def test_publishes_measured_velocity_and_preserves_stamps_and_source(self):
        self.ready()
        output = self.node.odom_pub.publish.call_args.args[0]
        self.assertEqual((output.header.frame_id, output.child_frame_id), (ODOM_FRAME, BASE_FRAME))
        self.assertEqual(self.module.stamp_ns(output.header.stamp), 100_100_000_000)
        self.assertAlmostEqual(output.twist.twist.linear.x, .1)
        self.assertEqual(output.twist.covariance[0], 1e6)
        cloud = self.cloud(100.2)
        self.node.now_ns = 100_200_000_000
        self.node.receive_cloud(cloud)
        forwarded = self.node.cloud_pub.publish.call_args.args[0]
        self.assertEqual(forwarded.header.frame_id, IMU_FRAME)
        self.assertEqual(cloud.header.frame_id, 'body')
        self.assertEqual(forwarded.data, cloud.data)

    def test_stale_feedback_and_command_timeout_publish_zero(self):
        self.ready()
        command = TwistMessage()
        command.linear.x = .1
        self.node.receive_command(command)
        self.node.publish_command()
        self.assertEqual(self.node.cmd_pub.publish.call_args.args[0].linear.x, .1)
        with patch.object(self.module.time, 'monotonic', return_value=10.31):
            self.node.publish_command()
        self.assertEqual(self.node.cmd_pub.publish.call_args.args[0].linear.x, 0)
        self.node.receive_command(command)
        self.node.now_ns = 101_000_000_000
        self.node.publish_command()
        self.assertIsNone(self.node.latest_command)
        self.assertEqual(self.node.cmd_pub.publish.call_args.args[0].linear.x, 0)

    def test_invalid_cloud_and_twist_cancel_previous_command(self):
        self.ready()
        command = TwistMessage()
        command.linear.x = .1
        self.node.receive_command(command)
        command.angular.z = float('nan')
        self.node.receive_command(command)
        self.assertIsNone(self.node.latest_command)
        malformed = self.cloud(100.2)
        malformed.header.frame_id = 'livox_frame'
        self.node.receive_cloud(malformed)
        self.assertIn('cloud', self.node.feedback_reason())

    def test_preview_cannot_forward_a_nonzero_command(self):
        self.ready()
        self.node.enable_output = False
        command = TwistMessage()
        command.linear.x = .1
        self.node.receive_command(command)
        self.node.publish_command()
        self.node.cmd_pub.publish.assert_not_called()

    def test_known_clock_discrepancy_is_not_silently_retimed(self):
        source = self.odom(73)
        self.node.receive_odometry(source)
        self.node.odom_pub.publish.assert_not_called()
        self.node.offset = 27.0
        self.node.receive_odometry(source)
        output = self.node.odom_pub.publish.call_args.args[0]
        self.assertEqual(output.header.stamp.sec, 100)
        self.assertEqual(source.header.stamp.sec, 73)

    def test_nominal_mounts_match_current_stack(self):
        base, lidar = self.module.load_mounts()
        np.testing.assert_allclose(base[:3, 3], [-.203837491, -.183281143, .210277], atol=1e-9)
        np.testing.assert_allclose(lidar[:3, 3], [-.011, -.02329, .04412])


class RvizGoalTests(BridgeFixture):
    def setUp(self):
        super().setUp()
        self.goals = self.node.rviz_goals
        self.goals.client.server_is_ready.return_value = True
        self.response = Future()
        self.goals.client.send_goal_async.return_value = self.response

    def map_origin(self):
        self.ready()
        world_from_odom = pose_matrix([3., 2., .2], Rotation.from_euler('z', np.pi / 2).as_quat())
        self.node.projector.odom_from_world = np.linalg.inv(world_from_odom)

    def goal(self, x=3.0, y=3.0):
        goal = PoseStampedMessage()
        goal.pose.position.x, goal.pose.position.y = x, y
        goal.pose.orientation.z = np.sin(np.pi / 4)
        goal.pose.orientation.w = np.cos(np.pi / 4)
        return goal

    def accept(self, identifier=1):
        handle = Mock(accepted=True, goal_id=NS(uuid=bytes([identifier] * 16)))
        handle.result_future = Future()
        handle.get_result_async.return_value = handle.result_future
        cancel_response = Future()
        cancel_response.set_result(NS(goals_canceling=[True]))
        handle.cancel_goal_async.return_value = cancel_response
        self.response.set_result(handle)
        return handle

    def test_click_is_transformed_from_map_to_nav_origin_with_heading(self):
        self.map_origin()
        source = self.goal()
        self.goals.receive_pose(source)
        request = self.goals.client.send_goal_async.call_args.args[0]
        self.assertEqual(request.pose.header.frame_id, ODOM_FRAME)
        self.assertAlmostEqual(request.pose.pose.position.x, 1.0)
        self.assertAlmostEqual(request.pose.pose.position.y, 0.0)
        self.assertEqual(request.pose.pose.position.z, 0.0)
        self.assertAlmostEqual(request.pose.pose.orientation.w, 1.0)
        self.assertEqual(source.header.frame_id, 'map')
        self.assertEqual(source.pose.position.x, 3.0)
        handle = self.accept()
        self.assertIs(self.goals.active, handle)
        markers = self.goals.markers.publish.call_args.args[0].markers
        self.assertEqual(markers[0].header.frame_id, 'map')
        self.assertAlmostEqual(markers[0].pose.position.x, 3.0)
        self.assertAlmostEqual(markers[0].pose.position.y, 3.0)
        self.assertIn('Navigating', markers[1].text)

    def test_point_uses_xy_and_current_heading_ignoring_surface_height(self):
        self.map_origin()
        point = NS(header=NS(frame_id='map', stamp=TimeMessage()), point=NS(x=3., y=3., z=8.))
        self.goals.receive_point(point)
        request = self.goals.client.send_goal_async.call_args.args[0]
        self.assertAlmostEqual(request.pose.pose.position.x, 1.0)
        self.assertEqual(request.pose.pose.position.z, 0.0)
        self.assertAlmostEqual(request.pose.pose.orientation.w, 1.0)

    def test_invalid_or_unready_clicks_are_not_queued(self):
        self.goals.receive_pose(self.goal())
        self.goals.client.send_goal_async.assert_not_called()
        self.ready()
        for value in ('wrong_frame', 'nan', 'quaternion', 'unavailable'):
            goal = self.goal()
            if value == 'wrong_frame':
                goal.header.frame_id = ODOM_FRAME
            elif value == 'nan':
                goal.pose.position.x = float('nan')
            elif value == 'quaternion':
                goal.pose.orientation.z = goal.pose.orientation.w = 0.0
            else:
                self.goals.client.server_is_ready.return_value = False
            self.goals.receive_pose(goal)
        self.goals.client.send_goal_async.assert_not_called()

    def test_pending_request_prevents_double_click_submission(self):
        self.ready()
        self.goals.receive_pose(self.goal())
        self.goals.receive_pose(self.goal(2.0, 2.0))
        self.goals.client.send_goal_async.assert_called_once()

    def test_replacement_ignores_previous_goals_result_and_feedback(self):
        self.ready()
        self.goals.receive_pose(self.goal())
        previous = self.accept()
        self.response = Future()
        self.goals.client.send_goal_async.return_value = self.response
        self.goals.receive_pose(self.goal(2.0, 2.0))
        current = self.accept(2)
        previous.result_future.set_result(NS(status=6))
        self.assertIs(self.goals.active, current)
        self.goals.status.publish.reset_mock()
        self.goals.feedback(NS(goal_id=previous.goal_id, feedback=NS(distance_remaining=1.0)))
        self.goals.status.publish.assert_not_called()
        self.goals.feedback(NS(goal_id=current.goal_id, feedback=NS(distance_remaining=.5)))
        self.assertIn('0.50 m', self.goals.status.publish.call_args.args[0].data)
        current.result_future.set_result(NS(status=4))
        self.assertIsNone(self.goals.active)
        self.assertEqual(self.goals.status.publish.call_args.args[0].data, 'Target reached')

    def test_rejected_replacement_restores_previous_active_target_marker(self):
        self.ready()
        self.goals.receive_pose(self.goal())
        handle = self.accept()
        previous = deepcopy(self.goals.target)
        self.response = Future()
        self.goals.client.send_goal_async.return_value = self.response
        self.goals.receive_pose(self.goal(2.0, 2.0))
        self.response.set_result(NS(accepted=False))
        self.assertIs(self.goals.active, handle)
        self.assertEqual(self.goals.target.pose.position.x, previous.pose.position.x)
        self.assertIn('previous goal remains active', self.goals.status.publish.call_args.args[0].data)

    def test_preview_target_is_labeled_as_motion_disabled(self):
        self.ready()
        self.node.enable_output = False
        self.goals.receive_pose(self.goal())
        self.accept()
        self.assertIn('Preview (motion disabled)', self.goals.status.publish.call_args.args[0].data)

    def test_cancel_pending_goal_and_stale_feedback_cancel_on_accept(self):
        self.ready()
        self.goals.receive_pose(self.goal())
        response = self.goals.cancel(None, NS())
        self.assertTrue(response.success)
        handle = self.accept()
        handle.cancel_goal_async.assert_called_once()

    def test_accepted_goal_is_canceled_if_feedback_went_stale(self):
        self.ready()
        self.goals.receive_pose(self.goal())
        self.node.now_ns = 102_000_000_000
        self.accept().cancel_goal_async.assert_called_once()

    def test_path_is_converted_for_existing_map_rviz_without_touching_source(self):
        self.map_origin()
        item = PoseStampedMessage()
        item.header.frame_id = ODOM_FRAME
        item.pose.position.x = 1.0
        source = NS(header=NS(frame_id=ODOM_FRAME), poses=[item])
        self.goals.receive_path(source)
        result = self.goals.path.publish.call_args.args[0]
        self.assertEqual(result.header.frame_id, 'map')
        self.assertEqual(result.poses[0].header.frame_id, 'map')
        self.assertAlmostEqual(result.poses[0].pose.position.x, 3.0)
        self.assertAlmostEqual(result.poses[0].pose.position.y, 3.0)
        self.assertEqual(source.poses[0].pose.position.x, 1.0)

    def test_rviz_tools_use_bridge_topics_and_map_feedback_displays(self):
        config = yaml.safe_load((ROOT / 'config/astribot_fast_lio.rviz').read_text())['Visualization Manager']
        tools = {item['Class']: item for item in config['Tools']}
        self.assertEqual(tools['rviz_default_plugins/SetGoal']['Topic'], '/astribot_nav/goal_pose_map')
        self.assertEqual(tools['rviz_default_plugins/PublishPoint']['Topic'], '/astribot_nav/goal_point_map')
        self.assertEqual(config['Global Options']['Fixed Frame'], 'map')
        group = next(item for item in config['Displays'] if item['Name'] == 'Click to navigate')
        self.assertEqual(group['Displays'][0]['Marker Topic']['Value'], '/astribot_nav/goal_markers')
        self.assertEqual(group['Displays'][1]['Topic']['Value'], '/astribot_nav/plan_map')


class ConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT / 'config/nav2_humble.yaml').read_text())

    def test_omni_controller_and_smoother_have_nonzero_lateral_limits(self):
        controller = self.config['controller_server']['ros__parameters']
        dwb = controller['FollowPath']
        smoother = self.config['velocity_smoother']['ros__parameters']
        self.assertLess(dwb['min_vel_y'], 0)
        self.assertGreater(dwb['max_vel_y'], 0)
        self.assertGreater(dwb['acc_lim_y'], 0)
        self.assertLess(dwb['decel_lim_y'], 0)
        self.assertLess(controller['min_y_velocity_threshold'], .01)
        self.assertGreater(smoother['max_velocity'][1], 0)
        self.assertLessEqual(dwb['max_speed_xy'], .2)
        self.assertLessEqual(dwb['max_vel_theta'], .4)

    def test_costmaps_use_lio_cloud_mount_and_no_static_map(self):
        for name in ('local_costmap', 'global_costmap'):
            params = self.config[name][name]['ros__parameters']
            self.assertEqual(params['global_frame'], ODOM_FRAME)
            self.assertEqual(params['robot_base_frame'], BASE_FRAME)
            self.assertTrue(params['rolling_window'])
            self.assertNotIn('static_layer', params['plugins'])
            sensor = params['obstacle_layer']['rear_points']
            self.assertEqual(sensor['topic'], '/astribot_nav/points')
            self.assertEqual(sensor['sensor_frame'], 'astribot_nav_rear_lidar')
            self.assertEqual(sensor['data_type'], 'PointCloud2')

    def test_merge_preserves_installed_bt_plugins_and_removes_sample_scan(self):
        defaults = deepcopy(self.config)
        defaults['bt_navigator']['ros__parameters']['plugin_lib_names'] = ['installed_bt_plugin']
        for name in ('local_costmap', 'global_costmap'):
            params = defaults[name][name]['ros__parameters']
            params.update(static_layer={'plugin': 'map'}, voxel_layer={'plugin': 'voxel'})
            params['obstacle_layer']['scan'] = {'topic': '/scan'}
        merged = merge_parameters(defaults, self.config)
        self.assertEqual(merged['bt_navigator']['ros__parameters']['plugin_lib_names'], ['installed_bt_plugin'])
        for name in ('local_costmap', 'global_costmap'):
            params = merged[name][name]['ros__parameters']
            self.assertNotIn('static_layer', params)
            self.assertNotIn('voxel_layer', params)
            self.assertNotIn('scan', params['obstacle_layer'])
            self.assertIn('scan', defaults[name][name]['ros__parameters']['obstacle_layer'])


class LaunchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        class Action:
            def __init__(self, *args, **kwargs):
                self.args, self.kwargs = args, kwargs

        class Configuration:
            def __init__(self, name):
                self.name = name

            def perform(self, context):
                return context[self.name]

        def actions(*names):
            return NS(**{name: type(name, (Action,), {}) for name in names})

        modules = {
            'ament_index_python.packages': NS(get_package_share_directory=Mock()),
            'launch': actions('LaunchDescription'),
            'launch.actions': actions('DeclareLaunchArgument', 'EmitEvent', 'ExecuteProcess', 'GroupAction',
                                       'IncludeLaunchDescription', 'OpaqueFunction', 'RegisterEventHandler'),
            'launch.event_handlers': actions('OnProcessExit', 'OnShutdown'),
            'launch.events': actions('Shutdown'),
            'launch.launch_description_sources': actions('PythonLaunchDescriptionSource'),
            'launch.substitutions': NS(LaunchConfiguration=Configuration),
            'launch_ros.actions': actions('PushRosNamespace'),
        }
        spec = importlib.util.spec_from_file_location('nav2_launch_under_test', ROOT / 'launch/astribot_nav2.launch.py')
        cls.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(cls.module)

    def context(self, **updates):
        result = {'enable_control': 'true', 'use_sim_time': 'false', 'with_joints': 'false',
                  'stamp_offset': '0.0', 'mount_file': '', 'params_file': str(ROOT / 'config/nav2_humble.yaml')}
        result.update(updates)
        return result

    def configure(self, context):
        with tempfile.TemporaryDirectory() as directory:
            share = Path(directory)
            (share / 'params').mkdir()
            defaults = yaml.safe_load((ROOT / 'config/nav2_humble.yaml').read_text())
            defaults['bt_navigator']['ros__parameters']['plugin_lib_names'] = ['installed_plugin']
            (share / 'params/nav2_params.yaml').write_text(yaml.safe_dump(defaults))
            with patch.dict(os.environ, {'ROS_DISTRO': 'humble'}), \
                    patch.object(self.module, 'get_package_share_directory', return_value=str(share)):
                actions = self.module.configure(context)
        cleanup = actions[0].args[0].kwargs['on_shutdown'][0].kwargs['function']
        self.addCleanup(cleanup, None)
        return actions

    def test_combined_launch_has_one_owner_and_isolated_navigation_namespace(self):
        actions = self.configure(self.context(with_joints='true'))
        processes = [item for item in actions if type(item).__name__ == 'ExecuteProcess']
        self.assertEqual(len(processes), 2)
        bridge, receiver = [item.kwargs['cmd'] for item in processes]
        self.assertIn('/tf:=/astribot_nav/tf', bridge)
        self.assertNotIn('mount_file:=', bridge)
        self.assertIn('enable_output:=true', bridge)
        self.assertIn('astribot_scripts.control_mobile_base', receiver)
        self.assertIn('/astribot/nav_cmd_vel', receiver)
        self.assertIn('--with-joints', receiver)
        group = next(item for item in actions if type(item).__name__ == 'GroupAction')
        self.assertEqual(group.args[0][0].args, ('astribot_nav',))
        launch_args = dict(group.args[0][1].kwargs['launch_arguments'])
        self.assertEqual(launch_args['namespace'], 'astribot_nav')
        params = yaml.safe_load(Path(launch_args['params_file']).read_text())
        self.assertEqual(params['bt_navigator']['ros__parameters']['plugin_lib_names'], ['installed_plugin'])

    def test_preview_launch_has_no_sdk_process_and_disables_output(self):
        actions = self.configure(self.context(enable_control='false', use_sim_time='true'))
        processes = [item for item in actions if type(item).__name__ == 'ExecuteProcess']
        self.assertEqual(len(processes), 1)
        self.assertIn('enable_output:=false', processes[0].kwargs['cmd'])
        self.assertIn('use_sim_time:=true', processes[0].kwargs['cmd'])

    def test_simulation_clock_cannot_start_physical_sdk_control(self):
        with patch.dict(os.environ, {'ROS_DISTRO': 'humble'}), self.assertRaisesRegex(RuntimeError, 'enable_control'):
            self.module.configure(self.context(use_sim_time='true'))


if __name__ == '__main__':
    unittest.main()
