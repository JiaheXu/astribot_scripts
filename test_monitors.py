"""Offline checks; ROS, the SDK and OpenCV are replaced before importing scripts."""
import builtins
import contextlib
import importlib.util
import io
from pathlib import Path
import runpy
import sys
import unittest
from types import SimpleNamespace
import numpy as np
from unittest.mock import Mock, patch


class MonitorTests(unittest.TestCase):
    def load(self, filename):
        self.robot = Mock()
        self.robot.whole_body_names = ['arm']
        self.robot.arm_names = ['left', 'right']
        for kind in ('current', 'desired'):
            for quantity in ('position', 'velocity', 'torque'):
                getattr(self.robot, f'get_{kind}_joints_{quantity}').return_value = [[1, 2]]
            getattr(self.robot, f'get_{kind}_cartesian_pose').return_value = [[1, 2, 3, 0, 0, 0, 1]] * 2
        self.ros = Mock()
        modules = {name: Mock() for name in (
            'rclpy', 'rclpy.qos', 'sensor_msgs', 'sensor_msgs.msg',
            'cv2', 'numpy', 'tabulate',
            'astribot_sdk.core.astribot_api.astribot_client')}
        modules['rclpy'] = self.ros
        modules['numpy'] = np
        modules['sensor_msgs.msg'].Image.side_effect = SimpleNamespace
        modules['astribot_sdk.core.astribot_api.astribot_client'].Astribot.return_value = self.robot
        modules['tabulate'].tabulate.side_effect = lambda rows, **kwargs: repr(rows)
        self.modules = modules
        path = Path(__file__).parent / filename
        spec = importlib.util.spec_from_file_location('monitor_under_test', path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
        return module

    def run_main(self, module, args):
        with patch.object(sys, 'argv', ['monitor', *args]), \
                patch.object(module.time, 'sleep'), \
                contextlib.redirect_stdout(io.StringIO()) as output:
            module.main()
        return output.getvalue()

    def test_joint_and_pose_monitors_refresh_and_close(self):
        for filename in ('101-get_joint_states.py', '102-get_cartesian_states.py'):
            with self.subTest(filename=filename):
                module = self.load(filename)
                self.assertTrue(callable(getattr(module, 'main', None)), 'needs a continuous monitor entry point')
                self.ros.ok.side_effect = [True, True, False, True]
                output = self.run_main(module, [])
                self.assertIn('1', output)
                if filename.startswith('101'):
                    self.assertEqual(self.robot.get_current_joints_position.call_count, 2)
                    self.assertEqual(self.robot.get_current_joints_torque.call_count, 2)
                else:
                    calls = self.robot.get_current_cartesian_pose.call_args_list
                    self.assertEqual(len(calls), 2)
                    self.assertEqual(calls[0].kwargs, {'names': ['left', 'right'], 'frame': 'chassis'})
                    self.assertEqual(self.robot.get_desired_cartesian_pose.call_args.kwargs, calls[0].kwargs)
                self.ros.shutdown.assert_called_once()

    def test_invalid_refresh_rate_rejected_before_connecting(self):
        for filename in ('101-get_joint_states.py', '102-get_cartesian_states.py', 'get_joints_and_poses.py'):
            module = self.load(filename)
            self.assertTrue(callable(getattr(module, 'main', None)))
            for rate in ('0', '-1', 'nan', 'inf'):
                with self.subTest(filename=filename, rate=rate), contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit) as result:
                        self.run_main(module, ['--rate', rate])
                    self.assertEqual(result.exception.code, 2)
            self.modules['astribot_sdk.core.astribot_api.astribot_client'].Astribot.assert_not_called()

    def test_combined_joints_and_poses_refreshes_both_states_in_selected_frame(self):
        module = self.load('get_joints_and_poses.py')
        self.robot.whole_body_names = ['arm', 'gripper']
        self.robot.effector_names = ['gripper']
        for state in ('current', 'desired'):
            for quantity in ('position', 'velocity', 'torque'):
                getattr(self.robot, f'get_{state}_joints_{quantity}').return_value = [[1, 2], [50]]
            getattr(self.robot, f'get_{state}_cartesian_pose').return_value = [[1, 2, 3, 0, 0, 0, 1]]
        self.ros.ok.side_effect = [True, True, False, True]
        output = self.run_main(module, ['--rate', '5', '--frame', 'world'])
        for state in ('current', 'desired'):
            for quantity in ('position', 'velocity', 'torque'):
                self.assertEqual(getattr(self.robot, f'get_{state}_joints_{quantity}').call_count, 2)
            getter = getattr(self.robot, f'get_{state}_cartesian_pose')
            self.assertEqual(getter.call_count, 2)
            self.assertEqual(getter.call_args.kwargs, {'names': ['arm'], 'frame': 'world'})
        self.assertIn('frame=world', output)
        self.assertIn('gripper', output)
        self.robot.astribot_interface.node.create_publisher.assert_not_called()
        self.ros.shutdown.assert_called_once()

    def test_combined_reader_rejects_missing_part_data_and_closes(self):
        module = self.load('get_joints_and_poses.py')
        self.robot.effector_names = []
        self.robot.get_current_joints_position.return_value = []
        with patch.object(sys, 'argv', ['monitor']), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, 'Incomplete current joint position data'):
                module.main()
        self.ros.shutdown.assert_called_once()

    def test_lidar_leaves_running_driver_alone_by_default(self):
        module = self.load('301-get_lidar_scan.py')
        monitor = module.LidarMonitor()
        self.modules['astribot_sdk.core.astribot_api.astribot_client'].Astribot.assert_called_once_with()
        monitor.start_hardware()
        monitor.lidar_callback('front', Mock(width=5, height=2))
        monitor.close()
        self.assertEqual(monitor.frame_count['front'], 1)
        self.assertEqual(monitor.point_count['front'], 10)
        self.robot.activate_lidar.assert_not_called()
        self.robot.deactivate_lidar.assert_not_called()

    def test_camera_receives_multiple_streams_without_activation_or_gui(self):
        module = self.load('300-get_images.py')
        self.assertTrue(callable(getattr(module, 'main', None)), 'needs a monitor entry point')
        def subscribe(camera, kind, callback, need_decode):
            callback(f'/astribot_camera/{camera}/color_compress/compressed', Mock(), 2, 2, object())
            return camera
        self.robot.register_image_callback.side_effect = subscribe
        self.ros.ok.side_effect = [True, False, True]
        output = self.run_main(module, ['--cameras', 'head_rgbd', 'left_wrist_rgbd', '--headless'])
        self.modules['astribot_sdk.core.astribot_api.astribot_client'].Astribot.assert_called_once_with()
        self.assertIn("'head_rgbd': 1", output)
        self.assertIn("'left_wrist_rgbd': 1", output)
        self.assertEqual(self.robot.register_image_callback.call_count, 2)
        self.assertEqual(self.robot.astribot_interface.node.destroy_subscription.call_count, 2)
        self.robot.activate_camera.assert_not_called()
        self.robot.deactivate_camera.assert_not_called()
        self.modules['cv2'].imshow.assert_not_called()
        self.ros.shutdown.assert_called_once()

    def test_startup_answers_enter_and_restores_input(self):
        for filename in ('300-get_images.py', '301-get_lidar_scan.py', 'get_liadra_and_img.py', 'get_joints_and_poses.py'):
            for fails in (False, True):
                with self.subTest(filename=filename, fails=fails):
                    module = self.load(filename)
                    answers = []
                    def connect():
                        answers.append(input("Control-rights prompt"))
                        if fails:
                            raise RuntimeError("Connection failed")
                        return self.robot
                    self.modules['astribot_sdk.core.astribot_api.astribot_client'].Astribot.side_effect = connect
                    self.ros.ok.side_effect = [False, True]
                    def start():
                        if filename == 'get_joints_and_poses.py':
                            self.run_main(module, [])
                        elif not filename.startswith('301'):
                            self.run_main(module, ['--headless'])
                        else:
                            module.LidarMonitor().close()
                    with patch('builtins.input', side_effect=AssertionError('must not read stdin')) as original_input:
                        if fails:
                            with self.assertRaisesRegex(RuntimeError, 'Connection failed'):
                                start()
                        else:
                            start()
                        self.assertIs(builtins.input, original_input)
                    self.assertEqual(answers, [''])

    def test_combined_stream_publishes_standard_images_and_original_clouds(self):
        module = self.load('get_liadra_and_img.py')
        node = self.robot.astribot_interface.node
        for name in ('imshow', 'waitKey', 'destroyAllWindows'):
            getattr(self.modules['cv2'], name).side_effect = AssertionError('GUI must never be used')
        publishers, clouds, images = {}, {}, {}
        depth = Mock(format='raw', data=bytes(640 * 576 * 2))
        pixels = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
        def publisher(message_type, topic, qos):
            publishers[topic] = Mock()
            return publishers[topic]
        def subscribe_lidar(message_type, topic, callback, qos):
            if topic.endswith('/depth_compress'):
                callback(depth)
            else:
                clouds[topic] = Mock(width=5, height=2)
                callback(clouds[topic])
            return topic
        def subscribe_image(camera, kind, callback, need_decode):
            self.assertTrue(need_decode, 'headless output still needs decoded pixels')
            images[camera] = Mock()
            callback(f'/astribot_camera/{camera}/color_compress/compressed', images[camera], 2, 2, pixels)
            return camera
        node.create_publisher.side_effect = publisher
        node.create_subscription.side_effect = subscribe_lidar
        self.robot.register_image_callback.side_effect = subscribe_image
        self.ros.ok.side_effect = [True, False, True]
        output = self.run_main(module, ['--cameras', 'head_rgbd', 'left_wrist_rgbd'])
        self.modules['astribot_sdk.core.astribot_api.astribot_client'].Astribot.assert_called_once_with()
        self.robot.activate_camera.assert_called_once_with()
        self.robot.activate_lidar.assert_called_once_with()
        self.assertEqual([call.args[1] for call in node.create_subscription.call_args_list],
                         ['/livox/lidar_front', '/livox/lidar_back', '/astribot_camera/head_rgbd/depth_compress'])
        self.assertEqual(set(publishers), {
            '/sdk/astribot_camera/head_rgbd/color_image',
            '/sdk/astribot_camera/left_wrist_rgbd/color_image',
            '/sdk/astribot_lidar/front_scan', '/sdk/astribot_lidar/back_scan',
            '/sdk/astribot_camera/head_rgbd/depth_image'})
        for camera, source in images.items():
            published = publishers[f'/sdk/astribot_camera/{camera}/color_image'].publish.call_args.args[0]
            self.assertIs(published.header, source.header)
            self.assertEqual((published.height, published.width, published.step), (2, 2, 6))
            self.assertEqual(published.encoding, 'bgr8')
            self.assertEqual(published.data, pixels.tobytes())
        published_depth = publishers['/sdk/astribot_camera/head_rgbd/depth_image'].publish.call_args.args[0]
        self.assertIs(published_depth.header, depth.header)
        self.assertEqual(published_depth.encoding, '16UC1')
        self.assertEqual((published_depth.width, published_depth.height), (640, 576))
        self.assertEqual(bytes(published_depth.data), depth.data)
        for name in ('front', 'back'):
            published = publishers[f'/sdk/astribot_lidar/{name}_scan'].publish.call_args.args[0]
            self.assertIs(published, clouds[f'/livox/lidar_{name}'])
        self.assertIn("'head_rgbd': 1", output)
        self.assertIn('front: 1 scans, 10 points', output)
        self.assertIn('back: 1 scans, 10 points', output)
        self.assertEqual(node.destroy_subscription.call_count, 5)
        self.assertEqual(node.destroy_publisher.call_count, 5)
        self.robot.deactivate_camera.assert_not_called()
        self.robot.deactivate_lidar.assert_not_called()
        self.ros.shutdown.assert_called_once()

    def test_combined_stream_reports_activation_failure(self):
        for driver in ('camera', 'lidar'):
            with self.subTest(driver=driver):
                module = self.load('get_liadra_and_img.py')
                getattr(self.robot, f'activate_{driver}').return_value = False
                with self.assertRaisesRegex(RuntimeError, 'activation failed'):
                    self.run_main(module, ['--headless'])
                self.robot.register_image_callback.assert_not_called()
                self.robot.astribot_interface.node.create_subscription.assert_not_called()
                self.ros.shutdown.assert_called_once()

    def test_camera_displays_latest_frame_and_quits_with_q(self):
        module = self.load('300-get_images.py')
        first, latest = object(), object()
        def subscribe(camera, kind, callback, need_decode):
            self.assertTrue(need_decode)
            for frame in (first, latest):
                callback(f'/astribot_camera/{camera}/color_compress/compressed', Mock(), 2, 2, frame)
            return camera
        self.robot.register_image_callback.side_effect = subscribe
        self.modules['cv2'].waitKey.return_value = ord('q')
        self.run_main(module, [])
        self.modules['cv2'].imshow.assert_called_once_with('head_rgbd', latest)
        self.modules['cv2'].destroyAllWindows.assert_called_once()
        self.robot.astribot_interface.node.destroy_subscription.assert_called_once_with('head_rgbd')

    def test_camera_activation_failure_stops_before_subscribing(self):
        module = self.load('300-get_images.py')
        self.robot.activate_camera.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'Camera activation failed'):
            self.run_main(module, ['--activate', '--headless'])
        self.robot.register_image_callback.assert_not_called()
        self.ros.shutdown.assert_called_once()

    def test_joint_monitor_cleans_up_on_keyboard_interrupt(self):
        module = self.load('101-get_joint_states.py')
        self.robot.get_current_joints_position.side_effect = KeyboardInterrupt
        self.run_main(module, [])
        self.ros.shutdown.assert_called_once()

    def test_property_limits_are_labeled_in_sdk_return_order(self):
        # Reversing the SDK's (lower, upper) tuple must fail this check.
        self.load('101-get_joint_states.py')
        self.robot.get_dof.return_value = [2]
        self.robot.get_joints_position_limit.return_value = ([[-2, -3]], [[2, 3]])
        self.robot.get_joints_velocity_limit.return_value = [[4, 4]]
        self.robot.get_joints_torque_limit.return_value = [[5, 5]]
        with patch.dict(sys.modules, self.modules), contextlib.redirect_stdout(io.StringIO()) as output:
            runpy.run_path(str(Path(__file__).parent / '100-get_robot_properties.py'), run_name='__main__')
        self.assertIn("['arm', [2, 3], [-2, -3], [4, 4], [5, 5]]", output.getvalue())


if __name__ == '__main__':
    unittest.main()
