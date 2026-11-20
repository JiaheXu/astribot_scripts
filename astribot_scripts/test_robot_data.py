"""Offline combined-publisher checks with real ROS messages and a mocked SDK."""
import builtins
import contextlib
import io
import json
from pathlib import Path
import os
import re
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

from builtin_interfaces.msg import Time
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu, JointState

import publish_robot_data as data
from get_lidar_and_img import SensorPublisher
from rviz_state import FRAME, MODEL_JOINTS, POSE_PARTS


def robot_fixture():
    robot = Mock(robot_type='S1')
    robot.whole_body_names = ['astribot_chassis', 'astribot_torso', 'astribot_arm_left',
                             'astribot_gripper_left', 'astribot_arm_right',
                             'astribot_gripper_right', 'astribot_head']
    robot.whole_body_dofs = [3, 4, 7, 1, 7, 1, 2]
    for state in ('current', 'desired'):
        for quantity, value in [('position', 0.1), ('velocity', 0.2), ('torque', 0.3)]:
            rows = [[value] * dof for dof in robot.whole_body_dofs]
            rows[3] = [50.0]
            getattr(robot, f'get_{state}_joints_{quantity}').return_value = rows
        getattr(robot, f'get_{state}_cartesian_pose').side_effect = (
            lambda names, frame: [[1., 2., 3., 0., 0., 0., 2.]] * len(names))
    robot.astribot_interface.node.get_clock().now().to_msg.return_value = Time(sec=42, nanosec=123)
    return robot


class RobotDataTests(unittest.TestCase):
    def test_full_snapshot_joint_mapping_and_numeric_poses(self):
        messages = data.read_state(robot_fixture(), 1.0)
        joint = deserialize_message(serialize_message(messages['/astribot_viz/joint_states']), JointState)
        self.assertEqual(joint.name, [name for names in MODEL_JOINTS.values() for name in names])
        self.assertEqual(list(joint.position), [0.1] * 20)
        self.assertEqual(list(joint.velocity), [0.2] * 20)
        self.assertEqual(list(joint.effort), [0.3] * 20)
        self.assertEqual(joint.header.stamp.sec, 42)
        snapshot = json.loads(messages['/sdk/robot_state'].data)
        self.assertEqual(snapshot['states']['current']['position']['astribot_gripper_left'], [50.0])
        self.assertEqual(sum(map(len, snapshot['states']['desired']['position'].values())), 25)
        self.assertEqual(snapshot['poll_started']['sec'], 42)
        self.assertEqual(snapshot['timestamp_basis'], 'sdk_polling_time')
        for state in ('current', 'desired'):
            for part in POSE_PARTS:
                pose = messages[f'/sdk/cartesian/{state}/{part}']
                self.assertEqual(pose.header.frame_id, FRAME)
                self.assertEqual(pose.pose.orientation.w, 1.)
                self.assertEqual(pose.pose.position.x, 1.)
                self.assertEqual(deserialize_message(serialize_message(pose), type(pose)), pose)
            self.assertEqual(messages[f'/sdk/chassis/{state}_pose'].header.frame_id, 'astribot_sdk_world')

    def test_invalid_values_or_missing_parts_are_rejected(self):
        for rows in ([], [[float('nan')]], None):
            robot = robot_fixture()
            robot.get_current_joints_velocity.return_value = rows
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                data.read_state(robot, 1.0)
        robot = robot_fixture()
        robot.get_desired_cartesian_pose.side_effect = lambda names, frame: [[0.] * 7] * len(names)
        with self.assertRaises(ValueError):
            data.read_state(robot, 1.0)

    def test_all_three_imus_relay_complete_messages_and_close(self):
        robot = robot_fixture()
        node = robot.astribot_interface.node
        publishers, callbacks = {}, {}
        node.create_publisher.side_effect = lambda kind, topic, qos: publishers.setdefault(topic, Mock())
        node.create_subscription.side_effect = lambda kind, topic, callback, qos: callbacks.setdefault(topic, callback)
        sensors = SensorPublisher(robot, [], imu_topics=data.IMU_TOPICS)
        sensors.start()
        self.assertEqual(len(data.IMU_TOPICS), 3)
        self.assertIn('/astribot_whole_body/imu_sensor/astribot_chassis_imu_site', callbacks)
        for name, (source, output) in data.IMU_TOPICS.items():
            message = Imu()
            message.header.frame_id = name
            message.header.stamp.sec = 21
            message.orientation_covariance[0] = -1.
            message.angular_velocity.x = 2.5
            message.linear_acceleration.z = 9.81
            callbacks[source](message)
            relayed = publishers[output].publish.call_args.args[0]
            self.assertIs(relayed, message)
            self.assertEqual(deserialize_message(serialize_message(relayed), Imu), message)
            self.assertEqual(sensors.imu_counts[name], 1)
        sensors.close()
        self.assertEqual(node.destroy_subscription.call_count, 5)
        self.assertEqual(node.destroy_publisher.call_count, 5)
        robot.deactivate_lidar.assert_not_called()

    def test_main_uses_one_connection_and_cleans_up_after_invalid_sample(self):
        robot = robot_fixture()
        sdk = Mock()
        def connect():
            self.assertEqual(builtins.input('SDK prompt'), '')
            return robot
        sdk.Astribot.side_effect = connect
        original_input = builtins.input
        valid = data.read_state(robot, 1.0)
        with patch.dict(sys.modules, {'astribot_sdk.core.astribot_api.astribot_client': sdk}), \
                patch.object(sys, 'argv', ['publish_robot_data', '--cameras', 'head_rgbd']), \
                patch.object(data, 'SensorPublisher') as sensors, \
                patch.object(data, 'read_state', side_effect=[ValueError('missing state'), valid]), \
                patch.object(data.rclpy, 'ok', side_effect=[True, True, False, True]), \
                patch.object(data.rclpy, 'shutdown') as shutdown, \
                patch.object(data.time, 'sleep'), contextlib.redirect_stdout(io.StringIO()):
            data.main()
        sdk.Astribot.assert_called_once_with()
        sensors.return_value.start.assert_called_once()
        sensors.return_value.close.assert_called_once()
        shutdown.assert_called_once()
        self.assertIs(builtins.input, original_input)
        robot.astribot_interface.node.get_logger().warning.assert_called_once()
        self.assertEqual(robot.astribot_interface.node.create_publisher.call_count,
                         robot.astribot_interface.node.destroy_publisher.call_count)
        self.assertEqual(robot.astribot_interface.node.create_publisher.return_value.publish.call_count,
                         len(valid) + 1)

    def test_recording_command_selects_sensors_and_state_without_shell_expansion(self):
        script = Path(__file__).with_name('record_robot_data.sh')
        with tempfile.TemporaryDirectory() as directory:
            ros2 = Path(directory) / 'ros2'
            ros2.write_text('#!/usr/bin/env python3\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n')
            ros2.chmod(0o755)
            bag = str(Path(directory) / 'bag with spaces $(not_a_command)')
            result = subprocess.run(['bash', str(script), bag], check=True, capture_output=True, text=True,
                                    env={**os.environ, 'PATH': directory + os.pathsep + os.environ['PATH']})
        args = json.loads(result.stdout)
        self.assertEqual(args[:4], ['bag', 'record', '-o', bag])
        pattern = args[args.index('--regex') + 1]
        for topic in [*data.STATE_TOPICS, '/sdk/robot_description', '/livox/lidar_back',
                      '/livox/lidar_front', *(src for src, out in data.IMU_TOPICS.values()),
                      *(out for src, out in data.IMU_TOPICS.values())]:
            self.assertIsNotNone(re.fullmatch(pattern, topic), topic)
        self.assertIsNone(re.fullmatch(pattern, '/astribot_torso/joint_space_command'))
        import yaml
        qos = yaml.safe_load(Path(args[args.index('--qos-profile-overrides-path') + 1]).read_text())
        self.assertEqual(qos['/sdk/robot_description']['durability'], 'transient_local')


if __name__ == '__main__':
    unittest.main()
