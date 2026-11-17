"""Offline checks: no SDK connection or robot commands."""
from pathlib import Path
import unittest
from unittest.mock import MagicMock, patch
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

from builtin_interfaces.msg import Time
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

import rviz_state as viz


class VisualizationTests(unittest.TestCase):
    def test_model_meshes_and_joint_order_match_sdk_groups(self):
        root = ET.fromstring(viz.robot_description())
        movable = {j.attrib['name'] for j in root.findall('joint')
                   if j.attrib['type'] != 'fixed'}
        expected = {j for names in viz.MODEL_JOINTS.values() for j in names}
        self.assertEqual(movable, expected)
        self.assertEqual(len(movable), 20)
        base = root.find("joint[@name='astribot_viz_base_joint']")
        self.assertEqual(base.find('parent').attrib['link'], viz.FRAME)
        self.assertEqual(base.find('child').attrib['link'], 'astribot_torso_base')
        self.assertEqual([float(v) for v in base.find('origin').attrib['xyz'].split()],
                         [0., 0., .097])
        for mesh in root.findall('.//mesh'):
            uri = urlparse(mesh.attrib['filename'])
            self.assertEqual(uri.scheme, 'file')
            self.assertTrue(Path(unquote(uri.path)).is_file())

    def test_joint_state_mapping_and_serialization(self):
        values = [[float(i + group * 10) for i in range(len(names))]
                  for group, names in enumerate(viz.MODEL_JOINTS.values())]
        stamp = Time(sec=42, nanosec=123)
        message = viz.joint_message(values, stamp)
        message = deserialize_message(serialize_message(message), JointState)
        self.assertEqual(message.header.stamp, stamp)
        self.assertEqual(message.name, [n for ns in viz.MODEL_JOINTS.values() for n in ns])
        self.assertEqual(list(message.position), [v for row in values for v in row])
        with self.assertRaises(ValueError):
            viz.joint_message(values[:-1], stamp)
        values[0] = [float('nan')] * 4
        with self.assertRaises(ValueError):
            viz.joint_message(values, stamp)

    def test_all_pose_axes_labels_orientation_and_expiry(self):
        poses = [[1., 2., 3., 0., 0., 2., 2.] for _ in viz.POSE_PARTS]
        for state in ('current', 'desired'):
            message = viz.pose_markers(poses, Time(sec=42), state, 1.0)
            message = deserialize_message(serialize_message(message), MarkerArray)
            self.assertEqual(len(message.markers), 20)
            self.assertEqual(len({(m.ns, m.id) for m in message.markers}), 20)
            for index, part in enumerate(viz.POSE_PARTS):
                x, y, z, label = message.markers[4 * index:4 * index + 4]
                self.assertEqual(label.type, Marker.TEXT_VIEW_FACING)
                self.assertIn(part.removeprefix('astribot_'), label.text)
                self.assertIn(state, label.text)
                for axis, marker in enumerate((x, y, z)):
                    self.assertEqual(marker.type, Marker.ARROW)
                    self.assertEqual(marker.header.frame_id, viz.FRAME)
                    self.assertEqual(marker.header.stamp.sec, 42)
                    self.assertEqual(marker.lifetime.sec, 1)
                    self.assertAlmostEqual(marker.pose.orientation.z, 2 ** -.5)
                    self.assertEqual(marker.pose.position.x, 1.)
                    self.assertEqual(len(marker.points), 2)
                    endpoint = marker.points[1]
                    self.assertGreater((endpoint.x, endpoint.y, endpoint.z)[axis], 0.)

    def test_reject_invalid_poses(self):
        poses = [[0., 0., 0., 0., 0., 0., 1.] for _ in viz.POSE_PARTS]
        for bad in ([], [0.] * 7, [float('inf')] * 7, [0.] * 6):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                viz.pose_markers([bad, *poses[1:]], Time(), 'current', 1.0)

    def test_polling_uses_one_connection_and_publishes_without_spinning_again(self):
        sdk = MagicMock()
        robot = sdk.Astribot.return_value
        robot.robot_type = 'S1'
        robot.get_current_joints_position.return_value = [[0.] * len(ns) for ns in viz.MODEL_JOINTS.values()]
        poses = [[0., 0., 0., 0., 0., 0., 1.] for _ in viz.POSE_PARTS]
        robot.get_current_cartesian_pose.return_value = poses
        robot.get_desired_cartesian_pose.return_value = poses
        node = robot.astribot_interface.node
        node.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=42)
        publishers = [MagicMock() for _ in range(3)]
        node.create_publisher.side_effect = publishers
        with patch.dict('sys.modules', {'astribot_sdk.core.astribot_api.astribot_client': sdk}), \
                patch('sys.argv', ['rviz_state.py']), patch.object(viz, 'rclpy') as ros, \
                patch.object(viz.time, 'sleep'):
            ros.ok.side_effect = [True, False, True]
            viz.main()
            sdk.Astribot.assert_called_once_with()
            ros.spin.assert_not_called()
            ros.shutdown.assert_called_once_with()
        robot.get_current_joints_position.assert_called_once_with(names=list(viz.MODEL_JOINTS))
        for state in ('current', 'desired'):
            getattr(robot, f'get_{state}_cartesian_pose').assert_called_once_with(
                names=viz.POSE_PARTS, frame='chassis')
        for publisher in publishers:
            publisher.publish.assert_called_once()
            node.destroy_publisher.assert_any_call(publisher)


if __name__ == '__main__':
    unittest.main()
