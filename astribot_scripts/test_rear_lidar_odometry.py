"""Rigid-frame conversion checks without a robot or ROS node."""
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message, serialize_message
import yaml

import rear_lidar_odometry as adapter


class RearOdometryTests(unittest.TestCase):
    def message(self, matrix):
        msg = Odometry()
        msg.header.frame_id = 'kiss_rear_odom'
        msg.header.stamp.sec = 42
        msg.header.stamp.nanosec = 123
        msg.child_frame_id = 'livox_frame'
        adapter.set_pose(msg.pose.pose, matrix)
        msg.pose.covariance = (np.eye(6) * 0.1).ravel().tolist()
        return msg

    def test_base_motion_preserves_map_frame_and_timestamp(self):
        mount = np.eye(4)
        mount[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
        mount[:3, 3] = [1, 2, 0.3]
        base = np.eye(4)
        base[:3, :3] = Rotation.from_euler('xyz', [.1, -.2, .8]).as_matrix()
        base[:3, 3] = [4, 5, 0.2]
        source = self.message(base @ mount)
        result = adapter.convert(source, mount, 'livox_frame')
        result = deserialize_message(serialize_message(result), Odometry)
        np.testing.assert_allclose(adapter.pose_matrix(result.pose.pose), base, atol=1e-12)
        self.assertEqual(result.header, source.header)
        self.assertEqual(result.child_frame_id, adapter.BASE_FRAME)
        self.assertEqual(list(result.twist.covariance)[::7], [1e12] * 6)
        self.assertEqual(source.child_frame_id, 'livox_frame')

    def test_rotating_lidar_offset_does_not_create_base_translation(self):
        mount = np.eye(4)
        mount[0, 3] = 1.
        for yaw in (0, .5, 1.5):
            base = np.eye(4)
            base[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
            result = adapter.convert(self.message(base @ mount), mount, 'livox_frame')
            np.testing.assert_allclose(adapter.pose_matrix(result.pose.pose)[:3, 3], 0, atol=1e-12)

    def test_covariance_includes_rotational_lever_arm(self):
        mount = np.eye(4)
        mount[0, 3] = 2.
        result = adapter.convert(self.message(np.eye(4)), mount, 'livox_frame')
        cov = np.array(result.pose.covariance).reshape(6, 6)
        self.assertAlmostEqual(cov[1, 1], .5)
        self.assertAlmostEqual(cov[1, 5], -.2)
        np.testing.assert_allclose(cov, cov.T)
        self.assertGreater(np.linalg.eigvalsh(cov).min(), 0)

    def test_rejects_wrong_frames_and_invalid_pose(self):
        for field, value in [('child_frame_id', 'base_link'), ('header.frame_id', 'other_odom')]:
            msg = self.message(np.eye(4))
            if '.' in field:
                msg.header.frame_id = value
            else:
                msg.child_frame_id = value
            with self.assertRaises(ValueError):
                adapter.convert(msg, np.eye(4), 'livox_frame')
        msg = self.message(np.eye(4))
        msg.pose.pose.orientation.w = 0.
        with self.assertRaises(ValueError):
            adapter.convert(msg, np.eye(4), 'livox_frame')

    def test_calibration_is_required_and_reflections_are_rejected(self):
        template = Path(__file__).with_name('rear_lidar_to_base.yaml')
        with self.assertRaises(ValueError):
            adapter.load_calibration(template)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'calibration.yaml'
            data = {'cloud_frame': 'livox_frame', 'base_from_cloud': np.eye(4).tolist()}
            path.write_text(yaml.safe_dump(data))
            frame, matrix = adapter.load_calibration(path)
            self.assertEqual(frame, 'livox_frame')
            np.testing.assert_allclose(matrix, np.eye(4))
            for invalid in (np.diag([-1, 1, 1, 1]), np.ones((4, 4)), np.eye(3)):
                data['base_from_cloud'] = invalid.tolist()
                path.write_text(yaml.safe_dump(data))
                with self.assertRaises(ValueError):
                    adapter.load_calibration(path)


if __name__ == '__main__':
    unittest.main()
