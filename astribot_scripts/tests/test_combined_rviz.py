"""Combined LIO/robot visualization geometry and one-window launch contract."""
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry, Path as RosPath
from geometry_msgs.msg import PoseStamped

from astribot_scripts.common.fast_lio_viz import rear_imu_from_sdk_chassis
from astribot_scripts.fast_lio_display_tf import (display_transform, chassis_odometry,
                                                 chassis_path, pose_matrix, set_pose)
from astribot_scripts.common.state import FRAME

ROOT = Path(__file__).resolve().parents[1]


class CombinedRvizTests(unittest.TestCase):
    def test_mount_includes_sdk_weld_and_has_correct_inverse_direction(self):
        imu_from_chassis = rear_imu_from_sdk_chassis()
        chassis_from_imu = np.linalg.inv(imu_from_chassis)
        np.testing.assert_allclose(chassis_from_imu[:3, 3],
                                   [-0.203837491, -0.183281143, 0.210277], atol=1e-9)
        self.assertAlmostEqual(Rotation.from_matrix(chassis_from_imu[:3, :3]).as_euler('xyz', degrees=True)[2],
                               135.0, places=5)
        # A nonzero world yaw and position must not rotate/translate the lever arm twice.
        world_from_chassis = np.eye(4)
        world_from_chassis[:3, :3] = Rotation.from_euler('z', 0.7).as_matrix()
        world_from_chassis[:3, 3] = [3., -2., .1]
        world_from_imu = world_from_chassis @ chassis_from_imu
        np.testing.assert_allclose(world_from_imu @ imu_from_chassis, world_from_chassis, atol=1e-12)

    def test_nominal_model_origin_matches_first_urdf_joint(self):
        # Pinned MJCF kinematic_visual.xml first torso joint, relative to chassis_base.
        mjcf = Rotation.from_quat([-.4999999999966268, .4999999999966268,
                                  .5000018366025517, .49999816339744824]).as_matrix()
        urdf = ET.parse(ROOT.parent / 'astribot_config/robot_config/astribot_s1/model/astribot_whole_body_with_head.urdf')
        joint = urdf.find("joint[@name='astribot_torso_joint_1']")
        self.assertEqual(joint.find('parent').get('link'), 'astribot_torso_base')
        origin = joint.find('origin')
        np.testing.assert_allclose(list(map(float, origin.get('xyz').split())), [0., 0., .12])
        np.testing.assert_allclose(Rotation.from_euler('xyz', list(map(float, origin.get('rpy').split()))).as_matrix(),
                                   mjcf, atol=1e-12)

    def test_yaml_uses_one_combined_rviz_and_shared_tf(self):
        config = yaml.safe_load((ROOT / 'config/astribot.tmux.yaml').read_text())
        panes = config['windows'][0]['panes']
        sensor, mapping, view = (' '.join(p['shell_command']) for p in panes)
        self.assertIn('--observe-only', sensor)
        self.assertIn('--cameras head_rgbd torso_rgbd left_wrist_rgbd right_wrist_rgbd', sensor)
        self.assertIn('--no-depth --color-encoding rgb8', sensor)
        self.assertIn('--image-rate 3', sensor)
        self.assertIn('--image-size 640 480', sensor)
        self.assertIn('rviz:=false', mapping)
        self.assertNotIn('tf_namespace:=astribot_viz', mapping)
        self.assertIn('astribot_lio_rviz.launch.py', view)
        self.assertIn('desktop_exec.py', view)
        launch = (ROOT / 'launch/astribot_lio_rviz.launch.py').read_text()
        self.assertEqual(launch.count("executable='rviz2'"), 1)
        self.assertIn("'rviz': 'false', 'publish_state': 'false', 'odometry': 'false'", launch)
        self.assertIn('GroupAction([IncludeLaunchDescription', launch)

    def test_display_time_tf_preserves_source_pose_and_measurement_stamp(self):
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'map', 'body'
        odom.header.stamp.sec = 100
        odom.pose.pose.position.x = 2.5
        odom.pose.pose.orientation.w = 2.0
        converted = chassis_odometry(odom, rear_imu_from_sdk_chassis())
        tf = display_transform(converted, Time(sec=130))
        self.assertEqual(tf.header.stamp.sec, 130)
        self.assertEqual(tf.child_frame_id, FRAME)
        self.assertEqual(converted.header.stamp.sec, 100)
        self.assertEqual(odom.header.stamp.sec, 100)
        self.assertEqual(odom.pose.pose.orientation.w, 2.0)
        self.assertEqual(tf.transform.translation.x, converted.pose.pose.position.x)
        self.assertAlmostEqual(tf.transform.rotation.w, converted.pose.pose.orientation.w)
        converted.child_frame_id = 'wrong_sensor'
        with self.assertRaises(ValueError):
            display_transform(converted, Time())

    def test_rotating_imu_lever_arm_does_not_move_the_chassis_center(self):
        mount = rear_imu_from_sdk_chassis()
        for yaw in [0., .7, -1.2]:
            world_from_chassis = np.eye(4)
            world_from_chassis[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
            world_from_chassis[:3, 3] = [3., -2., .1]
            source = Odometry()
            source.header.frame_id, source.child_frame_id = 'map', 'body'
            source.header.stamp.sec = 42
            set_pose(source.pose.pose, world_from_chassis @ np.linalg.inv(mount))
            source.pose.covariance = np.eye(6).ravel().tolist()
            result = chassis_odometry(source, mount)
            np.testing.assert_allclose(pose_matrix(result.pose.pose), world_from_chassis, atol=1e-12)
            self.assertEqual(result.header, source.header)
            self.assertEqual(result.child_frame_id, FRAME)
            covariance = np.asarray(result.pose.covariance).reshape(6, 6)
            self.assertGreater(np.trace(covariance[:3, :3]), 3.)
            np.testing.assert_allclose(covariance, covariance.T, atol=1e-12)
            self.assertTrue(np.all(np.linalg.eigvalsh(covariance) > 0))
            self.assertEqual(list(result.twist.covariance)[0], 1e12)

    def test_chassis_path_preserves_world_and_all_pose_timestamps(self):
        mount = rear_imu_from_sdk_chassis()
        source = RosPath()
        source.header.frame_id, source.header.stamp.sec = 'map', 22
        for i in range(2):
            pose = PoseStamped()
            pose.header.frame_id, pose.header.stamp.sec = 'map', 20 + i
            pose.pose.position.x = float(i)
            pose.pose.orientation.w = 1.0
            source.poses.append(pose)
        result = chassis_path(source, mount)
        self.assertEqual(result.header, source.header)
        for before, after in zip(source.poses, result.poses):
            self.assertEqual(after.header, before.header)
            np.testing.assert_allclose(pose_matrix(after.pose), pose_matrix(before.pose) @ mount, atol=1e-12)
        self.assertEqual(source.poses[0].pose.position.x, 0.)
        self.assertEqual(source.poses[0].pose.orientation.w, 1.)

    def test_chassis_conversion_rejects_wrong_frame_or_nonfinite_covariance(self):
        source = Odometry()
        source.header.frame_id, source.child_frame_id = 'map', 'body'
        source.pose.pose.orientation.w = 1.
        source.pose.covariance[0] = float('nan')
        with self.assertRaises(ValueError):
            chassis_odometry(source, rear_imu_from_sdk_chassis())
        source.pose.covariance[0] = 0.
        source.child_frame_id = FRAME
        with self.assertRaises(ValueError):
            chassis_odometry(source, rear_imu_from_sdk_chassis())

    def test_displays_use_lio_world_and_preserve_robot_and_sensor_topics(self):
        config = yaml.safe_load((ROOT / 'config/astribot_fast_lio.rviz').read_text())['Visualization Manager']
        self.assertEqual(config['Global Options']['Fixed Frame'], 'map')
        displays = config['Displays']
        topics = {d['Topic']['Value'] for d in displays if 'Topic' in d}
        self.assertTrue({'/fast_lio/rear/cloud_registered','/fast_lio/chassis/odometry',
                         '/fast_lio/chassis/path'}.issubset(topics))
        self.assertEqual(config['Views']['Current']['Target Frame'], 'map')
        grid = next(d for d in displays if d['Class'] == 'rviz_default_plugins/Grid')
        self.assertEqual(grid['Reference Frame'], 'map')
        launch = (ROOT / 'launch/astribot_lio_rviz.launch.py').read_text()
        self.assertIn("'--frame-id', FRAME, '--child-frame-id', 'body'", launch)
        self.assertEqual(sum(d['Class']=='rviz_default_plugins/RobotModel' for d in displays), 1)
        self.assertEqual(sum(d['Class']=='rviz_default_plugins/MarkerArray' for d in displays), 2)
        images = [d for d in displays if d['Class']=='rviz_default_plugins/Image']
        self.assertEqual({d['Topic']['Value'] for d in images}, {
            f'/sdk/astribot_camera/{c}/color_image' for c in
            ['head_rgbd', 'torso_rgbd', 'left_wrist_rgbd', 'right_wrist_rgbd']})
        cloud = next(d for d in displays if d['Class']=='rviz_default_plugins/PointCloud2')
        self.assertGreater(cloud['Decay Time'], 0)
        self.assertLessEqual(cloud['Decay Time'], 120)

    def test_odometry_history_and_full_session_path_are_visible_in_map(self):
        config = yaml.safe_load((ROOT / 'config/astribot_fast_lio.rviz').read_text())['Visualization Manager']
        odom = next(d for d in config['Displays'] if d['Class'] == 'rviz_default_plugins/Odometry')
        self.assertTrue(odom['Enabled'])
        self.assertEqual(odom['Keep'], 1000)
        self.assertEqual(odom['Topic']['Value'], '/fast_lio/chassis/odometry')
        self.assertGreater(odom['Position Tolerance'], 0)
        self.assertGreater(odom['Angle Tolerance'], 0)
        path = next(d for d in config['Displays'] if d['Class'] == 'rviz_default_plugins/Path')
        self.assertTrue(path['Enabled'])
        self.assertEqual(path['Topic']['Value'], '/fast_lio/chassis/path')
        self.assertEqual(path['Buffer Length'], 1)  # Each source message contains the session trajectory.
        self.assertEqual(path['Line Style'], 'Billboards')
        self.assertGreater(path['Line Width'], 0)
        self.assertEqual(config['Global Options']['Fixed Frame'], 'map')
        self.assertEqual(config['Views']['Current']['Target Frame'], 'map')

    def test_joint_axes_reference_real_chassis_torso_head_and_wrist_frames(self):
        displays = yaml.safe_load((ROOT / 'config/astribot_fast_lio.rviz').read_text())['Visualization Manager']['Displays']
        group = next(d for d in displays if d['Name'] == 'Robot joint axes')
        self.assertEqual(group['Class'], 'rviz_common/Group')
        self.assertTrue(group['Enabled'])
        expected = {FRAME, *(f'astribot_torso_link_{i}' for i in range(1, 5)),
                    'astribot_head_link_1', 'astribot_head_link_2',
                    'astribot_arm_left_link_7', 'astribot_arm_right_link_7'}
        axes = group['Displays']
        self.assertEqual({a['Reference Frame'] for a in axes}, expected)
        self.assertEqual(len(axes), len(expected))
        urdf = ET.parse(ROOT.parent / 'astribot_config/robot_config/astribot_s1/model/astribot_whole_body_with_head.urdf')
        links = {link.get('name') for link in urdf.findall('link')} | {FRAME}
        for axis in axes:
            self.assertEqual(axis['Class'], 'rviz_default_plugins/Axes')
            self.assertTrue(axis['Enabled'])
            self.assertIn(axis['Reference Frame'], links)
            self.assertGreater(axis['Length'], axis['Radius'])


if __name__ == '__main__':
    unittest.main()
