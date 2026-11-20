"""Nominal optical mounts: direction, articulated parents and RViz references."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from astribot_scripts.common.camera_frames import camera_mounts, optical_rpy
from astribot_scripts.common.state import robot_description

ROOT = Path(__file__).resolve().parents[1]


class CameraFramesTests(unittest.TestCase):
    def test_optical_forward_and_down_match_source_mujoco_camera(self):
        for mount in camera_mounts().values():
            w, x, y, z = mount['quaternion_wxyz']
            source = Rotation.from_quat([x, y, z, w]).as_matrix()
            optical = Rotation.from_euler('xyz', optical_rpy([w, x, y, z])).as_matrix()
            np.testing.assert_allclose(optical[:, 0], source[:, 0], atol=1e-12)
            np.testing.assert_allclose(optical[:, 1:], -source[:, 1:], atol=1e-12)
            scaled = optical_rpy([5*w, 5*x, 5*y, 5*z])
            np.testing.assert_allclose(Rotation.from_euler('xyz', scaled).as_matrix(), optical, atol=1e-12)
        for bad in ([0, 0, 0, 0], [1, 0, 0], [float('nan'), 0, 0, 1]):
            with self.assertRaises(ValueError):
                optical_rpy(bad)

    def test_generated_frames_follow_correct_articulated_links(self):
        root = ET.fromstring(robot_description())
        mounts = camera_mounts()
        expected = {'head_rgbd': 'astribot_head_link_2', 'torso_rgbd': 'astribot_torso_link_4',
                    'left_wrist_rgbd': 'astribot_arm_left_link_7',
                    'right_wrist_rgbd': 'astribot_arm_right_link_7'}
        self.assertEqual(set(mounts), set(expected))
        for camera, parent in expected.items():
            child = f'astribot_viz_{camera}_optical'
            joint = root.find(f"joint[@name='{child}_joint']")
            self.assertEqual(joint.get('type'), 'fixed')
            self.assertEqual(joint.find('parent').get('link'), parent)
            self.assertEqual(joint.find('child').get('link'), child)
            np.testing.assert_allclose(list(map(float, joint.find('origin').get('xyz').split())),
                                       mounts[camera]['position'], atol=1e-12)
        self.assertEqual(len([j for j in root.findall('joint') if j.get('type') != 'fixed']), 20)

    def test_rviz_camera_axes_resolve_to_generated_optical_frames(self):
        displays = yaml.safe_load((ROOT/'config/astribot_fast_lio.rviz').read_text())['Visualization Manager']['Displays']
        group = next(d for d in displays if d['Name'] == 'Camera optical frames')
        self.assertTrue(group['Enabled'])
        self.assertEqual({d['Reference Frame'] for d in group['Displays']},
                         {f'astribot_viz_{name}_optical' for name in camera_mounts()})
        self.assertTrue(all(d['Enabled'] and d['Class'] == 'rviz_default_plugins/Axes' for d in group['Displays']))


if __name__ == '__main__':
    unittest.main()
