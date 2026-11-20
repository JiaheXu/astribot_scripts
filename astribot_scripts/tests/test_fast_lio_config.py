"""Check the rear profile against the recorded nominal robot-description poses."""
import json
from pathlib import Path
import unittest

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def pose_matrix(pose):
    q = np.asarray(pose['quaternion_xyzw'], dtype=float)
    x, y, z, w = q / np.linalg.norm(q)
    out = np.eye(4)
    out[:3, :3] = [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ]
    out[:3, 3] = pose['translation_m']
    return out


class FastLioConfigTests(unittest.TestCase):
    def setUp(self):
        self.model = json.loads((ROOT / 'integrations/fast_lio/rear_nominal_extrinsics.json').read_text())
        self.config = yaml.safe_load((ROOT / 'config/fast_lio_rear.yaml').read_text())['/**']['ros__parameters']

    def test_cloud_correction_matches_model_frame_composition(self):
        expected = np.linalg.inv(pose_matrix(self.model['rear_in_chassis_base'])) @ pose_matrix(self.model['front_in_chassis_base'])
        pre = self.config['preprocess']
        actual = np.eye(4)
        actual[:3, :3] = np.array(pre['cloud_to_lidar_R']).reshape(3, 3)
        actual[:3, 3] = pre['cloud_to_lidar_T']
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        np.testing.assert_allclose(actual[:3, :3].T @ actual[:3, :3], np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(actual[:3, :3]), 1.0)

    def test_full_chain_recovers_native_point_then_imu_coordinates(self):
        cloud_from_rear = np.linalg.inv(pose_matrix(self.model['front_in_chassis_base'])) @ pose_matrix(self.model['rear_in_chassis_base'])
        imu_from_rear = np.linalg.inv(pose_matrix(self.model['imu_in_rear_lidar']))
        pre, mapping = self.config['preprocess'], self.config['mapping']
        R = np.array(pre['cloud_to_lidar_R']).reshape(3, 3)
        t = np.array(pre['cloud_to_lidar_T'])
        imu_R = np.array(mapping['extrinsic_R']).reshape(3, 3)
        imu_t = np.array(mapping['extrinsic_T'])
        for native in ([0, 0, 0, 1], [1.2, -0.7, 0.3, 1], [-3, 2, -0.1, 1]):
            cloud = cloud_from_rear @ native
            recovered = R @ cloud[:3] + t
            np.testing.assert_allclose(recovered, native[:3], atol=1e-12)
            np.testing.assert_allclose(imu_R @ recovered + imu_t, (imu_from_rear @ native)[:3], atol=1e-12)

    def test_uses_rear_pair_and_fixed_extrinsics_with_measured_point_times(self):
        c = self.config
        self.assertEqual(c['common']['lid_topic'], '/livox/lidar_back')
        self.assertEqual(c['common']['imu_topic'], '/livox/imu_back')
        self.assertEqual(c['preprocess']['lidar_type'], 5)
        self.assertEqual(c['preprocess']['timestamp_unit'], 3)
        self.assertFalse(c['mapping']['extrinsic_est_en'])
        self.assertFalse(c['common']['time_sync_en'])
        self.assertEqual(c['common']['time_offset_lidar_to_imu'], 0.0)

    def test_ros_parameter_arrays_have_homogeneous_double_types(self):
        for sensor in ('front', 'rear'):
            c = yaml.safe_load((ROOT / f'config/fast_lio_{sensor}.yaml').read_text())['/**']['ros__parameters']
            self.assertEqual(c['common']['world_frame'], 'map')
            arrays = [c['mapping']['extrinsic_R'], c['mapping']['extrinsic_T']]
            if sensor == 'rear':
                arrays += [c['preprocess']['cloud_to_lidar_R'], c['preprocess']['cloud_to_lidar_T']]
            for values in arrays:
                self.assertTrue(all(type(value) is float for value in values))


if __name__ == '__main__':
    unittest.main()
