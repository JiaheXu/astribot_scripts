"""Nominal rear IMU mount for the combined visualization, not calibration."""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def rear_imu_from_sdk_chassis():
    """Return T_imu_sdk_chassis, including the SDK's torso weld offset.

    The pinned MJCF chassis_base matches the URDF astribot_torso_base model
    origin: same base mesh placement and all four torso joint transforms.
    It is NOT the SDK chassis origin, which is 0.097 m below torso_base.
    This correspondence is nominal model geometry, not a measured calibration.
    """
    root = Path(__file__).resolve().parents[2]
    model = json.loads((root / 'astribot_scripts/integrations/fast_lio/rear_nominal_extrinsics.json').read_text())
    torso = yaml.safe_load((root / 'astribot_config/robot_config/astribot_s1/astribot_torso.yaml').read_text())
    weld = np.asarray(torso['transform']['weld_to_base_pose'], dtype=float)
    if weld.shape != (6,) or not np.isfinite(weld).all():
        raise ValueError('Invalid SDK chassis-to-torso weld')
    chassis_from_torso = np.eye(4)
    chassis_from_torso[:3, :3] = Rotation.from_euler('xyz', weld[3:]).as_matrix()
    chassis_from_torso[:3, 3] = weld[:3]

    def matrix(pose):
        values = np.asarray([*pose['translation_m'], *pose['quaternion_xyzw']], dtype=float)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError('Invalid nominal sensor mount')
        result = np.eye(4)
        result[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
        result[:3, 3] = values[:3]
        return result

    chassis_from_imu = (chassis_from_torso @ matrix(model['rear_in_chassis_base'])
                        @ matrix(model['imu_in_rear_lidar']))
    return np.linalg.inv(chassis_from_imu)
