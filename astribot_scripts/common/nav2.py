"""Geometry, freshness checks and parameter merging for the Humble Nav2 bridge."""
from copy import deepcopy
import math

import numpy as np
from scipy.spatial.transform import Rotation


ODOM_FRAME = 'astribot_nav_odom'
BASE_FRAME = 'astribot_nav_base_link'
IMU_FRAME = 'astribot_nav_rear_imu'
LIDAR_FRAME = 'astribot_nav_rear_lidar'


def pose_matrix(position, quaternion):
    values = np.asarray([*position, *quaternion], dtype=float)
    if values.shape != (7,) or not np.isfinite(values).all():
        raise ValueError('Pose must contain seven finite values')
    norm = np.linalg.norm(values[3:])
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid pose quaternion')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(values[3:] / norm).as_matrix()
    result[:3, 3] = values[:3]
    return result


def rigid_matrix(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1.0, atol=1e-8)):
        raise ValueError('Expected a finite, proper 4x4 rigid transform')
    return matrix.copy()


class OdometryProjector:
    """Track the chassis in a gravity-aligned, continuous local LIO frame.

    The first chassis translation/yaw define the origin. Full 3D poses are
    retained for cloud transforms; Nav2 uses their planar components. Twist is
    finite-differenced from measured chassis poses, never from commanded motion.
    """
    def __init__(self, imu_from_chassis, max_gap=1.0):
        self.imu_from_chassis = rigid_matrix(imu_from_chassis)
        self.odom_from_world = None
        self.previous = None
        self.max_gap = max_gap
        self.fault = None

    def project(self, position, quaternion, stamp_ns):
        if self.fault:
            raise ValueError(self.fault)
        world_from_chassis = pose_matrix(position, quaternion) @ self.imu_from_chassis
        if self.odom_from_world is None:
            origin = np.eye(4)
            rotation = world_from_chassis[:3, :3]
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            origin[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
            origin[:3, 3] = world_from_chassis[:3, 3]
            self.odom_from_world = np.linalg.inv(origin)
        pose = self.odom_from_world @ world_from_chassis
        twist = None
        if self.previous is not None:
            previous_stamp, previous_pose = self.previous
            dt = (stamp_ns - previous_stamp) * 1e-9
            if dt <= 0.0:
                self.fault = 'LIO timestamps reset/repeated; restart the navigation launch'
                raise ValueError(self.fault)
            if dt > self.max_gap:
                self.fault = 'LIO feedback gap exceeded one second; restart navigation after checking localization'
                raise ValueError(self.fault)
            linear = pose[:3, :3].T @ (pose[:3, 3] - previous_pose[:3, 3]) / dt
            angular = Rotation.from_matrix(previous_pose[:3, :3].T @ pose[:3, :3]).as_rotvec() / dt
            if np.linalg.norm(linear) > 1.5 or np.linalg.norm(angular) > 3.0:
                self.fault = 'Implausible LIO pose jump; restart navigation after checking localization'
                raise ValueError(self.fault)
            twist = np.concatenate([linear, angular])
        self.previous = (stamp_ns, pose.copy())
        return pose, twist


class FeedbackGate:
    """Require recent receipt AND recent, mutually consistent measurement stamps."""
    def __init__(self, timeout=0.5, future_tolerance=0.05, max_skew=0.2):
        self.timeout = timeout
        self.future_tolerance = future_tolerance
        self.max_skew = max_skew
        self.samples = {}

    def mark(self, stream, stamp, received):
        self.samples[stream] = (stamp, received)

    def invalidate(self, stream):
        self.samples.pop(stream, None)

    def reason(self, ros_now, monotonic_now):
        for stream in ('odom', 'cloud'):
            if stream not in self.samples:
                return f'waiting for valid {stream}'
            stamp, received = self.samples[stream]
            if not all(math.isfinite(v) for v in (stamp, received, ros_now, monotonic_now)):
                return f'invalid {stream} clock'
            if not 0.0 <= monotonic_now - received < self.timeout:
                return f'{stream} receipt timed out'
            age = ros_now - stamp
            if not -self.future_tolerance <= age < self.timeout:
                return f'{stream} stamp age {age:.3f}s; check clock synchronization/stamp_offset'
        if abs(self.samples['odom'][0] - self.samples['cloud'][0]) > self.max_skew:
            return 'odometry/cloud timestamps disagree'
        return None


def merge_parameters(defaults, overrides):
    """Retain installed Humble BT plugins while replacing robot-specific values."""
    def merge(target, updates):
        for key, value in updates.items():
            if isinstance(value, dict) and isinstance(target.get(key), dict):
                merge(target[key], value)
            else:
                target[key] = deepcopy(value)

    result = deepcopy(defaults)
    merge(result, overrides)
    # ROS parameter dictionaries merge recursively. Remove unused sample-robot
    # sensor plugins entirely instead of retaining their /scan subscriptions.
    for name in ('local_costmap', 'global_costmap'):
        params = result[name][name]['ros__parameters']
        for plugin in ('static_layer', 'voxel_layer'):
            if plugin not in params['plugins']:
                params.pop(plugin, None)
        params['obstacle_layer'].pop('scan', None)
    return result
