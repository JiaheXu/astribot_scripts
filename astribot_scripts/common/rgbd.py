"""RGB-D projection and timestamp pairing, independent of ROS imports."""
from collections import deque
import math

import cv2
import numpy as np


POINT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])


def stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class FramePairs:
    """Bounded, one-use RGB/depth pairs matched on source timestamps.

    Receipt time is only used to expire queues; it never substitutes for source
    timestamps. A delayed RGB republisher can match an earlier depth arrival.
    """
    def __init__(self, slop=0.06, max_age=1.0, queue_size=30):
        if not all(math.isfinite(v) and v > 0 for v in (slop, max_age)) or queue_size < 1:
            raise ValueError('Pairing limits must be positive and finite')
        self.slop_ns, self.max_age = round(slop * 1e9), max_age
        self.queues = {kind: deque(maxlen=queue_size) for kind in ('color', 'depth')}
        self.last = {'color': -1, 'depth': -1}

    def add(self, kind, message, now):
        timestamp = stamp_ns(message)
        if timestamp <= 0:
            raise ValueError('RGB-D pairing requires nonzero source timestamps')
        for queue in self.queues.values():
            while queue and now - queue[0][0] > self.max_age:
                queue.popleft()
        if timestamp <= self.last[kind]:
            return None
        queue = self.queues[kind]
        if any(stamp_ns(item[1]) == timestamp for item in queue):
            return None
        queue.append((now, message))
        other = 'depth' if kind == 'color' else 'color'
        candidates = self.queues[other]
        if not candidates:
            return None
        index = min(range(len(candidates)), key=lambda i: abs(stamp_ns(candidates[i][1]) - timestamp))
        matched = candidates[index][1]
        if abs(stamp_ns(matched) - timestamp) > self.slop_ns:
            return None
        pair = {kind: message, other: matched}
        for stream, item in pair.items():
            self.last[stream] = stamp_ns(item)
            self.queues[stream] = deque(
                (entry for entry in self.queues[stream] if stamp_ns(entry[1]) > self.last[stream]),
                maxlen=self.queues[stream].maxlen)
        return pair['color'], pair['depth']


def image_array(message):
    """Read Image rows with padding and byte order; return RGB or numeric depth."""
    formats = {'rgb8': ('u1', 3), 'bgr8': ('u1', 3), '16UC1': ('u2', 1), '32FC1': ('f4', 1)}
    if message.encoding not in formats:
        raise ValueError(f'Unsupported image encoding: {message.encoding}')
    dtype, channels = formats[message.encoding]
    dtype = np.dtype(('>' if message.is_bigendian else '<') + dtype)
    width, height, step = message.width, message.height, message.step
    if width <= 0 or height <= 0 or step < width * channels * dtype.itemsize:
        raise ValueError('Invalid Image dimensions/step')
    if len(message.data) != height * step:
        raise ValueError('Image byte count does not match height * step')
    result = np.ndarray((height, width, channels), dtype=dtype,
                        buffer=memoryview(message.data), strides=(step, channels * dtype.itemsize, dtype.itemsize))
    if channels == 1:
        return result[:, :, 0]
    return result[:, :, ::-1] if message.encoding == 'bgr8' else result


def camera_model(config):
    width, height = config['width'], config['height']
    if type(width) is not int or type(height) is not int or min(width, height) <= 0:
        raise ValueError('Calibration width/height must be positive integers')
    if config.get('camera_matrix') is None:
        raise ValueError('Depth camera_matrix is missing; obtain the driver calibration')
    matrix = np.asarray(config['camera_matrix'], dtype=float).reshape(3, 3)
    distortion = np.asarray(config['distortion_coefficients'], dtype=float)
    model = config['distortion_model']
    if model not in ('plumb_bob', 'rational_polynomial'):
        raise ValueError(f'Unsupported distortion model: {model}')
    if distortion.shape != (5 if model == 'plumb_bob' else 8,):
        raise ValueError('Expected 5 plumb_bob or 8 rational_polynomial coefficients')
    if (not np.isfinite(matrix).all() or not np.isfinite(distortion).all()
            or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], [0, 0, 1]) or matrix[0, 1] != 0 or matrix[1, 0] != 0):
        raise ValueError('Invalid camera calibration')
    return (width, height), matrix, distortion


def rigid_matrix(value):
    if value is None:
        raise ValueError('color_from_depth is missing; verify depth-to-color registration/extrinsics')
    result = np.asarray(value, dtype=float).reshape(4, 4)
    rotation = result[:3, :3]
    if (not np.isfinite(result).all() or not np.allclose(result[3], [0, 0, 0, 1])
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5)):
        raise ValueError('Expected a rigid 4x4 transform with translation in meters')
    return result


class Projector:
    """Back-project depth, register to color, and sample measured RGB pixels."""
    def __init__(self, config, stride=2, min_depth=0.2, max_depth=5.0):
        self.color_size, self.color_k, self.color_d = camera_model(config['color'])
        self.depth_size, depth_k, depth_d = camera_model(config['depth'])
        self.color_from_depth = rigid_matrix(config.get('color_from_depth'))
        scale = config.get('depth_unit_m')
        if scale is None or not math.isfinite(scale) or scale <= 0:
            raise ValueError('depth_unit_m must specify verified meters per uint16 depth unit')
        self.scale = scale
        if (type(stride) is not int or stride < 1 or not math.isfinite(min_depth)
                or not math.isfinite(max_depth) or not 0 < min_depth < max_depth):
            raise ValueError('Invalid stride/depth range')
        self.stride, self.min_depth, self.max_depth = stride, min_depth, max_depth
        width, height = self.depth_size
        rows, columns = np.mgrid[0:height:stride, 0:width:stride]
        pixels = np.stack((columns, rows), axis=-1).astype(np.float64).reshape(-1, 1, 2)
        rays = cv2.undistortPoints(pixels, depth_k, depth_d).reshape(-1, 2)
        self.rays = np.column_stack((rays, np.ones(len(rays))))

    def _color_points(self, depth):
        if depth.shape != (self.depth_size[1], self.depth_size[0]):
            raise ValueError('Depth image does not match its calibrated resolution')
        if (depth.dtype.kind, depth.dtype.itemsize) not in (('u', 2), ('f', 4)):
            raise ValueError('Expected uint16 depth units or float32 depth in meters')
        z = depth[::self.stride, ::self.stride].reshape(-1).astype(np.float64)
        if depth.dtype.kind == 'u':
            z *= self.scale
        valid = np.isfinite(z) & (z >= self.min_depth) & (z <= self.max_depth)
        points = self.rays[valid] * z[valid, None]
        points = points @ self.color_from_depth[:3, :3].T + self.color_from_depth[:3, 3]
        return points[np.isfinite(points).all(axis=1) & (points[:, 2] > 0)]

    def rectified_depth(self, depth):
        """Nearest visible metric Z in the undistorted RGB raster, zero in holes."""
        points = self._color_points(depth)
        width, height = self.color_size
        result = np.full((height, width), np.inf, dtype=np.float32)
        if len(points):
            uvz = points @ self.color_k.T
            uv = np.rint(uvz[:, :2] / uvz[:, 2:3])
            inside = ((uv[:, 0] >= 0) & (uv[:, 0] < width) &
                      (uv[:, 1] >= 0) & (uv[:, 1] < height))
            u, v = uv[inside].astype(int).T
            np.minimum.at(result, (v, u), points[inside, 2])
        result[~np.isfinite(result)] = 0
        return result

    def project(self, color, depth):
        if color.shape != (self.color_size[1], self.color_size[0], 3) or color.dtype != np.uint8:
            raise ValueError('RGB image does not match its calibrated resolution/encoding')
        points = self._color_points(depth)
        if not len(points):
            return np.empty(0, dtype=POINT_DTYPE)
        pixels, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), self.color_k, self.color_d)
        pixels = pixels.reshape(-1, 2)
        width, height = self.color_size
        visible = (np.isfinite(pixels).all(axis=1) & (pixels[:, 0] >= 0) & (pixels[:, 0] <= width - 1)
                   & (pixels[:, 1] >= 0) & (pixels[:, 1] <= height - 1))
        points, pixels = points[visible], np.rint(pixels[visible]).astype(int)
        # Keep the nearest depth at each color pixel to reduce foreground color
        # bleeding onto occluded background surfaces during registration.
        order = np.argsort(points[:, 2], kind='stable')
        _, first = np.unique(pixels[order, 1] * width + pixels[order, 0], return_index=True)
        keep = order[first]
        points, pixels = points[keep], pixels[keep]
        rgb = color[pixels[:, 1], pixels[:, 0]].astype(np.uint32)
        result = np.empty(len(points), dtype=POINT_DTYPE)
        for i, field in enumerate(('x', 'y', 'z')):
            result[field] = points[:, i]
        result['rgb'] = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
        return result


def transform_points(points, matrix):
    matrix = rigid_matrix(matrix)
    xyz = np.column_stack([points[field] for field in ('x', 'y', 'z')])
    xyz = xyz @ matrix[:3, :3].T + matrix[:3, 3]
    result = points.copy()
    for i, field in enumerate(('x', 'y', 'z')):
        result[field] = xyz[:, i]
    return result
