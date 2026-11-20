"""AprilTag/ArUco pose validation and short-lived RGB axes for the RViz tree."""
from copy import deepcopy
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray


def load_sources(cameras_path, tags_path):
    cameras = yaml.safe_load(Path(cameras_path).read_text())['cameras']
    tags = yaml.safe_load(Path(tags_path).read_text())
    family = tags.get('family', 'tag36h11')
    counts = {'tag16h5': 30, 'tag25h9': 35, 'tag36h11': 587}
    counts.update({f'DICT_{bits}X{bits}_{count}': count
                   for bits in (4, 5, 6, 7) for count in (50, 100, 250, 1000)})
    counts['DICT_ARUCO_ORIGINAL'] = 1024
    if family not in counts:
        raise ValueError(f'Unsupported tag family: {family}')
    lengths = {}
    for tag in tags['tags']:
        ident, length = tag['id'], float(tag['length_mm']) / 1000.0
        if (type(ident) is not int or not 0 <= ident < counts[family] or ident in lengths
                or not math.isfinite(length) or length <= 0):
            raise ValueError('Tag IDs must be unique/in range and lengths finite/positive')
        lengths[ident] = length
    if not cameras or not lengths:
        raise ValueError('Camera and tag configurations must be nonempty')
    sources = {}
    for name, camera in cameras.items():
        frame = camera['frame_id']
        topic = camera.get('detections_topic', f'{name}/tag_detections')
        if not isinstance(frame, str) or not frame or frame.startswith('/'):
            raise ValueError(f'Invalid optical frame for {name}')
        sources[name] = (topic, frame)
    if len({topic for topic, _ in sources.values()}) != len(sources):
        raise ValueError('Detection topics must be unique')
    return sources, family, lengths


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    values = np.asarray([p.x, p.y, p.z, q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values[3:])
    if not np.isfinite(values).all() or not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid tag/TF position or quaternion')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(values[3:] / norm).as_matrix()
    result[:3, 3] = values[:3]
    return result


class TagAxes:
    """Each camera owns its marker namespace; lost IDs are deleted immediately."""
    def __init__(self, family, lengths, axis_length=0.10, lifetime=1.0):
        if not all(math.isfinite(v) and v > 0 for v in (axis_length, lifetime)):
            raise ValueError('Axis length and lifetime must be finite and positive')
        self.family, self.lengths = family, lengths
        self.axis_length, self.lifetime = axis_length, lifetime
        self.visible = {}

    def update(self, camera, message, fixed_from_camera, frame, stamp):
        markers, current = MarkerArray(), set()
        for detection in message.detections:
            try:
                ident = int(detection.id)
                if ident not in self.lengths or ident in current or len(detection.results) != 1:
                    continue
                result = detection.results[0]
                if result.hypothesis.class_id != str(ident):
                    continue
                if detection.header.frame_id not in ('', message.header.frame_id):
                    continue
                tag_pose = pose_matrix(result.pose.pose)
                if tag_pose[2, 3] <= 0:
                    continue
                fixed_from_tag = fixed_from_camera @ tag_pose
            except (ValueError, TypeError, OverflowError):
                continue
            current.add(ident)
            quaternion = Rotation.from_matrix(fixed_from_tag[:3, :3]).as_quat()
            for axis in range(4):
                marker = Marker()
                marker.header.frame_id, marker.header.stamp = frame, stamp
                marker.ns, marker.id = f'apriltag/{camera}/{self.family}', ident * 4 + axis
                marker.action = Marker.ADD
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(
                    float, fixed_from_tag[:3, 3])
                q = marker.pose.orientation
                q.x, q.y, q.z, q.w = map(float, quaternion)
                marker.lifetime = Duration(seconds=self.lifetime).to_msg()
                marker.color.a = 1.0
                if axis < 3:
                    marker.type = Marker.ARROW
                    end = Point()
                    setattr(end, ('x', 'y', 'z')[axis], self.axis_length)
                    marker.points = [Point(), end]
                    marker.scale.x, marker.scale.y, marker.scale.z = 0.006, 0.012, 0.018
                    setattr(marker.color, ('r', 'g', 'b')[axis], 1.0)
                else:
                    marker.type = Marker.TEXT_VIEW_FACING
                    marker.pose.position.z += self.axis_length + 0.03
                    marker.scale.z = 0.05
                    marker.color.r = marker.color.g = marker.color.b = 1.0
                    marker.text = f'{camera}: {self.family} / {ident}'
                markers.markers.append(marker)
        for ident in self.visible.get(camera, set()) - current:
            for axis in range(4):
                marker = Marker()
                marker.header.frame_id, marker.header.stamp = frame, stamp
                marker.ns, marker.id = f'apriltag/{camera}/{self.family}', ident * 4 + axis
                marker.action = Marker.DELETE
                markers.markers.append(marker)
        self.visible[camera] = current
        return markers

    def clear(self, camera, message, frame, stamp):
        empty = deepcopy(message)
        empty.detections = []
        return self.update(camera, empty, np.eye(4), frame, stamp)
