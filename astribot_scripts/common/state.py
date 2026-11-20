"""S1 model description, joint mapping and Cartesian pose markers."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from rclpy.duration import Duration
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
import yaml
from astribot_scripts.common.camera_frames import add_camera_frames


FRAME = 'astribot_viz_chassis'
MODEL_JOINTS = {
    f'astribot_{part}': [f'astribot_{part}_joint_{i}' for i in range(1, count + 1)]
    for part, count in [('torso', 4), ('arm_left', 7), ('arm_right', 7), ('head', 2)]
}
POSE_PARTS = ['astribot_chassis', *MODEL_JOINTS]


def robot_description():
    config = Path(__file__).resolve().parents[2] / 'astribot_config/robot_config/astribot_s1'
    root = ET.parse(config / 'model/astribot_whole_body_with_head.urdf').getroot()
    for mesh in root.findall('.//mesh'):
        path = config / 'meshes' / mesh.attrib['filename']
        if not path.is_file():
            raise FileNotFoundError(path)
        mesh.set('filename', path.resolve().as_uri())
    mount = yaml.safe_load((config / 'astribot_torso.yaml').read_text())['transform']['weld_to_base_pose']
    if len(mount) != 6 or not all(math.isfinite(v) for v in mount):
        raise ValueError('Invalid torso-to-chassis transform')
    ET.SubElement(root, 'link', name=FRAME)
    joint = ET.SubElement(root, 'joint', name='astribot_viz_base_joint', type='fixed')
    ET.SubElement(joint, 'parent', link=FRAME)
    ET.SubElement(joint, 'child', link='astribot_torso_base')
    ET.SubElement(joint, 'origin', xyz=' '.join(map(str, mount[:3])), rpy=' '.join(map(str, mount[3:])))
    add_camera_frames(root)
    return ET.tostring(root, encoding='unicode')


def joint_message(values, stamp):
    if values is None or len(values) != len(MODEL_JOINTS):
        raise ValueError('Incomplete current joint data')
    message = JointState()
    message.header.stamp = stamp
    for (part, names), row in zip(MODEL_JOINTS.items(), values):
        if row is None or len(row) != len(names) or not all(math.isfinite(v) for v in row):
            raise ValueError(f'Invalid joints for {part}')
        message.name.extend(names)
        message.position.extend(float(v) for v in row)
    return message


def pose_markers(poses, stamp, state, lifetime):
    if poses is None or len(poses) != len(POSE_PARTS):
        raise ValueError(f'Incomplete {state} Cartesian poses')
    result = MarkerArray()
    length, alpha = (0.15, 1.0) if state == 'current' else (0.10, 0.4)
    for index, (part, pose) in enumerate(zip(POSE_PARTS, poses)):
        if pose is None or len(pose) != 7 or not all(math.isfinite(v) for v in pose):
            raise ValueError(f'Invalid {state} pose for {part}')
        norm = math.hypot(*pose[3:])
        if not math.isfinite(norm) or norm < 1e-8:
            raise ValueError(f'Invalid {state} quaternion for {part}')
        for axis in range(4):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = FRAME, stamp
            marker.ns, marker.id = state, index * 4 + axis
            marker.action = Marker.ADD
            # Follow the latest chassis TF; SDK polling and LiDAR clocks are not synchronized.
            marker.frame_locked = True
            marker.lifetime = Duration(seconds=lifetime).to_msg()
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, pose[:3])
            marker.color.a = alpha
            if axis < 3:
                marker.type = Marker.ARROW
                q = marker.pose.orientation
                q.x, q.y, q.z, q.w = (float(v / norm) for v in pose[3:])
                marker.points = [Point(), Point(**{('x', 'y', 'z')[axis]: length})]
                marker.scale.x, marker.scale.y, marker.scale.z = 0.008, 0.018, 0.025
                setattr(marker.color, ('r', 'g', 'b')[axis], 1.0)
            else:
                marker.type = Marker.TEXT_VIEW_FACING
                marker.pose.orientation.w = 1.0
                marker.pose.position.z += 0.19 if state == 'current' else 0.25
                marker.scale.z = 0.035
                marker.color.r = marker.color.g = marker.color.b = 1.0
                marker.text = f'{state} {part.removeprefix("astribot_")}'
            result.markers.append(marker)
    return result
