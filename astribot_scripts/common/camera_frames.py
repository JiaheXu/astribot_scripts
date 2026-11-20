"""Add nominal optical camera frames to the generated visualization URDF."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
from scipy.spatial.transform import Rotation


def camera_mounts():
    path = Path(__file__).resolve().parents[1] / 'config/camera_frames.yaml'
    return yaml.safe_load(path.read_text())['cameras']


def optical_rpy(quaternion_wxyz):
    if len(quaternion_wxyz) != 4 or not all(math.isfinite(v) for v in quaternion_wxyz):
        raise ValueError('Invalid camera quaternion')
    norm = math.hypot(*quaternion_wxyz)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid camera quaternion')
    w, x, y, z = (v / norm for v in quaternion_wxyz)
    # q_mujoco * q_Rx(pi): flips local Y/Z, not the parent link's axes.
    return Rotation.from_quat([w, z, -y, -x]).as_euler('xyz').tolist()


def add_camera_frames(root):
    links = {link.get('name') for link in root.findall('link')}
    for name, mount in camera_mounts().items():
        parent, position = mount['parent'], mount['position']
        child = f'astribot_viz_{name}_optical'
        if parent not in links or child in links:
            raise ValueError(f'Camera {name}: missing parent or duplicate frame')
        if len(position) != 3 or not all(math.isfinite(v) for v in position):
            raise ValueError(f'Camera {name}: invalid position')
        rpy = optical_rpy(mount['quaternion_wxyz'])
        ET.SubElement(root, 'link', name=child)
        joint = ET.SubElement(root, 'joint', name=f'{child}_joint', type='fixed')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=child)
        ET.SubElement(joint, 'origin', xyz=' '.join(map(str, position)), rpy=' '.join(map(str, rpy)))
        links.add(child)
