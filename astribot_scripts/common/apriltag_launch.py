"""Optional AprilTag detector and axes for the combined RViz launch."""
import importlib.util
import os
from pathlib import Path
import sys

import yaml

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SCRIPT_DIR = Path(__file__).resolve().parents[1]


def arguments(default='auto'):
    return [
        DeclareLaunchArgument('apriltags', default_value=default, choices=['auto', 'true', 'false'],
                              description='Detect tags and show axes; auto starts when the package is available'),
        DeclareLaunchArgument('apriltag_root', default_value=str(SCRIPT_DIR.parent.parent / 'apriltag_utils'),
                              description='Source package fallback if apriltag_utils is not installed'),
        DeclareLaunchArgument('apriltag_cameras', default_value='', description='Override detector camera YAML'),
        DeclareLaunchArgument('apriltag_tags', default_value='', description='Override tag family/IDs/sizes YAML'),
        DeclareLaunchArgument('apriltag_tf_time', default_value='latest', choices=['latest', 'measurement'],
                              description='Latest display TF, or image-time TF with synchronized clocks'),
        DeclareLaunchArgument('apriltag_axis_length', default_value='0.1', description='Tag axis length in meters'),
        DeclareLaunchArgument('apriltag_start_detector', default_value='true', choices=['true', 'false'],
                              description='False to visualize an already running detector without duplicating it'),
    ]


def configure(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    if value('apriltags') == 'false':
        return []
    source_root = Path(value('apriltag_root')).expanduser()
    start_detector = value('apriltag_start_detector') == 'true'
    installed = False
    try:
        package_root = Path(get_package_share_directory('apriltag_utils'))
        installed = True
    except PackageNotFoundError:
        package_root = source_root
        tags_path = Path(value('apriltag_tags') or package_root / 'config/tags.yaml')
        family = (yaml.safe_load(tags_path.read_text()) or {}).get('family', 'tag36h11') if tags_path.is_file() else 'tag36h11'
        dependencies = ['vision_msgs'] + (['cv_bridge', 'cv2'] if start_detector else [])
        if start_detector and not str(family).startswith('DICT_'):
            dependencies.append('pupil_apriltags')
        missing = [name for name in dependencies if importlib.util.find_spec(name) is None]
        source_missing = start_detector and not (package_root / 'apriltag_utils/node.py').is_file()
        if source_missing or missing:
            message = ('AprilTag axes are unavailable: install/source apriltag_utils, or set apriltag_root '
                       f'to its source and install its dependencies (missing: {missing}). '
                       'See astribot_scripts/docs/apriltags.md.')
            if value('apriltags') == 'auto':
                return [LogInfo(msg=message)]
            raise RuntimeError(message)
    cameras = value('apriltag_cameras') or str(package_root / 'config/cameras.yaml')
    tags = value('apriltag_tags') or str(package_root / 'config/tags.yaml')
    for path in (cameras, tags):
        if not Path(path).is_file():
            raise RuntimeError(f'AprilTag configuration not found: {path}')
    simulation = value('use_sim_time') == 'true'
    actions = []
    if start_detector:
        if installed:
            actions.append(Node(
                package='apriltag_utils', executable='apriltag_node', name='apriltag_detector', output='screen',
                parameters=[{'cameras_config': cameras, 'tags_config': tags, 'publish_tf': False,
                             'use_sim_time': simulation}]))
        else:
            actions.append(ExecuteProcess(
                cmd=[sys.executable, '-c', 'from apriltag_utils.node import main; main()', '--ros-args',
                     '-p', f'cameras_config:={cameras}', '-p', f'tags_config:={tags}',
                     '-p', 'publish_tf:=false', '-p', f'use_sim_time:={str(simulation).lower()}'],
                additional_env={'PYTHONPATH': str(source_root) + os.pathsep + os.environ.get('PYTHONPATH', '')},
                cwd=str(SCRIPT_DIR.parent), output='screen'))
    actions.append(ExecuteProcess(
        cmd=[sys.executable, '-m', 'astribot_scripts.apriltag_rviz', '--ros-args',
             '-p', f'cameras_config:={cameras}', '-p', f'tags_config:={tags}',
             '-p', f'tf_time_mode:={value("apriltag_tf_time")}',
             '-p', f'axis_length:={float(value("apriltag_axis_length"))}',
             '-p', f'use_sim_time:={str(simulation).lower()}',
             '-r', '/tf:=/astribot_viz/tf', '-r', '/tf_static:=/astribot_viz/tf_static'],
        cwd=str(SCRIPT_DIR.parent), output='screen'))
    return actions
