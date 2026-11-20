"""One RViz for rear FAST-LIO and the articulated robot, without SDK access."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import numpy as np
from scipy.spatial.transform import Rotation

SCRIPT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPT_DIR.parent))
from astribot_scripts.common.fast_lio_viz import rear_imu_from_sdk_chassis
from astribot_scripts.common.state import FRAME
from astribot_scripts.common.apriltag_launch import arguments as apriltag_arguments, configure as configure_apriltags


def generate_launch_description():
    mount = np.linalg.inv(rear_imu_from_sdk_chassis())
    quaternion = Rotation.from_matrix(mount[:3, :3]).as_quat()
    values = [*mount[:3, 3], *quaternion]
    mount_args = [arg for key, value in zip(
        ['--x', '--y', '--z', '--qx', '--qy', '--qz', '--qw'], values)
        for arg in (key, str(float(value)))]
    tf_remaps = [('/tf', '/astribot_viz/tf'), ('/tf_static', '/astribot_viz/tf_static')]
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=str(SCRIPT_DIR / 'config/astribot_fast_lio.rviz')),
        DeclareLaunchArgument('rgbd_clouds', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rgbd_config', default_value=str(SCRIPT_DIR / 'config/rgbd_pointcloud.yaml')),
        DeclareLaunchArgument('rgbd_tf_time', default_value='latest', choices=['latest', 'measurement']),
        *apriltag_arguments(),
        OpaqueFunction(function=configure_apriltags),
        ExecuteProcess(cmd=[sys.executable, '-m', 'astribot_scripts.rgbd_pointcloud',
                            '--ros-args', '-p', ['config:=', LaunchConfiguration('rgbd_config')],
                            '-p', ['tf_time_mode:=', LaunchConfiguration('rgbd_tf_time')],
                            '-p', ['use_sim_time:=', LaunchConfiguration('use_sim_time')],
                            '-r', '/tf:=/astribot_viz/tf', '-r', '/tf_static:=/astribot_viz/tf_static'],
                       cwd=str(SCRIPT_DIR.parent), output='screen',
                       condition=IfCondition(LaunchConfiguration('rgbd_clouds'))),
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(SCRIPT_DIR / 'launch/astribot_rviz.launch.py')),
            launch_arguments={'rviz': 'false', 'publish_state': 'false', 'odometry': 'false',
                              'use_sim_time': LaunchConfiguration('use_sim_time')}.items())]),
        ExecuteProcess(cmd=[sys.executable, str(SCRIPT_DIR / 'fast_lio_display_tf.py'),
                            '--ros-args', '-r', '/tf:=/astribot_viz/tf',
                            '-r', '/tf_static:=/astribot_viz/tf_static',
                            '-p', ['use_sim_time:=', LaunchConfiguration('use_sim_time')]], output='screen'),
        # Robot chassis is the dynamic base; the IMU is its fixed child.
        # The source LIO tree remains separate on /fast_lio/rear/tf.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='sdk_chassis_to_rear_imu', output='screen',
             arguments=[*mount_args, '--frame-id', FRAME, '--child-frame-id', 'body'],
             parameters=[{'use_sim_time': sim_time}], remappings=tf_remaps),
        Node(package='rviz2', executable='rviz2', name='astribot_lio_rviz', output='screen',
             arguments=['-d', LaunchConfiguration('rviz_config'), '-f', 'map'],
             parameters=[{'use_sim_time': sim_time}], remappings=tf_remaps,
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
