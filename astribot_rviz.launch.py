"""Launch directly by file path; no colcon build or SDK modifications needed."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from rviz_state import robot_description


def configure_odometry(context):
    enabled = LaunchConfiguration('odometry').perform(context) == 'true'
    actions = [SetLaunchConfiguration('fixed_frame', 'kiss_rear_odom' if enabled else 'astribot_viz_chassis')]
    if enabled:
        from rear_lidar_odometry import load_calibration
        calibration = LaunchConfiguration('calibration').perform(context)
        load_calibration(calibration)  # Fail before connecting the SDK if calibration is absent.
        actions.append(ExecuteProcess(
            cmd=[sys.executable, str(SCRIPT_DIR / 'rear_lidar_odometry.py'),
                 '--calibration', calibration, '--ros-args', '-p',
                 ['use_sim_time:=', LaunchConfiguration('use_sim_time')]], output='screen'))
    return actions


def generate_launch_description():
    tf_remaps = [('/tf', '/astribot_viz/tf'), ('/tf_static', '/astribot_viz/tf_static')]
    bridge = ExecuteProcess(
        cmd=[sys.executable, str(SCRIPT_DIR / 'rviz_state.py'), '--rate', LaunchConfiguration('rate')],
        output='screen', condition=IfCondition(LaunchConfiguration('publish_state')))
    return LaunchDescription([
        DeclareLaunchArgument('rate', default_value='10', description='SDK state polling Hz'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz GUI'),
        DeclareLaunchArgument('publish_state', default_value='true',
                              description='Connect to SDK (false when rviz_state.py runs on another PC)'),
        DeclareLaunchArgument('odometry', default_value='false', choices=['true', 'false'],
                              description='Convert rear KISS poses and move the model in kiss_rear_odom'),
        DeclareLaunchArgument('calibration', default_value=str(SCRIPT_DIR / 'rear_lidar_to_base.yaml'),
                              description='Calibrated cloud-to-SDK-chassis transform YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use bag clock for model, odometry and RViz (publish_state:=false for replay)'),
        OpaqueFunction(function=configure_odometry),
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='SDK visualization publisher stopped'))])),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='astribot_viz', output='screen',
             parameters=[{'robot_description': ParameterValue(robot_description(), value_type=str),
                          'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
             remappings=tf_remaps),
        bridge,
        Node(package='rviz2', executable='rviz2', name='astribot_rviz', output='screen',
             arguments=['-d', str(SCRIPT_DIR / 'astribot_poses.rviz'), '-f', LaunchConfiguration('fixed_frame')],
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
             remappings=tf_remaps, condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
