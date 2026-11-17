"""Launch directly by file path; no colcon build or SDK modifications needed."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from rviz_state import robot_description


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
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='SDK visualization publisher stopped'))])),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='astribot_viz', output='screen',
             parameters=[{'robot_description': ParameterValue(robot_description(), value_type=str)}],
             remappings=tf_remaps),
        bridge,
        Node(package='rviz2', executable='rviz2', name='astribot_rviz', output='screen',
             arguments=['-d', str(SCRIPT_DIR / 'astribot_poses.rviz')],
             remappings=tf_remaps, condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
