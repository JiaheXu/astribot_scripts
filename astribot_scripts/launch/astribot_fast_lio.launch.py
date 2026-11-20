"""Launch patched FAST-LIO with the rear pair by default; no driver startup."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SCRIPT_DIR = Path(__file__).resolve().parents[1]


def launch_sensor(context):
    sensor = LaunchConfiguration('sensor').perform(context)
    namespace = f'fast_lio/{sensor}'
    config = LaunchConfiguration('config').perform(context)
    if not config:
        config = str(SCRIPT_DIR / f'config/fast_lio_{sensor}.yaml')
    if not Path(config).is_file():
        raise RuntimeError(f'FAST-LIO configuration not found: {config}')
    # Profiles select map as the world frame; body remains the IMU. Keep TF on its
    # own topics so it cannot compete with the robot or the KISS visualization.
    tf_remaps = [('/tf', f'/{namespace}/tf'),
                 ('/tf_static', f'/{namespace}/tf_static')]
    outputs = [('/Odometry', f'/{namespace}/odometry'),
               ('/path', f'/{namespace}/path'),
               ('/cloud_registered', f'/{namespace}/cloud_registered'),
               ('/cloud_registered_body', f'/{namespace}/cloud_registered_body'),
               ('/cloud_effected', f'/{namespace}/cloud_effected'),
               ('/Laser_map', f'/{namespace}/map')]
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return [
        Node(package='fast_lio', executable='fastlio_mapping',
             namespace=namespace, name='mapping', output='screen',
             parameters=[config, {'use_sim_time': sim_time}],
             remappings=outputs + tf_remaps),
        Node(package='rviz2', executable='rviz2', name=f'fast_lio_{sensor}_rviz',
             arguments=['-d', str(Path(get_package_share_directory('fast_lio')) / 'rviz/fastlio.rviz'),
                        '-f', 'map'],
             parameters=[{'use_sim_time': sim_time}], remappings=outputs + tf_remaps,
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sensor', default_value='rear', choices=['rear', 'front'],
                              description='Lidar and its own IMU; rear uses nominal model cloud correction'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('config', default_value='',
                              description='Optional profile override; otherwise selected by sensor'),
        OpaqueFunction(function=launch_sensor),
    ])
