"""Nav2 Humble local navigation alongside the existing rear FAST-LIO stack."""
import os
import math
from pathlib import Path
import sys
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

SCRIPT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPT_DIR.parent))
from astribot_scripts.common.nav2 import merge_parameters


def configure(context):
    if os.environ.get('ROS_DISTRO') != 'humble':
        raise RuntimeError('This launch targets ROS 2 Humble; source /opt/ros/humble/setup.bash')
    value = lambda name: LaunchConfiguration(name).perform(context)
    offset = float(value('stamp_offset'))
    if not math.isfinite(offset):
        raise ValueError('stamp_offset must be finite')
    enabled, simulated = value('enable_control') == 'true', value('use_sim_time') == 'true'
    if enabled and simulated:
        raise RuntimeError('Bag/simulation preview requires enable_control:=false; no hardware output under /clock')
    bringup = Path(get_package_share_directory('nav2_bringup'))
    defaults = yaml.safe_load((bringup / 'params/nav2_params.yaml').read_text())
    overrides = yaml.safe_load(Path(value('params_file')).read_text())
    parameters = merge_parameters(defaults, overrides)
    with tempfile.NamedTemporaryFile(mode='w', prefix='astribot_nav2_', suffix='.yaml', delete=False) as output:
        yaml.safe_dump(parameters, output)
        generated = output.name

    def cleanup(context):
        Path(generated).unlink(missing_ok=True)
        return []

    bridge_command = [sys.executable, '-m', 'astribot_scripts.nav2_bridge', '--ros-args',
                      '-r', '/tf:=/astribot_nav/tf', '-r', '/tf_static:=/astribot_nav/tf_static',
                      '-p', f'stamp_offset:={offset}',
                      '-p', f'enable_output:={str(enabled).lower()}',
                      '-p', f'use_sim_time:={str(simulated).lower()}']
    if value('mount_file'):
        bridge_command += ['-p', f'mount_file:={value("mount_file")}']
    bridge = ExecuteProcess(
        cmd=bridge_command,
        cwd=str(SCRIPT_DIR.parent), output='screen')
    actions = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        RegisterEventHandler(OnProcessExit(target_action=bridge, on_exit=[
            EmitEvent(event=Shutdown(reason='Nav2 feedback bridge exited'))])),
        bridge,
        GroupAction([
            PushRosNamespace('astribot_nav'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(bringup / 'launch/navigation_launch.py')),
                launch_arguments={'namespace': 'astribot_nav', 'use_composition': 'False',
                                  'use_respawn': 'False', 'autostart': 'true',
                                  'use_sim_time': str(simulated).lower(), 'params_file': generated}.items())]),
    ]
    if enabled:
        command = [sys.executable, '-m', 'astribot_scripts.control_mobile_base',
                   '--topic', '/astribot/nav_cmd_vel', '--timeout', '0.3',
                   '--max-linear-speed', '0.2', '--max-angular-speed', '0.4']
        if value('with_joints') == 'true':
            command.append('--with-joints')
        receiver = ExecuteProcess(cmd=command, cwd=str(SCRIPT_DIR.parent), output='screen')
        actions += [RegisterEventHandler(OnProcessExit(target_action=receiver, on_exit=[
            EmitEvent(event=Shutdown(reason='SDK command receiver exited'))])), receiver]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_control', default_value='true', choices=['true', 'false'],
                              description='Start the SDK receiver and enable gated physical velocity output'),
        DeclareLaunchArgument('with_joints', default_value='false', choices=['true', 'false'],
                              description='Also receive /astribot/joint_cmd using the same SDK owner'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('stamp_offset', default_value='0.0',
                              description='Verified ROS clock minus sensor clock offset in seconds; never latency compensation'),
        DeclareLaunchArgument('mount_file', default_value='',
                              description='Optional JSON base_from_imu and imu_from_lidar 4x4 calibration matrices'),
        DeclareLaunchArgument('params_file', default_value=str(SCRIPT_DIR / 'config/nav2_humble.yaml'),
                              description='Robot-specific overrides for installed Nav2 Humble defaults'),
        OpaqueFunction(function=configure),
    ])
