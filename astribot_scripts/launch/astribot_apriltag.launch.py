"""Run the existing AprilTag detector and show its tag axes in combined RViz."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from astribot_scripts.common.apriltag_launch import arguments, configure


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        *arguments(default='true'),
        OpaqueFunction(function=configure),
    ])
