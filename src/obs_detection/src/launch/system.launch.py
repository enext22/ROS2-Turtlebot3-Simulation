#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    proj_pkg = get_package_share_directory('obs_detection')
    pkg_desc = get_package_share_directory('turtlebot3_description')

    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(proj_pkg, 'launch', 'start_world.launch.py'),
        )
    )
    spawn_agents = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(proj_pkg, 'launch', 'multi_turtlebot3.launch.py'),
        )
    )

    return LaunchDescription([
        start_world,
        spawn_agents
    ])