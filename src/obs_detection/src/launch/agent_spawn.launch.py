from launch import LaunchDescription

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='obs_detection',
            executable='agent_spawn.py',
            output='screen',
            arguments=[
                '--agent_urdf', LaunchConfiguration('agent_urdf'),
                '--agent_name', LaunchConfiguration('agent_name'),
                '--agent_namespace', LaunchConfiguration('agent_namespace'),
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
            ]),
    ])