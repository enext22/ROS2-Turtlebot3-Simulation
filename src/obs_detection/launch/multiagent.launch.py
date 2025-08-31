# ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import LaunchConfiguration, FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os

def generate_agent_list(num_agents):
    agent_list = []

    for i in range(1, num_agents+1):
        agent_name = "turtlebot"+str(i)
        x_pose = float(i)
        agent_list.append({'name': agent_name, 'x_pose': x_pos, 'y_pose': 0.5, 'z_pose': 0.1})

    return agent_list

def generate_launch_description():
    
    bringup_dir = get_package_share_directory('ros_ws')
    launch_dir = os.path.join(bringup_dir, 'launch')

    agents = generate_agent_list(3)

    # assign simulation settings
    world = LaunchConfiguration('world')
    simulator = LaunchConfiguration('simulator')

    # declare launch arguments


    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    example_pkg_path = FindPackageShare('obs_detection')  # Replace with your own package name
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    return LaunchDescription([
        SetEnvironmentVariable(
            'TUTLEBOT3_MODEL',
            "burger"
        ),

        # Bridging and remapping Gazebo topics to ROS 2 (replace with your own topics)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/example_imu_topic@sensor_msgs/msg/Imu@gz.msgs.IMU',],
            remappings=[('/example_imu_topic',
                         '/remapped_imu_topic'),],
            output='screen'
        ),
    ])