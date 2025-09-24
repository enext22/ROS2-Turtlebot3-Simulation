import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Define package names
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # Path to our custom world file
    world_file_path = os.path.join(get_package_share_directory('my_robot_bringup'), 
                                   'worlds', 'simple_world.world')

    # 1. Launch Gazebo
    # We are including a launch file from the gazebo_ros package
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    # 2. Get the robot's URDF file
    # The URDF is a file that describes the robot's physical properties (links, joints, etc.)
    urdf_file_path = os.path.join(pkg_turtlebot3_gazebo, 'urdf', 'turtlebot3_waffle.urdf')

    # 3. Spawn the robot in Gazebo
    # The 'spawner' node is a utility from gazebo_ros that takes a robot description
    # and spawns it in the running Gazebo simulation.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot3_waffle', '-file', urdf_file_path, '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # Create the launch description and return it
    return LaunchDescription([
        start_gazebo,
        spawn_entity
    ])