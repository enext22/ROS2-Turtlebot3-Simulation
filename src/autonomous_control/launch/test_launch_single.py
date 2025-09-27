import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushROSNamespace

def generate_launch_description():
    
    # Declare arguments
    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value='burger',
        description='Turtlebot3 Description'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='agent_1',
        description='Robot Namespace'
    )

    # Get package directories
    turtlebot3_gazebo_share_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_bringup_share_dir = get_package_share_directory('turtlebot3_bringup')

    # Include the robot state publisher launch file
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_bringup_share_dir, 'launch', 'turtlebot3_state_publisher.launch.py')
        ),
        launch_arguments={'frame_prefix': 'turtle1', 'use_sim_time': 'true', 'model': LaunchConfiguration('model')}.items()
    )
    robot_state_publisher_launch_with_namespace = GroupAction(
        actions=[
            PushROSNamespace('turtle1'),
            robot_state_publisher_launch
        ]
    )

    robot_state_publisher_launch_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_bringup_share_dir, 'launch', 'turtlebot3_state_publisher.launch.py')
        ),
        launch_arguments={'frame_prefix': 'turtle2', 'use_sim_time': 'true', 'model': LaunchConfiguration('model')}.items()
    )
    robot_state_publisher_launch_2_with_namespace = GroupAction(
        actions=[
            PushROSNamespace('turtle2'),
            robot_state_publisher_launch_2
        ]
    )


    # Include Gazebo world Launch File
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_share_dir, 'launch', 'empty_world.launch.py')
        )#,
        #launch_arguments={'namespace': 'turtle1'}.items()
    )
    gazebo_launch_with_namespace = GroupAction(
        actions=[
            PushROSNamespace('agent1'),
            gazebo_launch,
        ]
    )

    # Spawn the Turtlebot3 model
    spawn_turtlebot3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_share_dir, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'model': LaunchConfiguration('model'), 'x_pos': '0.0', 'y_pos': '0,0', 'z_pos': '0.0'}.items()
    )
    turtlebot3_with_namespace = GroupAction(
        actions=[
            PushROSNamespace('turtle1'),
            spawn_turtlebot3_launch,
        ]
    )
    spawn_turtlebot3_launch_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_share_dir, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'model': LaunchConfiguration('model'), 'x_pos': '1.0', 'y_pos': '1,0', 'z_pos': '0.0'}.items()
    )
    turtlebot3_with_namespace_2 = GroupAction(
        actions=[
            PushROSNamespace('turtle2'),
            spawn_turtlebot3_launch_2
        ]
    )

    return LaunchDescription([
        declare_model_arg,
        namespace_arg,
        gazebo_launch,

        robot_state_publisher_launch_with_namespace,
        turtlebot3_with_namespace,
        
        robot_state_publisher_launch_2_with_namespace,
        turtlebot3_with_namespace_2
    ])