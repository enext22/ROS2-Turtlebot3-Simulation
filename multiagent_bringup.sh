#!/bin/bash

source /opt/ros/jazzy/setup.bash
source ~/Documents/ros_ws/install/setup.bash

export TURTLEBOT3_MODEL=burger

colcon build

#ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py namespace:=/robot1
#ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py namespace:=/robot2
