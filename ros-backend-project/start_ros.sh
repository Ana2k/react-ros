#!/bin/bash
set -e

# Setup ROS environment
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Start roscore in background
roscore &

# Wait for roscore to start
sleep 5

# Launch our robot simulation
roslaunch my_pkg robot_sum.launch
