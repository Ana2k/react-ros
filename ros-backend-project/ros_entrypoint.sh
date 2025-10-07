#!/bin/bash
set -e

# Setup the ROS environment
source "/opt/ros/noetic/setup.bash"

# Execute the command passed to the script
"$@"