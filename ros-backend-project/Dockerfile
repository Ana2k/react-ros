# Use the official ROS Noetic base image
FROM ros:noetic-robot

# Set up the working directory for our ROS workspace
WORKDIR /catkin_ws

# Install rosbridge server, which allows web clients to communicate with ROS
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# Copy the entrypoint script and make it executable
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Create a ROS package to hold our nodes and launch files
# We add geometry_msgs because our commands will use it
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash; catkin_create_pkg my_pkg std_msgs rospy geometry_msgs"

# Copy our new application files into the workspace
COPY src/my_pkg /catkin_ws/src/my_pkg

# Make our python script executable
RUN chmod +x /catkin_ws/src/my_pkg/scripts/robot_sim_node.py

# Build the workspace so our package is recognized by ROS
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make"

# Source the new setup file in the entrypoint so ROS can find our package
RUN echo "source /catkin_ws/devel/setup.bash" >> /ros_entrypoint.sh