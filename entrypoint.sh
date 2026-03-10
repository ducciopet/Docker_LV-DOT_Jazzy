#!/bin/bash

# Source ROS 2 setup
. /opt/ros/jazzy/setup.bash

# Initialize and update rosdep (run once)
if [ ! -d /home/ros/.ros/rosdep ]; then
    rosdep update
fi

# Install ROS dependencies if workspace source directory exists
if [ -d /home/ros/ros2_ws/src ]; then
    cd /home/ros/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y || true
fi

# Source workspace setup if it exists
if [ -f /home/ros/ros2_ws/install/setup.bash ]; then
    . /home/ros/ros2_ws/install/setup.bash
fi

# Execute the command passed to the container
exec "$@"