#!/bin/bash

# Source ROS 2 setup
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Initialize and update rosdep (run once)
if [ ! -d /home/ros/.ros/rosdep ]; then
    rosdep update
fi

# Source workspace setup if it exists
if [ -f /home/ros/ros2_ws/install/setup.bash ]; then
    source /home/ros/ros2_ws/install/setup.bash
fi

# Execute the command passed to the container
exec "$@"