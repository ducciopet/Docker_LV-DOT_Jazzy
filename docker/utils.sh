#!/bin/bash
# Utility commands for the LV-DOT workspace.
# Sourced automatically by the container's .bashrc via entrypoint.sh.

# Source the built workspace if not already done
_ws_setup=/home/ros/ros2_ws/install/setup.bash
if [ -f "$_ws_setup" ] && [ -z "$AMENT_PREFIX_PATH" ]; then
    . "$_ws_setup"
fi
unset _ws_setup

alias detection='ros2 launch onboard_detector run_detector.launch.py'
alias rebuild='cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install && source install/setup.bash'

alias bag_play='ros2 bag play bags/rosbag2_2026_03_24-12_24_08_0 --loop --clock'
