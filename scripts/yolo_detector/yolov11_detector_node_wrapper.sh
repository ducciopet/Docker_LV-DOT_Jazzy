#!/bin/bash
# Wrapper script to run yolov11_detector_node.py with proper ROS 2 environment

# Source ROS 2 setup if available
if [ -n "$ROS_DISTRO" ]; then
    # ROS environment is already set up in the parent process
    # Just ensure PYTHONPATH includes the scripts directory
    export PYTHONPATH="${PYTHONPATH}:$(dirname "$0")"
fi

# Run the Python script
exec $(dirname "$0")/yolov11_detector_node.py "$@"
