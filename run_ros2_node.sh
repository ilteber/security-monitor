#!/bin/bash
# ROS2 Node Runner - Fixes Python version conflicts

# Deactivate conda to use system Python
conda deactivate

# Source ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run the specified ROS2 node
ros2 run security_monitor "$1"
