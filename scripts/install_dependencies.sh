#!/bin/bash
# Security Monitor Dependencies Installation Script

echo "Installing Security Monitor dependencies..."

# ROS2 dependencies
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-rosdep

# Python dependencies
pip3 install numpy opencv-python
pip3 install ultralytics  # For YOLO
pip3 install torch torchvision  # For PointPillars

# CARLA dependencies (optional)
# sudo apt install -y python3-carla

# Gazebo dependencies (optional)
# sudo apt install -y ros-humble-gazebo-ros-pkgs

echo "Dependencies installation completed!"
echo "Please source your ROS2 workspace: source install/setup.bash"
