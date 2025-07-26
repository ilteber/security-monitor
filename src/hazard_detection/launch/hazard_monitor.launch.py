#!/usr/bin/env python3
"""
Launch file for hazard monitor
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hazard_detection',
            executable='hazard_detector.py',
            name='hazard_detector',
            output='screen'
        )
    ])
