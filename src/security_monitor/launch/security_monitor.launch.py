#!/usr/bin/env python3
"""
Launch file for security monitor
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='security_monitor',
            executable='security_detector.py',
            name='security_detector',
            output='screen'
        )
    ])
