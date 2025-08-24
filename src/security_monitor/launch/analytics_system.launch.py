#!/usr/bin/env python3
"""
Analytics and Notification System Launch File
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Analytics Node
        Node(
            package='security_monitor',
            executable='analytics.py',
            name='analytics',
            output='screen'
        ),
        
        # Notification System Node
        Node(
            package='security_monitor',
            executable='notification_system.py',
            name='notification_system',
            output='screen'
        ),
        
        # Error Handler Node
        Node(
            package='security_monitor',
            executable='error_handler.py',
            name='error_handler',
            output='screen'
        ),
        
        # System Optimizer Node
        Node(
            package='security_monitor',
            executable='optimizer.py',
            name='system_optimizer',
            output='screen'
        )
    ])
