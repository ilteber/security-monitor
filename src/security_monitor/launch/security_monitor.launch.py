#!/usr/bin/env python3
"""
Launch file for security monitor
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Security Detector Node
        Node(
            package='security_monitor',
            executable='security_detector.py',
            name='security_detector',
            output='screen'
        ),
        
        # Sensor Monitor Node
        Node(
            package='security_monitor',
            executable='sensor_monitor.py',
            name='sensor_monitor',
            output='screen'
        ),
        
        # ML Detector Node
        Node(
            package='security_monitor',
            executable='ml_detector.py',
            name='ml_detector',
            output='screen'
        ),
        
        # Sensor Fusion Node
        Node(
            package='security_monitor',
            executable='sensor_fusion.py',
            name='sensor_fusion',
            output='screen'
        ),
        
        # Threat Classifier Node
        Node(
            package='security_monitor',
            executable='threat_classifier.py',
            name='threat_classifier',
            output='screen'
        ),
        
        # Emergency Handler Node
        Node(
            package='security_monitor',
            executable='emergency_handler.py',
            name='emergency_handler',
            output='screen'
        )
    ])
