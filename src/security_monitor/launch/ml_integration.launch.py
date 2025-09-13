#!/usr/bin/env python3
"""
ML Integration Launch File
Launches the ML detector with enhanced sensor simulator.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Enhanced Sensor Simulator Node
        Node(
            package='security_monitor',
            executable='enhanced_sensor_simulator.py',
            name='enhanced_sensor_simulator',
            output='screen',
            emulate_tty=True,
        ),
        
        # ML Detector Node
        Node(
            package='security_monitor',
            executable='ml_detector.py',
            name='ml_detector',
            output='screen',
            emulate_tty=True,
        ),
        
        # Security Detector Node (for threat classification)
        Node(
            package='security_monitor',
            executable='security_detector.py',
            name='security_detector',
            output='screen',
            emulate_tty=True,
        ),
    ])
