#!/usr/bin/env python3
"""
Enhanced Security Monitor System Launch File
Includes real threat detection and sensor simulation
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Enhanced Security Detector with Real Threat Detection
        Node(
            package='security_monitor',
            executable='security_detector.py',
            name='enhanced_security_detector',
            output='screen',
            parameters=[{
                'threat_detection_rate': 2.0,
                'threat_timeout': 30.0,
                'log_level': 'info'
            }]
        ),
        
        # Sensor Data Simulator
        Node(
            package='security_monitor',
            executable='sensor_simulator.py',
            name='sensor_simulator',
            output='screen',
            parameters=[{
                'camera_rate': 10.0,
                'lidar_rate': 20.0,
                'simulation_mode': True
            }]
        ),
        
        # Optional: Add more nodes as they become available
        # Node(
        #     package='security_monitor',
        #     executable='sensor_monitor.py',
        #     name='sensor_monitor',
        #     output='screen'
        # ),
    ])
