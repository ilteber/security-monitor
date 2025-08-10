#!/usr/bin/env python3
"""
Complete Security Monitor System Launch File
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='config/security_config.yaml',
        description='Path to configuration file'
    )
    
    return LaunchDescription([
        config_file_arg,
        
        # Core Security System
        Node(
            package='security_monitor',
            executable='security_detector.py',
            name='security_detector',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),
        
        # Sensor Monitoring
        Node(
            package='security_monitor',
            executable='sensor_monitor.py',
            name='sensor_monitor',
            output='screen'
        ),
        
        # ML Detection
        Node(
            package='security_monitor',
            executable='ml_detector.py',
            name='ml_detector',
            output='screen'
        ),
        
        # Sensor Fusion
        Node(
            package='security_monitor',
            executable='sensor_fusion.py',
            name='sensor_fusion',
            output='screen'
        ),
        
        # Threat Classification
        Node(
            package='security_monitor',
            executable='threat_classifier.py',
            name='threat_classifier',
            output='screen'
        ),
        
        # Emergency Handling
        Node(
            package='security_monitor',
            executable='emergency_handler.py',
            name='emergency_handler',
            output='screen'
        ),
        
        # Weather Monitoring
        Node(
            package='security_monitor',
            executable='weather_monitor.py',
            name='weather_monitor',
            output='screen'
        ),
        
        # Model Management
        Node(
            package='security_monitor',
            executable='model_manager.py',
            name='model_manager',
            output='screen'
        ),
        
        # Performance Monitoring
        Node(
            package='security_monitor',
            executable='performance_monitor.py',
            name='performance_monitor',
            output='screen'
        ),
        
        # Data Logging
        Node(
            package='security_monitor',
            executable='data_logger.py',
            name='data_logger',
            output='screen'
        ),
        
        # Health Monitoring
        Node(
            package='security_monitor',
            executable='health_monitor.py',
            name='health_monitor',
            output='screen'
        )
    ])
