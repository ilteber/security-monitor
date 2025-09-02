#!/usr/bin/env python3

"""
Launch file for Security Monitor API Server
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'host',
            default_value='0.0.0.0',
            description='API server host address'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='8000',
            description='API server port'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for API server'
        ),
        
        Node(
            package='security_monitor',
            executable='api/server.py',
            name='security_monitor_api',
            output='screen',
            parameters=[
                {'host': LaunchConfiguration('host')},
                {'port': LaunchConfiguration('port')},
                {'log_level': LaunchConfiguration('log_level')}
            ],
            remappings=[
                ('/vehicle/control', '/cmd_vel'),
                ('/api/status', '/security_monitor/status')
            ]
        )
    ])
