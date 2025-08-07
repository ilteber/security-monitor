#!/usr/bin/env python3
"""
Data Logger Node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, PointCloud2
import json
import os
from datetime import datetime

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        
        # Logging parameters
        self.log_directory = '/tmp/security_monitor_logs'
        self.log_enabled = True
        
        # Subscribers
        self.security_alerts_sub = self.create_subscription(
            String, 'security_alerts', self.security_callback, 10)
        self.threat_level_sub = self.create_subscription(
            Float32, 'threat_level', self.threat_callback, 10)
        self.performance_sub = self.create_subscription(
            String, 'performance_status', self.performance_callback, 10)
        
        # Publishers
        self.log_status_pub = self.create_publisher(String, 'log_status', 10)
        
        # Create log directory
        os.makedirs(self.log_directory, exist_ok=True)
        
        # Timer for log status updates
        self.timer = self.create_timer(10.0, self.log_status_update)
        
        self.get_logger().info('Data logger started')

    def security_callback(self, msg):
        """Log security alerts"""
        if self.log_enabled:
            self.log_data('security_alerts', msg.data)

    def threat_callback(self, msg):
        """Log threat levels"""
        if self.log_enabled:
            self.log_data('threat_levels', str(msg.data))

    def performance_callback(self, msg):
        """Log performance status"""
        if self.log_enabled:
            self.log_data('performance', msg.data)

    def log_data(self, data_type, data):
        """Log data to file"""
        timestamp = datetime.now().isoformat()
        log_entry = {
            'timestamp': timestamp,
            'type': data_type,
            'data': data
        }
        
        log_file = os.path.join(self.log_directory, f'{data_type}.log')
        with open(log_file, 'a') as f:
            f.write(json.dumps(log_entry) + '\n')

    def log_status_update(self):
        """Update log status"""
        status_msg = String()
        status_msg.data = f"logging_enabled: {self.log_enabled}, directory: {self.log_directory}"
        self.log_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
