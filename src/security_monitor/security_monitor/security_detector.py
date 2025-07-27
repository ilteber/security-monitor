#!/usr/bin/env python3
"""
Security Detector Node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SecurityDetector(Node):
    def __init__(self):
        super().__init__('security_detector')
        self.publisher = self.create_publisher(String, 'security_alerts', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Security detector started')

    def timer_callback(self):
        msg = String()
        msg.data = 'No security threats detected'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SecurityDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
