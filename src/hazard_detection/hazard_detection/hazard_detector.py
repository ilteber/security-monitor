#!/usr/bin/env python3
"""
Basic Hazard Detector Node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HazardDetector(Node):
    def __init__(self):
        super().__init__('hazard_detector')
        self.publisher = self.create_publisher(String, 'hazards', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Hazard detector started')

    def timer_callback(self):
        msg = String()
        msg.data = 'No hazards detected'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HazardDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
