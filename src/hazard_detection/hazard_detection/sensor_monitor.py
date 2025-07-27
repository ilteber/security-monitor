#!/usr/bin/env python3
"""
Sensor Monitor Node
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String

class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar/points', self.lidar_callback, 10)
        
        # Publisher
        self.sensor_status_pub = self.create_publisher(String, 'sensor_status', 10)
        
        self.get_logger().info('Sensor monitor started')

    def camera_callback(self, msg):
        self.get_logger().debug('Received camera data')

    def lidar_callback(self, msg):
        self.get_logger().debug('Received LiDAR data')

def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
