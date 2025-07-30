#!/usr/bin/env python3
"""
Sensor Fusion Node
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String, Float32
import numpy as np

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar/points', self.lidar_callback, 10)
        self.ml_detections_sub = self.create_subscription(
            PoseArray, 'ml_detections', self.ml_detections_callback, 10)
        
        # Publishers
        self.fused_detections_pub = self.create_publisher(PoseArray, 'fused_detections', 10)
        self.fusion_confidence_pub = self.create_publisher(Float32, 'fusion_confidence', 10)
        self.fusion_status_pub = self.create_publisher(String, 'fusion_status', 10)
        
        # Fusion parameters
        self.camera_data = None
        self.lidar_data = None
        self.ml_detections = None
        self.fusion_enabled = True
        
        self.get_logger().info('Sensor fusion started')

    def camera_callback(self, msg):
        """Store camera data for fusion"""
        self.camera_data = msg
        self.perform_fusion()

    def lidar_callback(self, msg):
        """Store LiDAR data for fusion"""
        self.lidar_data = msg
        self.perform_fusion()

    def ml_detections_callback(self, msg):
        """Store ML detection results"""
        self.ml_detections = msg
        self.perform_fusion()

    def perform_fusion(self):
        """Perform sensor fusion when data is available"""
        if not self.fusion_enabled:
            return
            
        if self.camera_data is None or self.lidar_data is None:
            return
            
        # TODO: Implement actual sensor fusion algorithm
        # For now, simulate fusion
        self.simulate_fusion()

    def simulate_fusion(self):
        """Simulate sensor fusion results"""
        # Simulate fusion confidence
        confidence = 0.85
        
        conf_msg = Float32()
        conf_msg.data = confidence
        self.fusion_confidence_pub.publish(conf_msg)
        
        # Simulate fusion status
        status_msg = String()
        status_msg.data = "fusion_active"
        self.fusion_status_pub.publish(status_msg)
        
        # Create fused detections
        fused_detections = PoseArray()
        fused_detections.header.stamp = self.get_clock().now().to_msg()
        fused_detections.header.frame_id = "base_link"
        
        self.fused_detections_pub.publish(fused_detections)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
