#!/usr/bin/env python3
"""
Performance Monitor Node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import psutil
import time

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        
        # Performance metrics
        self.cpu_usage = 0.0
        self.memory_usage = 0.0
        self.detection_latency = 0.0
        
        # Publishers
        self.cpu_pub = self.create_publisher(Float32, 'cpu_usage', 10)
        self.memory_pub = self.create_publisher(Float32, 'memory_usage', 10)
        self.latency_pub = self.create_publisher(Float32, 'detection_latency', 10)
        self.performance_status_pub = self.create_publisher(String, 'performance_status', 10)
        
        # Performance thresholds
        self.cpu_threshold = 80.0
        self.memory_threshold = 85.0
        self.latency_threshold = 100.0  # milliseconds
        
        # Timer for performance monitoring
        self.timer = self.create_timer(1.0, self.monitor_performance)
        
        self.get_logger().info('Performance monitor started')

    def monitor_performance(self):
        """Monitor system performance metrics"""
        # Get CPU usage
        self.cpu_usage = psutil.cpu_percent()
        
        # Get memory usage
        memory = psutil.virtual_memory()
        self.memory_usage = memory.percent
        
        # Simulate detection latency
        self.detection_latency = 45.0  # milliseconds
        
        # Publish metrics
        self.publish_metrics()
        
        # Check for performance issues
        self.check_performance_thresholds()

    def publish_metrics(self):
        """Publish performance metrics"""
        # CPU usage
        cpu_msg = Float32()
        cpu_msg.data = self.cpu_usage
        self.cpu_pub.publish(cpu_msg)
        
        # Memory usage
        memory_msg = Float32()
        memory_msg.data = self.memory_usage
        self.memory_pub.publish(memory_msg)
        
        # Detection latency
        latency_msg = Float32()
        latency_msg.data = self.detection_latency
        self.latency_pub.publish(latency_msg)

    def check_performance_thresholds(self):
        """Check if performance metrics exceed thresholds"""
        status = "normal"
        
        if self.cpu_usage > self.cpu_threshold:
            status = "high_cpu"
            self.get_logger().warn(f"High CPU usage: {self.cpu_usage:.1f}%")
        
        if self.memory_usage > self.memory_threshold:
            status = "high_memory"
            self.get_logger().warn(f"High memory usage: {self.memory_usage:.1f}%")
        
        if self.detection_latency > self.latency_threshold:
            status = "high_latency"
            self.get_logger().warn(f"High detection latency: {self.detection_latency:.1f}ms")
        
        # Publish performance status
        status_msg = String()
        status_msg.data = status
        self.performance_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PerformanceMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
