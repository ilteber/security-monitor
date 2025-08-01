#!/usr/bin/env python3
"""
Weather Condition Monitor Node
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
import numpy as np

class WeatherMonitor(Node):
    def __init__(self):
        super().__init__('weather_monitor')
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        
        # Publishers
        self.weather_status_pub = self.create_publisher(String, 'weather_status', 10)
        self.visibility_pub = self.create_publisher(Float32, 'visibility_range', 10)
        self.weather_alert_pub = self.create_publisher(String, 'weather_alert', 10)
        
        # Weather detection parameters
        self.weather_conditions = ['clear', 'rain', 'fog', 'snow', 'storm']
        self.current_weather = 'clear'
        self.visibility_range = 100.0  # meters
        
        # Weather thresholds
        self.low_visibility_threshold = 50.0
        self.severe_weather_threshold = 0.7
        
        self.get_logger().info('Weather monitor started')

    def camera_callback(self, msg):
        """Analyze camera data for weather conditions"""
        # TODO: Implement actual weather detection using computer vision
        # For now, simulate weather detection
        self.simulate_weather_detection()

    def simulate_weather_detection(self):
        """Simulate weather condition detection"""
        # Simulate weather analysis
        weather_confidence = np.random.random()
        
        if weather_confidence > 0.8:
            self.current_weather = 'clear'
            self.visibility_range = 100.0
        elif weather_confidence > 0.6:
            self.current_weather = 'rain'
            self.visibility_range = 75.0
        elif weather_confidence > 0.4:
            self.current_weather = 'fog'
            self.visibility_range = 30.0
        else:
            self.current_weather = 'storm'
            self.visibility_range = 20.0
        
        # Publish weather status
        status_msg = String()
        status_msg.data = self.current_weather
        self.weather_status_pub.publish(status_msg)
        
        # Publish visibility range
        vis_msg = Float32()
        vis_msg.data = self.visibility_range
        self.visibility_pub.publish(vis_msg)
        
        # Check for weather alerts
        if self.visibility_range < self.low_visibility_threshold:
            alert_msg = String()
            alert_msg.data = f"Low visibility: {self.visibility_range:.1f}m"
            self.weather_alert_pub.publish(alert_msg)
            self.get_logger().warn(f"Weather alert: {alert_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = WeatherMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
