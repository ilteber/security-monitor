#!/usr/bin/env python3
"""
Security Monitor System Viewer
Displays real-time threat detection and sensor data
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        
        # Subscribers
        self.security_sub = self.create_subscription(
            String, 'security_alerts', self.security_callback, 10)
        self.threat_sub = self.create_subscription(
            String, 'threat_details', self.threat_callback, 10)
        self.camera_sub = self.create_subscription(
            String, 'camera_data', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(
            String, 'lidar_data', self.lidar_callback, 10)
        
        self.get_logger().info('System monitor started - watching for threats...')

    def security_callback(self, msg):
        """Handle security alerts"""
        print(f"\n🚨 SECURITY ALERT: {msg.data}")

    def threat_callback(self, msg):
        """Handle detailed threat information"""
        try:
            threat = json.loads(msg.data)
            print(f"\n⚠️  THREAT DETAILS:")
            print(f"   Type: {threat['type']}")
            print(f"   Severity: {threat['severity']}")
            print(f"   Confidence: {threat['confidence']}")
            print(f"   Description: {threat['description']}")
            print(f"   Location: ({threat['location']['x']}, {threat['location']['y']}, {threat['location']['z']})")
        except json.JSONDecodeError:
            print(f"Raw threat data: {msg.data}")

    def camera_callback(self, msg):
        """Handle camera data"""
        try:
            data = json.loads(msg.data)
            if data['objects']:
                print(f"\n📷 CAMERA: Detected {len(data['objects'])} objects")
                for obj in data['objects']:
                    print(f"   - {obj['type']} at {obj['position']} (conf: {obj['confidence']:.2f})")
        except:
            pass

    def lidar_callback(self, msg):
        """Handle LiDAR data"""
        try:
            data = json.loads(msg.data)
            print(f"\n📡 LIDAR: {len(data['points'])} points, health: {data['sensor_health']:.2f}")
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
