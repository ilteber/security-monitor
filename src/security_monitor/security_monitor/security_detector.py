#!/usr/bin/env python3
"""
Enhanced Security Detector Node with Real Threat Detection
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random
import time
from datetime import datetime

class SecurityDetector(Node):
    def __init__(self):
        super().__init__('security_detector')
        
        # Publishers
        self.publisher = self.create_publisher(String, 'security_alerts', 10)
        self.threat_publisher = self.create_publisher(String, 'threat_details', 10)
        
        # Subscribers (for future sensor integration)
        self.camera_sub = self.create_subscription(
            String, 'camera_data', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(
            String, 'lidar_data', self.lidar_callback, 10)
        
        # Timer for threat detection
        self.timer = self.create_timer(2.0, self.threat_detection_loop)
        
        # Threat detection state
        self.active_threats = []
        self.threat_counter = 0
        self.last_detection_time = time.time()
        
        # Simulated sensor data
        self.camera_data = None
        self.lidar_data = None
        
        self.get_logger().info('Enhanced security detector started')

    def camera_callback(self, msg):
        """Process camera data for threat detection"""
        self.camera_data = msg.data
        self.get_logger().debug(f'Camera data received: {len(msg.data)} chars')

    def lidar_callback(self, msg):
        """Process LiDAR data for threat detection"""
        self.lidar_data = msg.data
        self.get_logger().debug(f'LiDAR data received: {len(msg.data)} chars')

    def detect_threats(self):
        """Main threat detection logic"""
        threats = []
        current_time = time.time()
        
        # Simulate different types of threats
        threat_probability = 0.3  # 30% chance of threat per check
        
        if random.random() < threat_probability:
            threat_types = [
                "pedestrian_crossing",
                "vehicle_collision_risk", 
                "obstacle_detected",
                "suspicious_activity",
                "weather_hazard",
                "sensor_malfunction"
            ]
            
            threat_type = random.choice(threat_types)
            severity = random.choice(["low", "medium", "high", "critical"])
            confidence = round(random.uniform(0.6, 0.95), 2)
            
            threat = {
                "id": self.threat_counter,
                "type": threat_type,
                "severity": severity,
                "confidence": confidence,
                "timestamp": datetime.now().isoformat(),
                "location": {
                    "x": round(random.uniform(-10, 10), 2),
                    "y": round(random.uniform(-10, 10), 2),
                    "z": round(random.uniform(0, 3), 2)
                },
                "description": self.get_threat_description(threat_type, severity)
            }
            
            threats.append(threat)
            self.threat_counter += 1
            
        return threats

    def get_threat_description(self, threat_type, severity):
        """Generate human-readable threat descriptions"""
        descriptions = {
            "pedestrian_crossing": f"{severity.title()} risk: Pedestrian detected in vehicle path",
            "vehicle_collision_risk": f"{severity.title()} risk: Potential vehicle collision detected",
            "obstacle_detected": f"{severity.title()} risk: Obstacle blocking vehicle path",
            "suspicious_activity": f"{severity.title()} risk: Suspicious activity detected nearby",
            "weather_hazard": f"{severity.title()} risk: Weather conditions affecting visibility",
            "sensor_malfunction": f"{severity.title()} risk: Sensor malfunction detected"
        }
        return descriptions.get(threat_type, f"{severity.title()} risk: Unknown threat detected")

    def threat_detection_loop(self):
        """Main threat detection loop"""
        # Detect new threats
        new_threats = self.detect_threats()
        
        # Add new threats to active list
        for threat in new_threats:
            self.active_threats.append(threat)
            self.get_logger().warn(f"THREAT DETECTED: {threat['type']} - {threat['description']}")
        
        # Remove old threats (older than 30 seconds)
        current_time = time.time()
        self.active_threats = [
            threat for threat in self.active_threats 
            if (current_time - datetime.fromisoformat(threat['timestamp']).timestamp()) < 30
        ]
        
        # Publish threat status
        self.publish_threat_status()
        
        # Publish detailed threat information
        if new_threats:
            self.publish_threat_details(new_threats)

    def publish_threat_status(self):
        """Publish current threat status"""
        msg = String()
        
        if self.active_threats:
            threat_count = len(self.active_threats)
            critical_threats = [t for t in self.active_threats if t['severity'] == 'critical']
            
            if critical_threats:
                msg.data = f"CRITICAL: {len(critical_threats)} critical threats, {threat_count} total active threats"
            else:
                msg.data = f"WARNING: {threat_count} active security threats detected"
        else:
            msg.data = "No security threats detected"
        
        self.publisher.publish(msg)

    def publish_threat_details(self, threats):
        """Publish detailed threat information"""
        for threat in threats:
            msg = String()
            msg.data = json.dumps(threat)
            self.threat_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SecurityDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
