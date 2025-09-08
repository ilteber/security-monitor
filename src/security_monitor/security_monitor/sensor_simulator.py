#!/usr/bin/env python3
"""
Sensor Data Simulator for Security Monitor
Simulates camera and LiDAR data for testing threat detection
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random
import time
import math

class SensorSimulator(Node):
    def __init__(self):
        super().__init__('sensor_simulator')
        
        # Publishers
        self.camera_pub = self.create_publisher(String, 'camera_data', 10)
        self.lidar_pub = self.create_publisher(String, 'lidar_data', 10)
        
        # Timers
        self.camera_timer = self.create_timer(0.1, self.publish_camera_data)  # 10 Hz
        self.lidar_timer = self.create_timer(0.05, self.publish_lidar_data)   # 20 Hz
        
        # Simulation state
        self.time_counter = 0.0
        self.vehicle_speed = 0.0
        self.obstacles = []
        
        self.get_logger().info('Sensor simulator started')

    def generate_camera_data(self):
        """Generate simulated camera data"""
        self.time_counter += 0.1
        
        # Simulate different scenarios
        scenario = int(self.time_counter) % 20
        
        if scenario < 5:
            # Normal driving scenario
            objects = []
        elif scenario < 10:
            # Pedestrian crossing scenario
            objects = [{
                "type": "pedestrian",
                "position": [random.uniform(-2, 2), random.uniform(5, 15), 0],
                "confidence": random.uniform(0.7, 0.95),
                "bbox": [100, 200, 150, 300]
            }]
        elif scenario < 15:
            # Vehicle collision risk
            objects = [{
                "type": "vehicle",
                "position": [random.uniform(-1, 1), random.uniform(3, 8), 0],
                "confidence": random.uniform(0.8, 0.95),
                "bbox": [200, 150, 400, 250]
            }]
        else:
            # Obstacle detection
            objects = [{
                "type": "obstacle",
                "position": [random.uniform(-3, 3), random.uniform(2, 10), 0],
                "confidence": random.uniform(0.6, 0.9),
                "bbox": [150, 100, 250, 200]
            }]
        
        camera_data = {
            "timestamp": time.time(),
            "frame_id": int(self.time_counter * 10),
            "objects": objects,
            "image_quality": random.uniform(0.8, 1.0),
            "lighting_condition": random.choice(["day", "night", "dawn", "dusk"])
        }
        
        return json.dumps(camera_data)

    def generate_lidar_data(self):
        """Generate simulated LiDAR point cloud data"""
        self.time_counter += 0.05
        
        points = []
        
        # Generate point cloud (simplified)
        for angle in range(0, 360, 5):
            for distance in range(1, 50, 2):
                # Add some noise
                noise = random.uniform(-0.1, 0.1)
                actual_distance = distance + noise
                
                # Convert to Cartesian
                x = actual_distance * math.cos(math.radians(angle))
                y = actual_distance * math.sin(math.radians(angle))
                z = random.uniform(-0.5, 2.0)  # Ground to 2m height
                
                # Add intensity (reflectivity)
                intensity = random.uniform(0.1, 1.0)
                
                points.append({
                    "x": round(x, 3),
                    "y": round(y, 3), 
                    "z": round(z, 3),
                    "intensity": round(intensity, 3)
                })
        
        lidar_data = {
            "timestamp": time.time(),
            "points": points,
            "sensor_health": random.uniform(0.9, 1.0),
            "range_max": 50.0,
            "range_min": 0.1
        }
        
        return json.dumps(lidar_data)

    def publish_camera_data(self):
        """Publish camera data"""
        msg = String()
        msg.data = self.generate_camera_data()
        self.camera_pub.publish(msg)

    def publish_lidar_data(self):
        """Publish LiDAR data"""
        msg = String()
        msg.data = self.generate_lidar_data()
        self.lidar_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
