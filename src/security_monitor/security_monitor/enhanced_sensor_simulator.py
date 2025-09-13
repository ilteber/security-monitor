#!/usr/bin/env python3
"""
Enhanced Sensor Simulator Node
Generates realistic camera and LiDAR data for ML model testing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, PointField
from std_msgs.msg import Header
import random
import struct
import numpy as np
import cv2
from cv_bridge import CvBridge
import time

class EnhancedSensorSimulator(Node):
    def __init__(self):
        super().__init__('enhanced_sensor_simulator')
        
        # Publishers
        self.camera_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.lidar_publisher = self.create_publisher(PointCloud2, '/lidar/points', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # Timer for data generation
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Simulation parameters
        self.frame_count = 0
        self.scene_objects = []
        self.generate_scene()
        
        self.get_logger().info('Enhanced sensor simulator started')

    def generate_scene(self):
        """Generate a simulated scene with objects"""
        self.scene_objects = []
        
        # Add some static objects
        for i in range(5):
            obj = {
                'type': random.choice(['person', 'car', 'bicycle', 'truck']),
                'x': random.uniform(-20, 20),
                'y': random.uniform(-20, 20),
                'z': random.uniform(0, 3),
                'size': random.uniform(0.5, 2.0),
                'color': (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            }
            self.scene_objects.append(obj)

    def timer_callback(self):
        """Main timer callback for data generation"""
        self.publish_camera_data()
        self.publish_lidar_data()
        self.publish_camera_info()
        self.frame_count += 1

    def publish_camera_data(self):
        """Generate and publish realistic camera data"""
        # Create a 640x480 image
        height, width = 480, 640
        image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Add background (road-like)
        image[:] = (50, 50, 50)  # Dark gray road
        
        # Add horizon line
        cv2.line(image, (0, height//2), (width, height//2), (100, 100, 100), 2)
        
        # Add lane markings
        for i in range(0, width, 100):
            cv2.line(image, (i, height//2 + 20), (i + 50, height//2 + 20), (255, 255, 255), 2)
        
        # Add simulated objects
        for obj in self.scene_objects:
            # Project 3D object to 2D image coordinates
            x_2d, y_2d = self.project_3d_to_2d(obj['x'], obj['y'], obj['z'])
            
            if 0 <= x_2d < width and 0 <= y_2d < height:
                # Draw object as rectangle
                size = int(obj['size'] * 50)
                cv2.rectangle(image, 
                             (x_2d - size//2, y_2d - size//2),
                             (x_2d + size//2, y_2d + size//2),
                             obj['color'], -1)
                
                # Add object label
                cv2.putText(image, obj['type'], 
                           (x_2d - size//2, y_2d - size//2 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add some noise
        noise = np.random.randint(0, 30, (height, width, 3), dtype=np.uint8)
        image = cv2.add(image, noise)
        
        # Convert to ROS Image message
        try:
            ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
            ros_image.header = Header()
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_frame'
            self.camera_publisher.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error publishing camera data: {str(e)}')

    def project_3d_to_2d(self, x, y, z):
        """Simple 3D to 2D projection"""
        # Camera parameters
        fx, fy = 525, 525  # Focal length
        cx, cy = 320, 240  # Principal point
        
        # Simple perspective projection
        if z > 0:
            x_2d = int((x * fx) / z + cx)
            y_2d = int((y * fy) / z + cy)
            return x_2d, y_2d
        return 0, 0

    def publish_lidar_data(self):
        """Generate and publish realistic LiDAR data"""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_frame'
        
        # Define fields for x, y, z, intensity
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16  # 4 floats * 4 bytes/float
        msg.is_dense = False
        
        # Generate point cloud data
        points_data = []
        num_points = 1000
        
        # Add ground points
        for _ in range(num_points // 2):
            x = random.uniform(-30, 30)
            y = random.uniform(-30, 30)
            z = 0.0
            intensity = random.uniform(0.1, 0.3)
            points_data.append(struct.pack('<ffff', x, y, z, intensity))
        
        # Add object points
        for obj in self.scene_objects:
            for _ in range(50):  # 50 points per object
                # Add some noise around the object
                x = obj['x'] + random.uniform(-obj['size'], obj['size'])
                y = obj['y'] + random.uniform(-obj['size'], obj['size'])
                z = obj['z'] + random.uniform(0, obj['size'])
                intensity = random.uniform(0.5, 1.0)
                points_data.append(struct.pack('<ffff', x, y, z, intensity))
        
        # Add random noise points
        for _ in range(num_points // 4):
            x = random.uniform(-50, 50)
            y = random.uniform(-50, 50)
            z = random.uniform(0, 10)
            intensity = random.uniform(0.0, 0.5)
            points_data.append(struct.pack('<ffff', x, y, z, intensity))
        
        msg.height = 1
        msg.width = len(points_data)
        msg.row_step = msg.point_step * len(points_data)
        msg.data = b''.join(points_data)
        
        self.lidar_publisher.publish(msg)

    def publish_camera_info(self):
        """Publish camera calibration information"""
        camera_info = CameraInfo()
        camera_info.header = Header()
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.header.frame_id = 'camera_frame'
        
        # Camera parameters
        camera_info.width = 640
        camera_info.height = 480
        camera_info.distortion_model = 'plumb_bob'
        
        # Camera matrix (3x3)
        camera_info.k = [525.0, 0.0, 320.0,
                        0.0, 525.0, 240.0,
                        0.0, 0.0, 1.0]
        
        # Distortion coefficients
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Rectification matrix
        camera_info.r = [1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0]
        
        # Projection matrix
        camera_info.p = [525.0, 0.0, 320.0, 0.0,
                        0.0, 525.0, 240.0, 0.0,
                        0.0, 0.0, 1.0, 0.0]
        
        self.camera_info_publisher.publish(camera_info)

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedSensorSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down enhanced sensor simulator...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
