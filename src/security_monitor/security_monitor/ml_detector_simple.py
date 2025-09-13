#!/usr/bin/env python3
"""
Simplified ML Detector Node
Works with ROS2 Python 3.10 environment without heavy ML dependencies.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String, Float32, Header
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import time
import random

class MLDetectorSimple(Node):
    def __init__(self):
        super().__init__('ml_detector_simple')
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar/points', self.lidar_callback, 10)
        
        # Publishers
        self.detections_pub = self.create_publisher(PoseArray, 'ml_detections', 10)
        self.confidence_pub = self.create_publisher(Float32, 'detection_confidence', 10)
        self.threat_objects_pub = self.create_publisher(String, 'threat_objects', 10)
        self.bounding_boxes_pub = self.create_publisher(String, 'bounding_boxes_2d', 10)
        self.bounding_boxes_3d_pub = self.create_publisher(String, 'bounding_boxes_3d', 10)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Detection parameters
        self.detection_threshold = 0.5
        self.threat_classes = ['person', 'bicycle', 'car', 'motorcycle', 'bus', 'truck', 'traffic_light', 'stop_sign']
        
        # Performance tracking
        self.detection_count = 0
        self.last_detection_time = time.time()
        
        # Simple object detection using OpenCV
        self.face_cascade = None
        self.car_cascade = None
        self.load_cascades()
        
        self.get_logger().info('Simplified ML Detector started')

    def load_cascades(self):
        """Load OpenCV Haar cascades for basic object detection"""
        try:
            # Try to load Haar cascades (these come with OpenCV)
            self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
            self.car_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_car.xml')
            self.get_logger().info('OpenCV cascades loaded successfully')
        except Exception as e:
            self.get_logger().warn(f'Could not load cascades: {str(e)}')
            self.face_cascade = None
            self.car_cascade = None

    def camera_callback(self, msg):
        """Process camera data with simplified detection"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run simplified detection
            detections = []
            threat_objects = []
            bounding_boxes = []
            
            # Use OpenCV cascades for basic detection
            if self.face_cascade is not None:
                faces = self.face_cascade.detectMultiScale(cv_image, 1.1, 4)
                for (x, y, w, h) in faces:
                    confidence = 0.8  # Simulated confidence
                    if confidence >= self.detection_threshold:
                        threat_objects.append('person')
                        
                        bbox_info = {
                            'class': 'person',
                            'confidence': confidence,
                            'bbox': [float(x), float(y), float(x+w), float(y+h)],
                            'timestamp': time.time()
                        }
                        bounding_boxes.append(bbox_info)
                        
                        # Create 3D pose (approximate depth)
                        center_x = x + w // 2
                        center_y = y + h // 2
                        estimated_depth = self.estimate_depth(h, 'person')
                        
                        pose = Pose()
                        pose.position = Point(x=float(center_x), y=float(center_y), z=estimated_depth)
                        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                        detections.append(pose)
            
            # Simulate additional detections based on image analysis
            self.simulate_additional_detections(cv_image, detections, threat_objects, bounding_boxes)
            
            # Publish results
            if detections:
                pose_array = PoseArray()
                pose_array.header = msg.header
                pose_array.poses = detections
                self.detections_pub.publish(pose_array)
                
                # Publish confidence
                avg_confidence = np.mean([bbox['confidence'] for bbox in bounding_boxes]) if bounding_boxes else 0.0
                conf_msg = Float32()
                conf_msg.data = float(avg_confidence)
                self.confidence_pub.publish(conf_msg)
                
                # Publish threat objects
                threat_msg = String()
                threat_msg.data = ','.join(set(threat_objects))
                self.threat_objects_pub.publish(threat_msg)
                
                # Publish bounding boxes
                bbox_msg = String()
                bbox_msg.data = json.dumps(bounding_boxes)
                self.bounding_boxes_pub.publish(bbox_msg)
                
                self.detection_count += 1
                self.last_detection_time = time.time()
                
                self.get_logger().info(f'Detected {len(detections)} threats: {threat_objects}')
            
        except Exception as e:
            self.get_logger().error(f'Error in camera callback: {str(e)}')

    def simulate_additional_detections(self, cv_image, detections, threat_objects, bounding_boxes):
        """Simulate additional detections based on image analysis"""
        height, width = cv_image.shape[:2]
        
        # Simulate detection based on image features
        if random.random() < 0.3:  # 30% chance of detection
            # Simulate vehicle detection
            x = random.randint(0, width - 100)
            y = random.randint(0, height - 100)
            w = random.randint(50, 150)
            h = random.randint(50, 100)
            confidence = random.uniform(0.6, 0.9)
            
            if confidence >= self.detection_threshold:
                threat_objects.append('car')
                
                bbox_info = {
                    'class': 'car',
                    'confidence': confidence,
                    'bbox': [float(x), float(y), float(x+w), float(y+h)],
                    'timestamp': time.time()
                }
                bounding_boxes.append(bbox_info)
                
                # Create 3D pose
                center_x = x + w // 2
                center_y = y + h // 2
                estimated_depth = self.estimate_depth(h, 'car')
                
                pose = Pose()
                pose.position = Point(x=float(center_x), y=float(center_y), z=estimated_depth)
                pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                detections.append(pose)

    def lidar_callback(self, msg):
        """Process LiDAR data with simplified 3D detection"""
        try:
            # Simulate 3D detection
            self.simulate_3d_detection(msg)
        except Exception as e:
            self.get_logger().error(f'Error in LiDAR callback: {str(e)}')

    def estimate_depth(self, bbox_height: float, class_name: str) -> float:
        """Estimate depth based on bounding box height and object class"""
        # Approximate real-world heights for different objects
        real_heights = {
            'person': 1.7,
            'bicycle': 1.0,
            'car': 1.5,
            'motorcycle': 1.2,
            'bus': 3.0,
            'truck': 3.5,
            'traffic_light': 3.0,
            'stop_sign': 2.5
        }
        
        # Camera parameters (approximate)
        focal_length = 525  # pixels
        real_height = real_heights.get(class_name, 1.5)  # default height
        
        # Estimate depth using similar triangles
        estimated_depth = (real_height * focal_length) / bbox_height
        
        # Clamp to reasonable range
        return max(1.0, min(estimated_depth, 100.0))

    def simulate_3d_detection(self, pointcloud_msg):
        """Simulate 3D object detection results"""
        # Create dummy 3D detections
        detections = PoseArray()
        detections.header = pointcloud_msg.header
        
        # Simulate some 3D detections
        for i in range(random.randint(0, 3)):  # 0-3 detections
            pose = Pose()
            pose.position = Point(
                x=float(random.uniform(-10, 10)),
                y=float(random.uniform(-10, 10)),
                z=float(random.uniform(0, 3))
            )
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            detections.poses.append(pose)
        
        self.detections_pub.publish(detections)
        
        # Publish 3D bounding boxes
        bbox_3d_info = {
            'detections': [
                {
                    'class': 'vehicle',
                    'confidence': random.uniform(0.7, 0.95),
                    'position': [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
                    'timestamp': time.time()
                } for pose in detections.poses
            ]
        }
        
        bbox_3d_msg = String()
        bbox_3d_msg.data = json.dumps(bbox_3d_info)
        self.bounding_boxes_3d_pub.publish(bbox_3d_msg)

    def get_performance_stats(self) -> dict:
        """Get performance statistics"""
        current_time = time.time()
        time_since_last = current_time - self.last_detection_time
        
        return {
            'total_detections': self.detection_count,
            'time_since_last_detection': time_since_last,
            'detection_threshold': self.detection_threshold
        }

def main(args=None):
    rclpy.init(args=args)
    node = MLDetectorSimple()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Simplified ML Detector...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
