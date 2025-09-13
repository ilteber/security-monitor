#!/usr/bin/env python3
"""
Real ML Integration Tests
Tests the actual ML detector with real data flow.
"""

import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseArray
import time
import json

class TestMLIntegrationReal(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('test_ml_integration_real')
        
        # Subscribers for ML detector output
        self.detections_received = []
        self.confidence_received = []
        self.threat_objects_received = []
        self.bounding_boxes_received = []
        
        # Create subscribers
        self.detections_sub = self.node.create_subscription(
            PoseArray, 'ml_detections', self.detections_callback, 10)
        self.confidence_sub = self.node.create_subscription(
            Float32, 'detection_confidence', self.confidence_callback, 10)
        self.threat_objects_sub = self.node.create_subscription(
            String, 'threat_objects', self.threat_objects_callback, 10)
        self.bounding_boxes_sub = self.node.create_subscription(
            String, 'bounding_boxes_2d', self.bounding_boxes_callback, 10)
        
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        
    def detections_callback(self, msg):
        self.detections_received.append(msg)
        
    def confidence_callback(self, msg):
        self.confidence_received.append(msg)
        
    def threat_objects_callback(self, msg):
        self.threat_objects_received.append(msg)
        
    def bounding_boxes_callback(self, msg):
        self.bounding_boxes_received.append(msg)

    def test_ml_detection_flow(self):
        """Test that ML detection is working end-to-end"""
        # Spin for 10 seconds to collect data
        start_time = time.time()
        while time.time() - start_time < 10.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        # Check if we received any detections
        self.assertGreater(len(self.detections_received), 0, "No detections received")
        self.assertGreater(len(self.confidence_received), 0, "No confidence values received")
        self.assertGreater(len(self.threat_objects_received), 0, "No threat objects received")
        self.assertGreater(len(self.bounding_boxes_received), 0, "No bounding boxes received")
        
        print(f"✅ Received {len(self.detections_received)} detections")
        print(f"✅ Received {len(self.confidence_received)} confidence values")
        print(f"✅ Received {len(self.threat_objects_received)} threat objects")
        print(f"✅ Received {len(self.bounding_boxes_received)} bounding boxes")
        
    def test_confidence_values_valid(self):
        """Test that confidence values are in valid range"""
        start_time = time.time()
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        for conf_msg in self.confidence_received:
            self.assertGreaterEqual(conf_msg.data, 0.0, f"Confidence {conf_msg.data} is negative")
            self.assertLessEqual(conf_msg.data, 1.0, f"Confidence {conf_msg.data} is > 1.0")
            
    def test_threat_objects_format(self):
        """Test that threat objects are in correct format"""
        start_time = time.time()
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        for threat_msg in self.threat_objects_received:
            # Should be comma-separated list of object names
            objects = threat_msg.data.split(',')
            for obj in objects:
                if obj:  # Skip empty strings
                    self.assertIsInstance(obj, str)
                    self.assertIn(obj, ['person', 'car', 'bicycle', 'motorcycle', 'bus', 'truck', 'traffic_light', 'stop_sign'])
                    
    def test_bounding_boxes_json(self):
        """Test that bounding boxes are valid JSON"""
        start_time = time.time()
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        for bbox_msg in self.bounding_boxes_received:
            try:
                bbox_data = json.loads(bbox_msg.data)
                self.assertIsInstance(bbox_data, list)
                for bbox in bbox_data:
                    self.assertIn('class', bbox)
                    self.assertIn('confidence', bbox)
                    self.assertIn('bbox', bbox)
                    self.assertIn('timestamp', bbox)
            except json.JSONDecodeError as e:
                self.fail(f"Bounding box data is not valid JSON: {e}")
                
    def test_detection_performance(self):
        """Test that detection performance is reasonable"""
        start_time = time.time()
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        # Check that we're getting detections at a reasonable rate
        detection_rate = len(self.detections_received) / 5.0  # detections per second
        self.assertGreater(detection_rate, 0.1, f"Detection rate too low: {detection_rate} detections/sec")
        
        print(f"✅ Detection rate: {detection_rate:.2f} detections/sec")

if __name__ == '__main__':
    unittest.main()
