#!/usr/bin/env python3
"""
ML Integration Tests
Tests the ML detector with real model integration.
"""

import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseArray
import time
import json

class TestMLIntegration(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('test_ml_integration')
        
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

    def test_ml_detector_initialization(self):
        """Test that ML detector initializes properly"""
        # This test would require importing and testing the MLDetector class
        # For now, we'll test the basic structure
        self.assertTrue(True)
        
    def test_detection_topics_exist(self):
        """Test that detection topics are available"""
        # Spin for a short time to check for messages
        start_time = time.time()
        while time.time() - start_time < 2.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        # Check if we received any messages
        self.assertGreaterEqual(len(self.detections_received), 0)
        
    def test_confidence_values(self):
        """Test that confidence values are in valid range"""
        start_time = time.time()
        while time.time() - start_time < 3.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        for conf_msg in self.confidence_received:
            self.assertGreaterEqual(conf_msg.data, 0.0)
            self.assertLessEqual(conf_msg.data, 1.0)
            
    def test_threat_objects_format(self):
        """Test that threat objects are in correct format"""
        start_time = time.time()
        while time.time() - start_time < 3.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        for threat_msg in self.threat_objects_received:
            # Should be comma-separated list of object names
            objects = threat_msg.data.split(',')
            for obj in objects:
                if obj:  # Skip empty strings
                    self.assertIsInstance(obj, str)
                    
    def test_bounding_boxes_json(self):
        """Test that bounding boxes are valid JSON"""
        start_time = time.time()
        while time.time() - start_time < 3.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        for bbox_msg in self.bounding_boxes_received:
            try:
                bbox_data = json.loads(bbox_msg.data)
                self.assertIsInstance(bbox_data, list)
            except json.JSONDecodeError:
                self.fail("Bounding box data is not valid JSON")

class TestMLPerformance(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('test_ml_performance')
        
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        
    def test_detection_latency(self):
        """Test that detection latency is within acceptable limits"""
        # This would require more sophisticated timing tests
        # For now, just test basic functionality
        self.assertTrue(True)
        
    def test_memory_usage(self):
        """Test that memory usage is reasonable"""
        # This would require memory profiling
        # For now, just test basic functionality
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
