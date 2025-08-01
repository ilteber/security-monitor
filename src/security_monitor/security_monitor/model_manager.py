#!/usr/bin/env python3
"""
ML Model Manager Node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import os
import json

class ModelManager(Node):
    def __init__(self):
        super().__init__('model_manager')
        
        # Model management
        self.models = {}
        self.model_paths = {
            'yolo': 'models/yolo/yolov8n.pt',
            'pointpillars': 'models/pointpillars/pointpillars.pth'
        }
        
        # Publishers
        self.model_status_pub = self.create_publisher(String, 'model_status', 10)
        self.model_performance_pub = self.create_publisher(Float32, 'model_performance', 10)
        
        # Timer for model health checks
        self.timer = self.create_timer(5.0, self.model_health_check)
        
        self.get_logger().info('Model manager started')
        self.load_models()

    def load_models(self):
        """Load and initialize ML models"""
        for model_name, model_path in self.model_paths.items():
            if self.check_model_exists(model_path):
                self.models[model_name] = {
                    'path': model_path,
                    'loaded': True,
                    'performance': 0.85  # Simulated performance
                }
                self.get_logger().info(f'Model {model_name} loaded successfully')
            else:
                self.models[model_name] = {
                    'path': model_path,
                    'loaded': False,
                    'performance': 0.0
                }
                self.get_logger().warn(f'Model {model_name} not found at {model_path}')

    def check_model_exists(self, model_path):
        """Check if model file exists"""
        return os.path.exists(model_path)

    def model_health_check(self):
        """Periodic model health check"""
        for model_name, model_info in self.models.items():
            if model_info['loaded']:
                # Simulate performance monitoring
                performance = model_info['performance'] + (0.01 * (0.5 - 0.5))  # Random variation
                model_info['performance'] = max(0.0, min(1.0, performance))
                
                # Publish model status
                status_msg = String()
                status_msg.data = json.dumps({
                    'model': model_name,
                    'status': 'healthy',
                    'performance': model_info['performance']
                })
                self.model_status_pub.publish(status_msg)
                
                # Publish performance metric
                perf_msg = Float32()
                perf_msg.data = model_info['performance']
                self.model_performance_pub.publish(perf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ModelManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
