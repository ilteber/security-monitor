#!/usr/bin/env python3
"""
Analytics Node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import time
from collections import defaultdict

class Analytics(Node):
    def __init__(self):
        super().__init__('analytics')
        
        # Analytics data
        self.threat_statistics = defaultdict(int)
        self.performance_metrics = []
        self.error_statistics = defaultdict(int)
        
        # Subscribers
        self.threat_level_sub = self.create_subscription(
            Float32, 'threat_level', self.threat_callback, 10)
        self.performance_sub = self.create_subscription(
            String, 'performance_status', self.performance_callback, 10)
        self.error_sub = self.create_subscription(
            String, 'error_report', self.error_callback, 10)
        
        # Publishers
        self.analytics_report_pub = self.create_publisher(String, 'analytics_report', 10)
        self.statistics_pub = self.create_publisher(String, 'statistics', 10)
        
        # Timer for analytics updates
        self.timer = self.create_timer(30.0, self.generate_analytics)
        
        self.get_logger().info('Analytics started')

    def threat_callback(self, msg):
        """Process threat level data"""
        threat_level = msg.data
        
        if threat_level < 0.2:
            self.threat_statistics['low'] += 1
        elif threat_level < 0.6:
            self.threat_statistics['medium'] += 1
        elif threat_level < 0.8:
            self.threat_statistics['high'] += 1
        else:
            self.threat_statistics['critical'] += 1

    def performance_callback(self, msg):
        """Process performance data"""
        self.performance_metrics.append({
            'timestamp': time.time(),
            'status': msg.data
        })
        
        # Keep only last 100 metrics
        if len(self.performance_metrics) > 100:
            self.performance_metrics.pop(0)

    def error_callback(self, msg):
        """Process error data"""
        error_type = msg.data.split(':')[0] if ':' in msg.data else 'unknown'
        self.error_statistics[error_type] += 1

    def generate_analytics(self):
        """Generate analytics report"""
        analytics_data = {
            'threat_statistics': dict(self.threat_statistics),
            'error_statistics': dict(self.error_statistics),
            'performance_metrics_count': len(self.performance_metrics),
            'timestamp': time.time()
        }
        
        # Publish analytics report
        report_msg = String()
        report_msg.data = json.dumps(analytics_data)
        self.analytics_report_pub.publish(report_msg)
        
        # Publish statistics
        stats_msg = String()
        stats_msg.data = f"Threats: {sum(self.threat_statistics.values())}, Errors: {sum(self.error_statistics.values())}"
        self.statistics_pub.publish(stats_msg)
        
        self.get_logger().info(f"Analytics report generated: {stats_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = Analytics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
