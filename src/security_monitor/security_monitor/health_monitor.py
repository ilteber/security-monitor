#!/usr/bin/env python3
"""
System Health Monitor Node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import psutil
import time

class HealthMonitor(Node):
    def __init__(self):
        super().__init__('health_monitor')
        
        # Health monitoring parameters
        self.system_healthy = True
        self.last_heartbeat = time.time()
        
        # Publishers
        self.health_status_pub = self.create_publisher(String, 'system_health', 10)
        self.health_alert_pub = self.create_publisher(Bool, 'health_alert', 10)
        
        # Timer for health checks
        self.timer = self.create_timer(2.0, self.health_check)
        
        self.get_logger().info('Health monitor started')

    def health_check(self):
        """Perform system health check"""
        current_time = time.time()
        
        # Check system resources
        cpu_usage = psutil.cpu_percent()
        memory_usage = psutil.virtual_memory().percent
        disk_usage = psutil.disk_usage('/').percent
        
        # Determine health status
        if cpu_usage > 90 or memory_usage > 95 or disk_usage > 95:
            self.system_healthy = False
            self.get_logger().error('System health degraded!')
        else:
            self.system_healthy = True
        
        # Publish health status
        status_msg = String()
        status_msg.data = f"healthy: {self.system_healthy}, cpu: {cpu_usage:.1f}%, memory: {memory_usage:.1f}%"
        self.health_status_pub.publish(status_msg)
        
        # Publish health alert
        alert_msg = Bool()
        alert_msg.data = not self.system_healthy
        self.health_alert_pub.publish(alert_msg)
        
        self.last_heartbeat = current_time

def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
