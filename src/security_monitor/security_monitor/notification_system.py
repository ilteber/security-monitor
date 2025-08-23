#!/usr/bin/env python3
"""
Notification System Node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import time

class NotificationSystem(Node):
    def __init__(self):
        super().__init__('notification_system')
        
        # Notification parameters
        self.notifications_enabled = True
        self.notification_history = []
        
        # Subscribers
        self.threat_level_sub = self.create_subscription(
            Float32, 'threat_level', self.threat_callback, 10)
        self.emergency_sub = self.create_subscription(
            Bool, 'emergency_stop', self.emergency_callback, 10)
        self.health_sub = self.create_subscription(
            Bool, 'system_status', self.health_callback, 10)
        
        # Publishers
        self.notification_pub = self.create_publisher(String, 'notifications', 10)
        self.alert_pub = self.create_publisher(String, 'alerts', 10)
        
        # Timer for notification cleanup
        self.timer = self.create_timer(60.0, self.cleanup_notifications)
        
        self.get_logger().info('Notification system started')

    def threat_callback(self, msg):
        """Handle threat level notifications"""
        threat_level = msg.data
        
        if threat_level > 0.8:
            self.send_notification("CRITICAL_THREAT", f"Critical threat detected: {threat_level:.2f}")
        elif threat_level > 0.6:
            self.send_notification("HIGH_THREAT", f"High threat detected: {threat_level:.2f}")

    def emergency_callback(self, msg):
        """Handle emergency notifications"""
        if msg.data:
            self.send_alert("EMERGENCY_STOP", "Emergency stop activated!")

    def health_callback(self, msg):
        """Handle system health notifications"""
        if not msg.data:
            self.send_alert("SYSTEM_UNHEALTHY", "System health degraded!")

    def send_notification(self, notification_type, message):
        """Send notification"""
        if not self.notifications_enabled:
            return
            
        notification = {
            'type': notification_type,
            'message': message,
            'timestamp': time.time(),
            'priority': 'normal'
        }
        
        # Add to history
        self.notification_history.append(notification)
        
        # Publish notification
        notification_msg = String()
        notification_msg.data = json.dumps(notification)
        self.notification_pub.publish(notification_msg)
        
        self.get_logger().info(f"Notification sent: {notification_type} - {message}")

    def send_alert(self, alert_type, message):
        """Send alert"""
        alert = {
            'type': alert_type,
            'message': message,
            'timestamp': time.time(),
            'priority': 'high'
        }
        
        # Publish alert
        alert_msg = String()
        alert_msg.data = json.dumps(alert)
        self.alert_pub.publish(alert_msg)
        
        self.get_logger().warn(f"Alert sent: {alert_type} - {message}")

    def cleanup_notifications(self):
        """Clean up old notifications"""
        current_time = time.time()
        # Keep only last 100 notifications
        self.notification_history = [
            n for n in self.notification_history 
            if current_time - n['timestamp'] < 3600  # Keep for 1 hour
        ][-100:]

def main(args=None):
    rclpy.init(args=args)
    node = NotificationSystem()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
