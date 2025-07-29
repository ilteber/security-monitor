#!/usr/bin/env python3
from std_msgs.msg import Float32
"""
Emergency Handler Node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

class EmergencyHandler(Node):
    def __init__(self):
        super().__init__('emergency_handler')
        
        # Subscribers
        self.threat_level_sub = self.create_subscription(
            Float32, 'threat_level', self.threat_callback, 10)
        
        # Publishers
        self.emergency_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        self.control_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.alert_pub = self.create_publisher(String, 'emergency_alert', 10)
        
        # Emergency thresholds
        self.critical_threshold = 0.8
        self.high_threshold = 0.6
        
        self.get_logger().info('Emergency handler started')

    def threat_callback(self, msg):
        threat_level = msg.data
        
        if threat_level >= self.critical_threshold:
            self.trigger_emergency_stop()
        elif threat_level >= self.high_threshold:
            self.trigger_high_alert()

    def trigger_emergency_stop(self):
        # Emergency stop
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_pub.publish(stop_msg)
        
        # Stop vehicle
        twist = Twist()
        self.control_pub.publish(twist)
        
        # Send alert
        alert_msg = String()
        alert_msg.data = "CRITICAL THREAT - EMERGENCY STOP"
        self.alert_pub.publish(alert_msg)
        
        self.get_logger().warn("EMERGENCY STOP TRIGGERED!")

    def trigger_high_alert(self):
        alert_msg = String()
        alert_msg.data = "HIGH THREAT DETECTED"
        self.alert_pub.publish(alert_msg)
        
        self.get_logger().warn("High threat detected")

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
