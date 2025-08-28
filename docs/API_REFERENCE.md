# API Reference

## ROS2 Topics

### Security Monitoring Topics
- `/security_alerts` (std_msgs/String): Security alerts and notifications
- `/threat_level` (std_msgs/Float32): Current threat level (0.0-1.0)
- `/threat_type` (std_msgs/String): Type of detected threat
- `/emergency_stop` (std_msgs/Bool): Emergency stop status

### Sensor Data Topics
- `/camera/image_raw` (sensor_msgs/Image): Camera image data
- `/lidar/points` (sensor_msgs/PointCloud2): LiDAR point cloud data
- `/odom` (nav_msgs/Odometry): Vehicle odometry data

### System Monitoring Topics
- `/system_health` (std_msgs/String): System health status
- `/performance_status` (std_msgs/String): Performance monitoring data
- `/cpu_usage` (std_msgs/Float32): CPU usage percentage
- `/memory_usage` (std_msgs/Float32): Memory usage percentage

### Analytics Topics
- `/analytics_report` (std_msgs/String): Analytics data report
- `/statistics` (std_msgs/String): System statistics
- `/notifications` (std_msgs/String): System notifications
- `/alerts` (std_msgs/String): Critical alerts

## ROS2 Services

### Configuration Services
- `/set_detection_threshold` (std_srvs/SetBool): Set detection threshold
- `/enable_emergency_stop` (std_srvs/SetBool): Enable/disable emergency stop
- `/configure_optimization` (std_srvs/SetBool): Configure system optimization

## ROS2 Parameters

### Security Detection Parameters
- `confidence_threshold` (double): Detection confidence threshold
- `nms_threshold` (double): Non-maximum suppression threshold
- `max_detections` (int): Maximum number of detections

### Performance Parameters
- `cpu_threshold` (double): CPU usage threshold
- `memory_threshold` (double): Memory usage threshold
- `latency_threshold` (double): Detection latency threshold

## Node Interfaces

### Security Detector Node
- **Input**: Camera, LiDAR, odometry data
- **Output**: Security alerts, threat levels
- **Parameters**: Detection thresholds, confidence levels

### ML Detector Node
- **Input**: Camera, LiDAR data
- **Output**: Object detections, confidence scores
- **Parameters**: Model paths, inference parameters

### Emergency Handler Node
- **Input**: Threat levels, system status
- **Output**: Emergency commands, alerts
- **Parameters**: Emergency thresholds, response actions
