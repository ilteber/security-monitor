# Security Monitor Architecture

## System Overview

The Security Monitor system is designed as a modular ROS2-based framework for real-time security threat detection in autonomous vehicles.

## Core Components

### 1. Security Detector
- Main security monitoring node
- Coordinates all security subsystems
- Publishes security alerts and status

### 2. Sensor Monitor
- Monitors camera and LiDAR data
- Detects sensor failures and anomalies
- Provides sensor health status

### 3. ML Detector
- YOLO-based object detection
- PointPillars 3D object detection
- Threat object classification

### 4. Sensor Fusion
- Multi-sensor data integration
- Camera and LiDAR data correlation
- Fused detection results

### 5. Threat Classifier
- Threat level assessment
- Risk evaluation and classification
- Threat type identification

### 6. Emergency Handler
- Critical threat response
- Emergency stop procedures
- Alert notification system

### 7. Weather Monitor
- Weather condition detection
- Visibility range assessment
- Weather-based threat adjustment

### 8. Model Manager
- ML model loading and management
- Model performance monitoring
- Model health checks

## Data Flow

```
Sensors → Sensor Monitor → ML Detector → Sensor Fusion → Threat Classifier → Emergency Handler
    ↓           ↓              ↓             ↓              ↓
Weather Monitor → Model Manager → Security Detector → Alerts/Controls
```

## Simulation Integration

- **CARLA**: High-fidelity autonomous driving simulation
- **Gazebo**: Physics-based robotics simulation
- **Scenario Testing**: Automated security scenario validation

## Future Development

- Advanced ML model integration
- Real-time performance optimization
- Enhanced scenario testing
- User interface development
