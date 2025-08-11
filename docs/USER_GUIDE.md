# Security Monitor User Guide

## Quick Start

### 1. Installation
```bash
# Clone the repository
git clone https://github.com/yourusername/security-monitor.git
cd security-monitor

# Install dependencies
./scripts/install_dependencies.sh

# Build the workspace
colcon build --symlink-install
source install/setup.bash
```

### 2. Running the System

#### Basic Security Monitor
```bash
ros2 launch security_monitor security_monitor.launch.py
```

#### Complete System
```bash
ros2 launch security_monitor complete_system.launch.py
```

#### With Custom Configuration
```bash
ros2 launch security_monitor complete_system.launch.py config_file:=/path/to/config.yaml
```

### 3. Monitoring Security Status

#### View Security Alerts
```bash
ros2 topic echo /security_alerts
```

#### Monitor Threat Levels
```bash
ros2 topic echo /threat_level
```

#### Check System Health
```bash
ros2 topic echo /system_health
```

## Configuration

Edit `config/security_config.yaml` to customize:
- Threat detection thresholds
- Performance monitoring parameters
- Model management settings
- Emergency handling configuration

## Testing

Run the test suite:
```bash
./scripts/run_tests.sh
```

## Troubleshooting

### Common Issues
1. **Model files not found**: Ensure model files are in the correct directories
2. **High CPU usage**: Adjust detection thresholds in configuration
3. **Sensor data not received**: Check sensor connections and topics

### Logs
Check logs in `/tmp/security_monitor_logs/` for detailed information.

## Support

For issues and questions, please open an issue on GitHub.
