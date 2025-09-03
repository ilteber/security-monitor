# 🚗 Security Monitor for ROS2

A comprehensive real-time security detection and monitoring system for autonomous vehicles using ROS2, featuring multi-modal sensor fusion, advanced ML models, and robust simulation environments.

## 🌟 Key Features

- **Real-time Security Detection**: Advanced threat detection using computer vision and LiDAR processing
- **Multi-Modal Sensor Fusion**: Integration of camera, LiDAR, IMU, and GPS data
- **Machine Learning Models**: YOLO, PointPillars, and custom ML models for threat classification
- **Simulation Support**: Full integration with CARLA and Gazebo simulators
- **REST API & WebSocket**: FastAPI-based API for real-time monitoring and control
- **Modular Architecture**: ROS2-based distributed system with independent nodes
- **Comprehensive Monitoring**: System health, performance, and security analytics

## 🏗️ System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Sensors       │    │   ML Models     │    │   API Server    │
│   • Camera      │───▶│   • YOLO        │───▶│   • FastAPI     │
│   • LiDAR       │    │   • PointPillars│    │   • WebSocket   │
│   • IMU         │    │   • Custom      │    │   • REST API    │
│   • GPS         │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Security Monitor Core                       │
│  • Threat Detection  • Sensor Fusion  • Emergency Handler     │
│  • Performance Monitor  • Health Monitor  • Analytics         │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Prerequisites

- ROS2 Humble/Galactic
- Python 3.8+
- CUDA (for GPU acceleration)
- CARLA Simulator (optional)

### Installation

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd hazard-monitor
   ```

2. **Install dependencies**
   ```bash
   ./scripts/install_dependencies.sh
   pip install -r requirements.txt
   ```

3. **Build the workspace**
   ```bash
   colcon build --packages-select security_monitor
   source install/setup.bash
   ```

### Running the System

1. **Start the complete security monitoring system**
   ```bash
   ros2 launch security_monitor complete_system.launch.py
   ```

2. **Start the API server**
   ```bash
   ./scripts/start_api_server.sh
   # Or manually:
   ros2 launch security_monitor api_server.launch.py
   ```

3. **Access the API documentation**
   - Swagger UI: http://localhost:8000/docs
   - ReDoc: http://localhost:8000/redoc

## 📡 API Integration

The Security Monitor provides a comprehensive REST API and WebSocket interface:

### Core Endpoints

- `GET /health` - API health check
- `GET /threats` - Current active threats
- `GET /status` - System status and health
- `POST /control` - Send control commands
- `GET /analytics/summary` - Security analytics
- `WS /ws/threats` - Real-time threat updates

### Python Client Example

```python
from security_monitor.api_client import SecurityMonitorClient

async def main():
    async with SecurityMonitorClient() as client:
        # Get current threats
        threats = await client.get_current_threats()
        print(f"Active threats: {len(threats)}")
        
        # Emergency stop if needed
        if len(threats) > 0:
            await client.emergency_stop()

asyncio.run(main())
```

See [API Integration Guide](docs/API_INTEGRATION_GUIDE.md) for detailed documentation.

## 🧪 Testing Scenarios

### CARLA Scenarios
- Highway construction zones
- Pedestrian crossings
- Emergency vehicle encounters
- Weather conditions

### Gazebo Scenarios
- Urban intersections
- Parking lot navigation
- Multi-vehicle interactions

## ⚙️ Configuration

System configuration is managed through YAML files in `src/security_monitor/config/`:

- `security_config.yaml` - Main security detection parameters
- Threat classification thresholds
- Emergency response settings
- Performance monitoring parameters

## 📊 Monitoring & Analytics

The system provides comprehensive monitoring capabilities:

- **Real-time Performance Metrics**: CPU, memory, GPU usage
- **Security Analytics**: Threat detection accuracy, false positive rates
- **System Health**: Component status and degradation detection
- **Data Logging**: Comprehensive event logging for analysis

## 🔧 Development

### Project Structure

```
hazard-monitor/
├── src/security_monitor/          # Main ROS2 package
│   ├── security_monitor/         # Python modules
│   ├── api/                      # FastAPI integration
│   ├── config/                   # Configuration files
│   └── launch/                   # Launch files
├── models/                       # ML model definitions
├── scenarios/                    # Test scenarios
├── docs/                         # Documentation
├── scripts/                      # Utility scripts
└── examples/                     # Usage examples
```

### Running Tests

```bash
./scripts/run_tests.sh
```

### Code Quality

```bash
# Format code
black src/ tests/

# Lint code
flake8 src/ tests/

# Type checking
mypy src/
```

## 📚 Documentation

- [Architecture Overview](docs/ARCHITECTURE.md)
- [API Integration Guide](docs/API_INTEGRATION_GUIDE.md)
- [User Guide](docs/USER_GUIDE.md)
- [Development Roadmap](docs/DEVELOPMENT_ROADMAP.md)
- [API Reference](docs/API_REFERENCE.md)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 👨‍💻 Author

**Ilteber Ayvaci**
- Email: ayvaci.ilteber@gmail.com
- GitHub: [@ilteber](https://github.com/ilteber)

## 🗺️ Roadmap

- [x] Basic ROS2 package structure
- [x] Core security detection nodes
- [x] ML model integration
- [x] FastAPI integration
- [x] Simulation interfaces
- [ ] Advanced ML models
- [ ] Performance optimization
- [ ] Production deployment
- [ ] Cloud integration

## �� Status

🟢 **Active Development** - Core functionality implemented, API integration complete

The project is in active development with core security monitoring capabilities and API integration fully functional.
# Updated Mi 3. Sep 17:20:17 CEST 2025
