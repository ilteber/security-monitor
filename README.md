# 🚗 Security Monitor for ROS2

A comprehensive real-time security detection and monitoring system for autonomous vehicles using ROS2, featuring multi-modal sensor fusion, advanced ML models, and a modern web dashboard.

## 🌟 Key Features

- **Real-time Security Detection**: Advanced threat detection using computer vision and LiDAR processing
- **Multi-Modal Sensor Fusion**: Integration of camera, LiDAR, IMU, and GPS data
- **Machine Learning Models**: YOLO, PointPillars, and custom ML models for threat classification
- **Modern Web Dashboard**: Next.js-based real-time monitoring interface with WebSocket support
- **REST API & WebSocket**: FastAPI-based API for real-time monitoring and control
- **Multi-Node Architecture**: ROS2-based distributed system with independent nodes
- **Simulation Support**: Full integration with CARLA and Gazebo simulators
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
         │
         ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Web Dashboard (Next.js)                     │
│  • Real-time Monitoring  • Threat Visualization               │
│  • System Status Cards  • WebSocket Integration               │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Prerequisites

- ROS2 Humble/Galactic
- Python 3.8+ (Python 3.10 for ROS2)
- Node.js 18+ (for web dashboard)
- CUDA (for GPU acceleration)
- CARLA Simulator (optional)

### Installation

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd hazard-monitor
   ```

2. **Install Python dependencies**
   ```bash
   ./scripts/install_dependencies.sh
   pip install -r requirements.txt
   pip install -r requirements-api.txt
   ```

3. **Build the workspace**
   ```bash
   colcon build --packages-select security_monitor
   source install/setup.bash
   ```

4. **Install Node.js dependencies (for web dashboard)**
   ```bash
   cd security-dashboard
   npm install
   ```

### Running the System

1. **Start the API server**
   ```bash
   python3 standalone_api_server.py
   # Or use the script:
   ./start_api_server.sh
   ```

2. **Start the web dashboard**
   ```bash
   cd security-dashboard
   npm run dev
   ```

3. **Start ROS2 nodes**
   ```bash
   # For security detector
   ./run_ros2_node.sh security_detector.py
   
   # For sensor simulator
   ./run_ros2_node.sh sensor_simulator.py
   ```

4. **Access the interfaces**
   - **Web Dashboard**: http://localhost:3000 (or 3001 if 3000 is busy)
   - **API Documentation**: http://localhost:8000/docs
   - **API Health**: http://localhost:8000/health

## 🖥️ Web Dashboard

The modern web dashboard provides real-time monitoring capabilities:

### Features
- **Real-time Threat Monitoring**: Live threat detection with WebSocket updates
- **System Status Cards**: Active threats, system health, confidence levels
- **Threat Visualization**: Color-coded severity levels and threat type icons
- **Responsive Design**: Works on desktop and mobile devices
- **Professional UI**: Modern interface built with Next.js and Tailwind CSS

### Dashboard Components
- **Threat Table**: Real-time list of active threats with details
- **Status Indicators**: Connection status, WebSocket health, system metrics
- **Threat Icons**: Visual representation of different threat types
- **Severity Colors**: Red (critical), Orange (high), Yellow (medium), Green (low)

## 📡 API Integration

The Security Monitor provides a comprehensive REST API and WebSocket interface:

### Core Endpoints

- `GET /health` - API health check
- `GET /threats` - Current active threats
- `GET /status` - System status and health
- `POST /control` - Send control commands
- `GET /analytics/summary` - Security analytics
- `WS /ws/threats` - Real-time threat updates via WebSocket

### Python Client Example

```python
import requests
import json

# Get current threats
response = requests.get('http://localhost:8000/threats')
threats = response.json()
print(f"Active threats: {len(threats)}")

# Get system status
status = requests.get('http://localhost:8000/status').json()
print(f"System health: {status['system_health']}")

# Generate test threat
control_data = {"command": "test_threat"}
requests.post('http://localhost:8000/control', json=control_data)
```

### WebSocket Example

```javascript
const ws = new WebSocket('ws://localhost:8000/ws/threats');

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.type === 'threat_update') {
        console.log('New threats:', data.threats);
    }
};
```

## 🧪 Testing Scenarios

### Real-time Testing
- **Threat Generation**: Random threat generation every 5 seconds
- **WebSocket Updates**: Live threat updates in the dashboard
- **API Endpoints**: Test all REST endpoints for functionality
- **Multi-Node Integration**: ROS2 nodes communicating with API server

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
- **Web Dashboard**: Visual monitoring interface with real-time updates
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
├── security-dashboard/           # Next.js web dashboard
│   ├── src/app/                  # React components
│   ├── src/types/                # TypeScript types
│   └── public/                   # Static assets
├── models/                       # ML model definitions
├── scenarios/                    # Test scenarios
├── docs/                         # Documentation
├── scripts/                      # Utility scripts
├── standalone_api_server.py      # Standalone API server
├── run_ros2_node.sh             # ROS2 node runner script
└── examples/                     # Usage examples
```

### Running Tests

```bash
./scripts/run_tests.sh
```

### Development Commands

```bash
# Start API server
python3 standalone_api_server.py

# Start web dashboard
cd security-dashboard && npm run dev

# Run ROS2 nodes (fixed Python version issues)
./run_ros2_node.sh security_detector.py
./run_ros2_node.sh sensor_simulator.py

# Build ROS2 workspace
colcon build --packages-select security_monitor
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
- [x] Web dashboard with real-time monitoring
- [x] Multi-node system integration
- [x] WebSocket support for live updates
- [x] ROS2 Python version conflict resolution
- [x] Complete system integration testing
- [ ] Advanced ML models
- [ ] Performance optimization
- [ ] Production deployment
- [ ] Cloud integration

## 🟢 Status

**Production Ready** - Core functionality implemented, web dashboard and multi-node integration complete, all Python version conflicts resolved

The project is production-ready with core security monitoring capabilities, modern web dashboard, comprehensive API integration, and fully functional multi-node system.

## 🚀 Recent Updates (September 9, 2025)

- ✅ **ROS2 Python Version Fix**: Resolved Python 3.10/3.12 conflicts
- ✅ **Multi-Node Integration**: Both security_detector and sensor_simulator nodes working
- ✅ **Web Dashboard**: Modern Next.js interface with real-time monitoring
- ✅ **WebSocket Support**: Live threat updates and system status
- ✅ **CORS Support**: Frontend-backend communication enabled
- ✅ **TypeScript Integration**: Proper type definitions for API
- ✅ **Responsive Design**: Mobile-friendly dashboard interface
- ✅ **Complete System Testing**: All components verified working together
- ✅ **Professional Workflow**: Git branching, committing, and merging practices

## 🔧 Troubleshooting

### ROS2 Python Version Issues
If you encounter Python version conflicts with ROS2:
```bash
# Use the provided wrapper script
./run_ros2_node.sh security_detector.py
./run_ros2_node.sh sensor_simulator.py
```

### WebSocket Connection Issues
If WebSocket connections fail:
```bash
# Install required dependencies
pip install 'uvicorn[standard]' websockets
```

### Port Conflicts
If port 3000 is busy, Next.js will automatically use port 3001.

---
*Last updated: September 9, 2025*
