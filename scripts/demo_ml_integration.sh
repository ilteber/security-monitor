#!/bin/bash

echo "🚀 Starting ML Integration Demo"
echo "================================"

# Function to check if a command is running
check_process() {
    if pgrep -f "$1" > /dev/null; then
        echo "✅ $1 is running"
        return 0
    else
        echo "❌ $1 is not running"
        return 1
    fi
}

# Function to start a process in background
start_background() {
    echo "Starting $1..."
    $2 &
    sleep 2
    if check_process "$1"; then
        echo "✅ $1 started successfully"
    else
        echo "❌ Failed to start $1"
        exit 1
    fi
}

# Clean up any existing processes
echo "Cleaning up existing processes..."
pkill -f "enhanced_sensor_simulator\|ml_detector_simple\|standalone_api_server" 2>/dev/null || true
sleep 2

# Start the ML integration system
echo ""
echo "1. Starting Enhanced Sensor Simulator..."
start_background "enhanced_sensor_simulator" "source /opt/ros/humble/setup.bash && source install/setup.bash && ./run_ros2_node.sh enhanced_sensor_simulator.py"

echo ""
echo "2. Starting ML Detector..."
start_background "ml_detector_simple" "source /opt/ros/humble/setup.bash && source install/setup.bash && ./run_ros2_node.sh ml_detector_simple.py"

echo ""
echo "3. Starting Enhanced API Server..."
start_background "standalone_api_server" "python3 standalone_api_server_ml.py"

# Wait for everything to start
echo ""
echo "Waiting for system to initialize..."
sleep 5

# Check system status
echo ""
echo "4. Checking System Status..."
echo "============================="

# Check ROS2 topics
echo "ROS2 Topics:"
source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 topic list | grep -E "(camera|lidar|ml_|threat|detection)" | head -10

echo ""
echo "API Server Status:"
curl -s http://localhost:8000/status | python3 -m json.tool 2>/dev/null || echo "API server not responding"

echo ""
echo "5. Running ML Integration Test..."
echo "================================="
source /opt/ros/humble/setup.bash && source install/setup.bash && python3 tests/test_ml_integration_real.py

echo ""
echo "6. Live Detection Monitoring (10 seconds)..."
echo "============================================="
echo "Watching threat objects and ML detections..."
timeout 10 bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && while true; do echo "Threat Objects:"; timeout 2 ros2 topic echo /threat_objects --once 2>/dev/null || echo "No threats"; echo "ML Detections:"; timeout 2 ros2 topic echo /ml_detections --once 2>/dev/null || echo "No detections"; echo "---"; sleep 1; done'

echo ""
echo "7. API Endpoints Test..."
echo "========================"
echo "Threats:"
curl -s http://localhost:8000/threats | python3 -m json.tool 2>/dev/null | head -20 || echo "No threats data"

echo ""
echo "ML Detections:"
curl -s http://localhost:8000/ml_detections | python3 -m json.tool 2>/dev/null | head -20 || echo "No ML detections data"

echo ""
echo "Analytics:"
curl -s http://localhost:8000/analytics | python3 -m json.tool 2>/dev/null || echo "No analytics data"

echo ""
echo "🎉 ML Integration Demo Complete!"
echo "================================"
echo "System is running with:"
echo "✅ Enhanced Sensor Simulator (realistic camera/LiDAR data)"
echo "✅ ML Detector (OpenCV-based object detection)"
echo "✅ Enhanced API Server (with ML detection endpoints)"
echo "✅ Real-time threat detection and classification"
echo ""
echo "Access the web dashboard at: http://localhost:3000"
echo "API documentation at: http://localhost:8000/docs"
echo ""
echo "Press Ctrl+C to stop all processes"

# Keep script running and show live updates
while true; do
    sleep 5
    echo ""
    echo "📊 Live Status Update:"
    echo "Active threats: $(curl -s http://localhost:8000/status 2>/dev/null | python3 -c 'import sys, json; data=json.load(sys.stdin); print(data.get("active_threats", 0))' 2>/dev/null || echo "N/A")"
    echo "ML detections: $(curl -s http://localhost:8000/status 2>/dev/null | python3 -c 'import sys, json; data=json.load(sys.stdin); print(data.get("ml_detections", 0))' 2>/dev/null || echo "N/A")"
done
