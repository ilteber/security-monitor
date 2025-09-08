#!/bin/bash
# Security Monitor API Server Startup Script

echo "🚀 Starting Security Monitor API Server..."
echo "📋 Make sure ROS2 is sourced if needed: source /opt/ros/humble/setup.bash"
echo ""

# Check if Python dependencies are installed
if ! python3 -c "import fastapi, uvicorn" 2>/dev/null; then
    echo "⚠️  Installing API dependencies..."
    pip3 install -r requirements-api.txt
fi

# Start the API server
echo "🌐 Starting API server on http://localhost:8000"
echo "📖 API Documentation: http://localhost:8000/docs"
echo "🔌 WebSocket: ws://localhost:8000/ws/threats"
echo ""
echo "Press Ctrl+C to stop the server"
echo ""

python3 standalone_api_server.py
