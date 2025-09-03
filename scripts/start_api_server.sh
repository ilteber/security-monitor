#!/bin/bash

# Security Monitor API Server Startup Script
# Author: Ilteber Ayvaci
# Email: ayvaci.ilteber@gmail.com

set -e

# Configuration
API_HOST=${API_HOST:-"0.0.0.0"}
API_PORT=${API_PORT:-8000}
LOG_LEVEL=${LOG_LEVEL:-"info"}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚗 Security Monitor API Server${NC}"
echo -e "${BLUE}==============================${NC}"

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}⚠️  ROS2 not sourced. Attempting to source...${NC}"
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}✅ ROS2 Humble sourced${NC}"
    elif [ -f "/opt/ros/galactic/setup.bash" ]; then
        source /opt/ros/galactic/setup.bash
        echo -e "${GREEN}✅ ROS2 Galactic sourced${NC}"
    else
        echo -e "${RED}❌ ROS2 not found. Please install and source ROS2${NC}"
        exit 1
    fi
fi

# Check if workspace is built
if [ ! -f "install/setup.bash" ]; then
    echo -e "${YELLOW}⚠️  Workspace not built. Building...${NC}"
    colcon build --packages-select security_monitor
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Workspace built successfully${NC}"
    else
        echo -e "${RED}❌ Build failed${NC}"
        exit 1
    fi
fi

# Source workspace
source install/setup.bash
echo -e "${GREEN}✅ Workspace sourced${NC}"

# Check dependencies
echo -e "${BLUE}🔍 Checking dependencies...${NC}"

# Check Python dependencies
python3 -c "import fastapi, uvicorn, aiohttp" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}⚠️  Python dependencies missing. Installing...${NC}"
    pip3 install -r requirements.txt
fi

# Check if port is available
if lsof -Pi :$API_PORT -sTCP:LISTEN -t >/dev/null 2>&1; then
    echo -e "${RED}❌ Port $API_PORT is already in use${NC}"
    echo -e "${YELLOW}💡 Try a different port: API_PORT=8001 $0${NC}"
    exit 1
fi

# Set ROS domain ID
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
echo -e "${GREEN}✅ ROS_DOMAIN_ID set to $ROS_DOMAIN_ID${NC}"

# Display configuration
echo -e "${BLUE}📋 Configuration:${NC}"
echo -e "   Host: $API_HOST"
echo -e "   Port: $API_PORT"
echo -e "   Log Level: $LOG_LEVEL"
echo -e "   ROS Domain ID: $ROS_DOMAIN_ID"

echo -e "${BLUE}🚀 Starting API Server...${NC}"
echo -e "${GREEN}📖 API Documentation: http://$API_HOST:$API_PORT/docs${NC}"
echo -e "${GREEN}🔌 WebSocket: ws://$API_HOST:$API_PORT/ws/threats${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop${NC}"

# Start the API server
ros2 launch security_monitor api_server.launch.py \
    host:=$API_HOST \
    port:=$API_PORT \
    log_level:=$LOG_LEVEL
