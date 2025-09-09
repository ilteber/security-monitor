#!/bin/bash

# ROS2 Node Runner Script
# Fixes Python version conflicts between Conda and ROS2

# Exit on any error
set -e

# Check if node script is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <node_script.py>"
    echo "Example: $0 security_detector.py"
    exit 1
fi

NODE_SCRIPT=$1

# Check if the script exists
if [ ! -f "src/security_monitor/security_monitor/$NODE_SCRIPT" ]; then
    echo "Error: Node script '$NODE_SCRIPT' not found in src/security_monitor/security_monitor/"
    exit 1
fi

echo "Starting ROS2 node: $NODE_SCRIPT"

# Deactivate conda environment if active
if [ ! -z "$CONDA_DEFAULT_ENV" ]; then
    echo "Deactivating conda environment: $CONDA_DEFAULT_ENV"
    # Use conda deactivate if available, otherwise unset variables
    if command -v conda &> /dev/null; then
        conda deactivate 2>/dev/null || true
    fi
    unset CONDA_DEFAULT_ENV
    unset CONDA_PREFIX
    unset CONDA_PYTHON_EXE
    unset CONDA_PROMPT_MODIFIER
fi

# Source ROS2 environment
echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

# Source workspace
echo "Sourcing workspace..."
source install/setup.bash

# Set Python path to use system Python 3.10
export PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH"

# Run the node with Python 3.10
echo "Running node with Python 3.10..."
/usr/bin/python3.10 src/security_monitor/security_monitor/$NODE_SCRIPT
