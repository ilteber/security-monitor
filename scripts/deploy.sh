#!/bin/bash
# Deployment script for security monitor

echo "Deploying Security Monitor System..."

# Build the workspace
echo "Building workspace..."
colcon build --symlink-install

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "Build successful!"
else
    echo "Build failed!"
    exit 1
fi

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

# Run tests
echo "Running tests..."
./scripts/run_tests.sh

# Check if tests passed
if [ $? -eq 0 ]; then
    echo "Tests passed!"
else
    echo "Tests failed!"
    exit 1
fi

echo "Deployment completed successfully!"
echo "To start the system, run:"
echo "ros2 launch security_monitor complete_system.launch.py"
