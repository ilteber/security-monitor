#!/bin/bash
# Test runner script for security monitor

echo "Running Security Monitor tests..."

# Run unit tests
echo "Running unit tests..."
python3 -m pytest tests/test_security_detector.py -v

# Run integration tests
echo "Running integration tests..."
python3 -m pytest tests/test_integration.py -v

# Run scenario tests
echo "Running scenario tests..."
python3 scenarios/carla/highway_scenario.py
python3 scenarios/gazebo/urban_scenario.py

echo "All tests completed!"
