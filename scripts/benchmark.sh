#!/bin/bash
# Benchmark script for security monitor

echo "Running Security Monitor Benchmark..."

# Start the system
echo "Starting security monitor system..."
ros2 launch security_monitor complete_system.launch.py &
SYSTEM_PID=$!

# Wait for system to start
sleep 10

# Run performance tests
echo "Running performance tests..."
python3 -c "
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class Benchmark(Node):
    def __init__(self):
        super().__init__('benchmark')
        self.start_time = time.time()
        self.message_count = 0
        
    def run_benchmark(self):
        # TODO: Implement actual benchmark tests
        print('Benchmark completed')
        print(f'Messages processed: {self.message_count}')
        print(f'Time elapsed: {time.time() - self.start_time:.2f}s')

if __name__ == '__main__':
    rclpy.init()
    benchmark = Benchmark()
    benchmark.run_benchmark()
    rclpy.shutdown()
"

# Stop the system
echo "Stopping security monitor system..."
kill $SYSTEM_PID

echo "Benchmark completed!"
