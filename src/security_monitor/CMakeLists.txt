cmake_minimum_required(VERSION 3.8)
project(security_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  security_monitor/security_detector.py
  security_monitor/sensor_monitor.py
  security_monitor/threat_classifier.py
  security_monitor/emergency_handler.py
  security_monitor/ml_detector.py
  security_monitor/sensor_fusion.py
  security_monitor/model_manager.py
  security_monitor/weather_monitor.py
  security_monitor/performance_monitor.py
  security_monitor/data_logger.py
  security_monitor/health_monitor.py
  security_monitor/analytics.py
  security_monitor/notification_system.py
  security_monitor/optimizer.py
  security_monitor/error_handler.py
  security_monitor/final_integration.py
  security_monitor/api/server.py
  security_monitor/api_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
