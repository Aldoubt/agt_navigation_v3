cmake_minimum_required(VERSION 3.8)
project(agt_livox_tools)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(livox_format_bridge src/livox_format_bridge.cpp)
ament_target_dependencies(livox_format_bridge rclcpp sensor_msgs livox_ros_driver2)

install(TARGETS livox_format_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
