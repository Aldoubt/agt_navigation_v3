cmake_minimum_required(VERSION 3.8)
project(agt_pointcloud_preprocessor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(obstacle_cloud_node src/obstacle_cloud_node.cpp)
ament_target_dependencies(obstacle_cloud_node
  rclcpp sensor_msgs tf2 tf2_ros geometry_msgs)

install(TARGETS obstacle_cloud_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
