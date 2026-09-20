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

add_executable(agt_pointcloud_preprocessor src/obstacle_cloud_node.cpp)
target_include_directories(agt_pointcloud_preprocessor PRIVATE include)
ament_target_dependencies(agt_pointcloud_preprocessor
  rclcpp sensor_msgs tf2 tf2_ros geometry_msgs)

install(TARGETS agt_pointcloud_preprocessor
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../config/perception.yaml
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_radial_ground_filter test/test_radial_ground_filter.cpp)
  target_include_directories(test_radial_ground_filter PRIVATE include)
endif()

ament_package()
