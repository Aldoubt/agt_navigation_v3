cmake_minimum_required(VERSION 3.16)
project(agt_local_row_perception LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
option(AGT_LOCAL_ROW_STANDALONE "Build the ROS-independent core and offline CLI only" OFF)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(row_perception_core src/core.cpp src/config.cpp)
target_include_directories(row_perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(row_perception_core PUBLIC Eigen3::Eigen yaml-cpp)
add_executable(local_row_offline src/offline.cpp)
target_link_libraries(local_row_offline PRIVATE row_perception_core)

if(AGT_LOCAL_ROW_STANDALONE)
  install(TARGETS row_perception_core local_row_offline
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  install(DIRECTORY include/ DESTINATION include)
  return()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(agt_navigation_interfaces REQUIRED)
add_executable(local_row_perception src/row_node.cpp)
target_link_libraries(local_row_perception row_perception_core)
ament_target_dependencies(local_row_perception rclcpp sensor_msgs nav_msgs geometry_msgs
  visualization_msgs diagnostic_msgs tf2 tf2_ros agt_navigation_interfaces)

install(TARGETS row_perception_core local_row_offline local_row_perception
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 yaml-cpp)
ament_package()
