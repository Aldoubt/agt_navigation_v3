cmake_minimum_required(VERSION 3.8)
project(agt_terrain_map_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(terrain_generator_node src/terrain_generator_node.cpp)
target_include_directories(terrain_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(terrain_generator_node rclcpp)

install(TARGETS terrain_generator_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
