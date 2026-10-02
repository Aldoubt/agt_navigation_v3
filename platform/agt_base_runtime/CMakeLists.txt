cmake_minimum_required(VERSION 3.8)
project(agt_base_runtime)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(agt_robot_interfaces REQUIRED)

add_library(motion_guard_core src/motion_guard_core.cpp)
target_include_directories(motion_guard_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(motion_guard src/motion_guard_node.cpp)
target_include_directories(motion_guard PRIVATE include)
target_link_libraries(motion_guard motion_guard_core)
ament_target_dependencies(motion_guard rclcpp geometry_msgs std_msgs agt_robot_interfaces)

install(TARGETS motion_guard_core motion_guard
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_motion_guard_core test/test_motion_guard_core.cpp)
  target_link_libraries(test_motion_guard_core motion_guard_core)
  target_include_directories(test_motion_guard_core PRIVATE include)

  ament_add_pytest_test(test_shadow_parity test/test_shadow_parity.py TIMEOUT 90)
  set_tests_properties(test_shadow_parity PROPERTIES
    ENVIRONMENT "MOTION_GUARD_CPP_EXECUTABLE=$<TARGET_FILE:motion_guard>")
endif()

ament_export_include_directories(include)
ament_export_libraries(motion_guard_core)
ament_export_dependencies(rclcpp geometry_msgs std_msgs agt_robot_interfaces)
ament_package()
