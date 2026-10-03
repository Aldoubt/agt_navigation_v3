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
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(bunker_msgs REQUIRED)
find_package(agt_robot_interfaces REQUIRED)
find_package(agt_navigation_interfaces REQUIRED)

add_library(motion_guard_core src/motion_guard_core.cpp)
target_include_directories(motion_guard_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(motion_guard src/motion_guard_node.cpp)
target_include_directories(motion_guard PRIVATE include)
target_link_libraries(motion_guard motion_guard_core)
ament_target_dependencies(motion_guard rclcpp geometry_msgs std_msgs agt_robot_interfaces agt_navigation_interfaces)

add_executable(epoch_velocity_smoother src/epoch_velocity_smoother.cpp)
ament_target_dependencies(epoch_velocity_smoother rclcpp geometry_msgs agt_navigation_interfaces)

add_library(base_adapter_core
  src/bunker_adapter.cpp
  src/ackermann_adapter.cpp)
target_include_directories(base_adapter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(base_adapter_core geometry_msgs nav_msgs)

add_executable(bunker_adapter src/bunker_adapter_node.cpp)
target_include_directories(bunker_adapter PRIVATE include)
target_link_libraries(bunker_adapter base_adapter_core)
ament_target_dependencies(
  bunker_adapter rclcpp geometry_msgs nav_msgs diagnostic_msgs bunker_msgs)

add_executable(tracked_twist_adapter src/tracked_twist_adapter_node.cpp)
ament_target_dependencies(tracked_twist_adapter rclcpp geometry_msgs nav_msgs agt_robot_interfaces)
install(TARGETS tracked_twist_adapter DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/driver_state_normalizer.py DESTINATION lib/${PROJECT_NAME})

install(TARGETS motion_guard_core motion_guard epoch_velocity_smoother base_adapter_core bunker_adapter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_motion_guard_core test/test_motion_guard_core.cpp)
  target_link_libraries(test_motion_guard_core motion_guard_core)
  target_include_directories(test_motion_guard_core PRIVATE include)

  ament_add_gtest(test_base_adapter_core test/test_base_adapter_core.cpp)
  target_link_libraries(test_base_adapter_core base_adapter_core)
  target_include_directories(test_base_adapter_core PRIVATE include)
  ament_target_dependencies(test_base_adapter_core geometry_msgs nav_msgs)

  ament_add_pytest_test(test_shadow_parity test/test_shadow_parity.py TIMEOUT 90)
  set_tests_properties(test_shadow_parity PROPERTIES
    ENVIRONMENT "MOTION_GUARD_CPP_EXECUTABLE=$<TARGET_FILE:motion_guard>;ROS_DOMAIN_ID=217")

  ament_add_pytest_test(
    test_bunker_adapter_integration test/test_bunker_adapter_integration.py TIMEOUT 60)
  set_tests_properties(test_bunker_adapter_integration PROPERTIES
    ENVIRONMENT "BUNKER_ADAPTER_EXECUTABLE=$<TARGET_FILE:bunker_adapter>;ROS_DOMAIN_ID=216")
endif()

ament_export_include_directories(include)
ament_export_libraries(motion_guard_core base_adapter_core)
ament_export_dependencies(
  rclcpp geometry_msgs nav_msgs std_msgs diagnostic_msgs agt_robot_interfaces agt_navigation_interfaces)
ament_package()
