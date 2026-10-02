cmake_minimum_required(VERSION 3.8)
project(agt_system_bringup)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(agt_tf_authority_probe src/tf_authority_probe.cpp)
ament_target_dependencies(agt_tf_authority_probe rclcpp tf2_msgs)

install(TARGETS agt_tf_authority_probe DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/robot.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/navigation.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/controller.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/costmap.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/perception.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/safety.yaml
  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../config/navigation_profiles
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_ownership_static
    test/test_launch_ownership_static.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_nav_profile_contract
    test/test_nav_profile_contract.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_yhs_nav_config_guard
    test/test_yhs_nav_config_guard.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
