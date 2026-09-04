cmake_minimum_required(VERSION 3.8)
project(agt_mapping_bringup)
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config scripts DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/mid360_imu_preflight.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
