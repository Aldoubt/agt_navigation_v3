cmake_minimum_required(VERSION 3.8)
project(agt_nav2_bringup)
find_package(ament_cmake REQUIRED)
# The six canonical Nav2 YAML files are installed below; this package has no
# tracked config/ directory of its own (an empty local dir hid that in-place).
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/robot.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/navigation.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/controller.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/costmap.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/perception.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/safety.yaml
  DESTINATION share/${PROJECT_NAME}/config)
ament_package()
