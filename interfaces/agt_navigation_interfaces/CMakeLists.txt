cmake_minimum_required(VERSION 3.8)
project(agt_navigation_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavigationHealth.msg"
  "msg/LocalRowState.msg"
  "msg/OdomQuality.msg"
  "msg/GlobalQuality.msg"
  "msg/RelocalizationCandidate.msg"
  "msg/RecoveryStatus.msg"
  "msg/NavigationMode.msg"
  "msg/RouteSegment.msg"
  "msg/TaskContext.msg"
  "msg/VelocityCommand.msg"
  "action/ExecuteTaskSegment.action"
  "action/NavigateTo.action"
  "action/FollowRoute.action"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
