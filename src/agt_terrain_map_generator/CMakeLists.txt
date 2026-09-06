cmake_minimum_required(VERSION 3.8)
project(agt_terrain_map_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

option(AGT_ENABLE_PATCHWORKPP "Build the Patchwork++ GroundSegmenter adapter when the native package is installed" ON)

add_library(agt_pcl_patch_set_source SHARED
  src/pcl_patch_set_source.cpp)
target_include_directories(agt_pcl_patch_set_source PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(agt_pcl_patch_set_source PRIVATE ${PCL_LIBRARIES})
target_compile_definitions(agt_pcl_patch_set_source PRIVATE ${PCL_DEFINITIONS})

add_executable(terrain_generator_node src/terrain_generator_node.cpp)
target_include_directories(terrain_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(terrain_generator_node rclcpp)
target_link_libraries(terrain_generator_node PRIVATE agt_pcl_patch_set_source)

set(AGT_TERRAIN_INSTALL_TARGETS
  terrain_generator_node
  agt_pcl_patch_set_source)

if(AGT_ENABLE_PATCHWORKPP)
  find_package(patchworkpp QUIET CONFIG)
  if(patchworkpp_FOUND)
    add_library(agt_patchworkpp_ground_segmenter SHARED
      plugins/ground_segmenters/patchworkpp_ground_segmenter.cpp)
    target_include_directories(agt_patchworkpp_ground_segmenter PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    target_compile_features(agt_patchworkpp_ground_segmenter PRIVATE cxx_std_20)
    target_link_libraries(agt_patchworkpp_ground_segmenter PRIVATE patchworkpp::ground_seg_cores)
    list(APPEND AGT_TERRAIN_INSTALL_TARGETS agt_patchworkpp_ground_segmenter)
    message(STATUS "AGT terrain: Patchwork++ adapter enabled")
  else()
    message(STATUS "AGT terrain: Patchwork++ not found; core terrain interfaces build without the adapter")
  endif()
endif()

install(TARGETS ${AGT_TERRAIN_INSTALL_TARGETS}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
