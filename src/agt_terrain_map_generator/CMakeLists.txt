cmake_minimum_required(VERSION 3.8)
project(agt_terrain_map_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(urdf REQUIRED)

option(AGT_ENABLE_PATCHWORKPP "Build the Patchwork++ GroundSegmenter adapter when the native package is installed" ON)

add_definitions(${PCL_DEFINITIONS})

add_library(agt_terrain_core SHARED
  src/gravity_level_patch_preprocessor.cpp
  src/terrain_preprocessor.cpp
  src/robot_geometry_loader.cpp
  src/robot_mesh_filter.cpp
  src/point_provenance.cpp
  src/voxel_persistence_filter.cpp
  src/neighborhood_analyzer.cpp
  src/static_confidence.cpp
  src/multi_scale_voxel.cpp
  src/traversability_builder.cpp
  src/pose_loader.cpp
  src/trajectory_carver.cpp
  src/terrain_package_exporter.cpp
  src/median_elevation_builder.cpp
  src/central_difference_slope_builder.cpp
  src/height_obstacle_builder.cpp)
target_include_directories(agt_terrain_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(agt_terrain_core Eigen3::Eigen)
ament_target_dependencies(agt_terrain_core urdf)

add_library(agt_pcl_patch_set_source SHARED
  src/pcl_patch_set_source.cpp)
target_include_directories(agt_pcl_patch_set_source PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(agt_pcl_patch_set_source PRIVATE ${PCL_LIBRARIES})

add_executable(terrain_generator_node src/terrain_generator_node.cpp)
target_include_directories(terrain_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(terrain_generator_node rclcpp)
target_link_libraries(terrain_generator_node
  agt_terrain_core
  agt_pcl_patch_set_source)

add_executable(robot_mesh_filter_test tools/robot_mesh_filter_test.cpp)
target_include_directories(robot_mesh_filter_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS})
target_link_libraries(robot_mesh_filter_test
  agt_terrain_core
  ${PCL_LIBRARIES})

add_executable(voxel_persistence_analyzer tools/voxel_persistence_analyzer.cpp)
target_include_directories(voxel_persistence_analyzer PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS})
target_link_libraries(voxel_persistence_analyzer
  agt_terrain_core
  ${PCL_LIBRARIES})

add_executable(static_confidence_exporter tools/static_confidence_exporter.cpp)
target_include_directories(static_confidence_exporter PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS})
target_link_libraries(static_confidence_exporter
  agt_terrain_core
  ${PCL_LIBRARIES})

set(AGT_TERRAIN_INSTALL_TARGETS
  terrain_generator_node
  robot_mesh_filter_test
  voxel_persistence_analyzer
  static_confidence_exporter
  agt_terrain_core
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_terrain_core test/test_terrain_core.cpp)
  target_include_directories(test_terrain_core PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_terrain_core agt_terrain_core)
  ament_add_gtest(test_static_confidence test/test_static_confidence.cpp)
  target_include_directories(test_static_confidence PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_static_confidence agt_terrain_core)
endif()

ament_export_include_directories(include)
ament_package()
