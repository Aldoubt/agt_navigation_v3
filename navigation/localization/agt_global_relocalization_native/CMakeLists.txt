cmake_minimum_required(VERSION 3.16)
project(agt_global_relocalization_native)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(OpenMP)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(small_gicp CONFIG REQUIRED)

set(AGT_NATIVE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Acceptance/manual-seed localization only needs small_gicp. 3D-BBS is kept as
# an optional capability so a machine can build/run the acceptance stack even
# when the global-search native dependency has not been installed yet.
find_path(
  BBS3D_INCLUDE_DIR
  cpu_bbs3d/bbs3d.hpp
  HINTS ENV CMAKE_PREFIX_PATH
  PATHS /usr/local/include /usr/include
)
find_library(
  BBS3D_CPU_LIBRARY
  cpu_bbs3d
  HINTS ENV CMAKE_PREFIX_PATH
  PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
)

set(AGT_HAVE_3D_BBS FALSE)
if(BBS3D_INCLUDE_DIR AND BBS3D_CPU_LIBRARY)
  set(AGT_HAVE_3D_BBS TRUE)
  message(STATUS "3D-BBS CPU support enabled: ${BBS3D_CPU_LIBRARY}")
else()
  message(WARNING
    "3D-BBS CPU library not found. Building acceptance/manual-seed GICP "
    "targets only. Global BBS relocalization and BBS asset generation will "
    "be unavailable until KOKIAOKI/3d_bbs is installed.")
endif()

add_executable(build_relocalization_candidates src/build_relocalization_candidates.cpp)
target_include_directories(build_relocalization_candidates PRIVATE
  ${AGT_NATIVE_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(build_relocalization_candidates PRIVATE
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
target_compile_definitions(build_relocalization_candidates PRIVATE ${PCL_DEFINITIONS})

add_executable(map_gicp_tracker src/map_gicp_tracker.cpp)
target_include_directories(map_gicp_tracker PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(map_gicp_tracker PRIVATE
  small_gicp::small_gicp
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
target_compile_definitions(map_gicp_tracker PRIVATE ${PCL_DEFINITIONS})

set(AGT_NATIVE_TARGETS
  build_relocalization_candidates
  map_gicp_tracker
)

if(AGT_HAVE_3D_BBS)
  # 3D-BBS upstream does not export a CMake package config, so these targets
  # retain the explicit include/library lookup and workspace runtime path.
  get_filename_component(AGT_NATIVE_LIBRARY_DIR "${BBS3D_CPU_LIBRARY}" DIRECTORY)
  set(AGT_NATIVE_BUILD_RPATH "${AGT_NATIVE_LIBRARY_DIR}")
  set(AGT_NATIVE_INSTALL_RPATH "\$ORIGIN/../../../../.agt_native/lib")

  function(agt_set_native_runtime_path target_name)
    set_target_properties(${target_name} PROPERTIES
      BUILD_RPATH "${AGT_NATIVE_BUILD_RPATH}"
      INSTALL_RPATH "${AGT_NATIVE_INSTALL_RPATH}"
    )
  endfunction()

  add_executable(bbs_gicp_localizer src/bbs_gicp_localizer.cpp)
  target_include_directories(bbs_gicp_localizer PRIVATE
    ${AGT_NATIVE_INCLUDE_DIR}
    ${BBS3D_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(bbs_gicp_localizer PRIVATE
    ${BBS3D_CPU_LIBRARY}
    small_gicp::small_gicp
    ${PCL_LIBRARIES}
    Boost::filesystem
    Boost::system
    Eigen3::Eigen
    $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>
  )
  target_compile_definitions(bbs_gicp_localizer PRIVATE ${PCL_DEFINITIONS})

  add_executable(candidate_bbs_gicp_localizer src/candidate_bbs_gicp_localizer.cpp)
  target_include_directories(candidate_bbs_gicp_localizer PRIVATE
    ${AGT_NATIVE_INCLUDE_DIR}
    ${BBS3D_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(candidate_bbs_gicp_localizer PRIVATE
    ${BBS3D_CPU_LIBRARY}
    small_gicp::small_gicp
    ${PCL_LIBRARIES}
    Boost::filesystem
    Boost::system
    Eigen3::Eigen
    $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>
  )
  target_compile_definitions(candidate_bbs_gicp_localizer PRIVATE ${PCL_DEFINITIONS})

  add_executable(build_relocalization_assets src/build_relocalization_assets.cpp)
  target_include_directories(build_relocalization_assets PRIVATE
    ${AGT_NATIVE_INCLUDE_DIR}
    ${BBS3D_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(build_relocalization_assets PRIVATE
    ${BBS3D_CPU_LIBRARY}
    ${PCL_LIBRARIES}
    Boost::filesystem
    Boost::system
    Eigen3::Eigen
    $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>
  )
  target_compile_definitions(build_relocalization_assets PRIVATE ${PCL_DEFINITIONS})

  agt_set_native_runtime_path(bbs_gicp_localizer)
  agt_set_native_runtime_path(candidate_bbs_gicp_localizer)
  agt_set_native_runtime_path(build_relocalization_assets)

  list(APPEND AGT_NATIVE_TARGETS
    bbs_gicp_localizer
    candidate_bbs_gicp_localizer
    build_relocalization_assets
  )
endif()

install(
  TARGETS ${AGT_NATIVE_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/bbs DESTINATION share/${PROJECT_NAME}/config)
ament_package()
