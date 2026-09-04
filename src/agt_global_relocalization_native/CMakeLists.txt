cmake_minimum_required(VERSION 3.16)
project(agt_global_relocalization_native)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(OpenMP)
find_package(Boost REQUIRED COMPONENTS filesystem system)

# 3D-BBS and small_gicp are pinned external dependencies in dependencies/field_demo.repos.
# Both are intentionally kept outside this repository so their upstream licenses/notices remain clear.
find_path(BBS3D_INCLUDE_DIR cpu_bbs3d/bbs3d.hpp PATHS /usr/local/include /usr/include)
find_library(BBS3D_CPU_LIBRARY cpu_bbs3d PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu)
find_path(SMALL_GICP_INCLUDE_DIR small_gicp/registration/registration_helper.hpp PATHS /usr/local/include /usr/include)
find_library(SMALL_GICP_LIBRARY small_gicp PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu)

if(NOT BBS3D_INCLUDE_DIR OR NOT BBS3D_CPU_LIBRARY)
  message(FATAL_ERROR "3D-BBS CPU library not found. Build KOKIAOKI/3d_bbs with -DBUILD_CUDA=OFF and install it.")
endif()
if(NOT SMALL_GICP_INCLUDE_DIR OR NOT SMALL_GICP_LIBRARY)
  message(FATAL_ERROR "small_gicp helper library not found. Build/install koide3/small_gicp with BUILD_HELPER=ON.")
endif()

add_executable(bbs_gicp_localizer src/bbs_gicp_localizer.cpp)
target_include_directories(bbs_gicp_localizer PRIVATE
  ${BBS3D_INCLUDE_DIR}
  ${SMALL_GICP_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(bbs_gicp_localizer PRIVATE
  ${BBS3D_CPU_LIBRARY}
  ${SMALL_GICP_LIBRARY}
  ${PCL_LIBRARIES}
  Boost::filesystem
  Boost::system
  Eigen3::Eigen
  $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>
)
target_compile_definitions(bbs_gicp_localizer PRIVATE ${PCL_DEFINITIONS})

add_executable(build_relocalization_assets src/build_relocalization_assets.cpp)
target_include_directories(build_relocalization_assets PRIVATE
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

install(TARGETS
  bbs_gicp_localizer
  build_relocalization_assets
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
