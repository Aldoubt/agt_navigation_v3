#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="/usr/local:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
export CMAKE_BUILD_PARALLEL_LEVEL=2
cd /opt/nav_ws
colcon build --executor sequential --packages-select livox_ros_driver2 interface fastlio2 \
  agt_robot_interfaces agt_map_converter agt_map_manager agt_livox_tools agt_fastlio_adapter \
  agt_global_relocalization_native agt_global_relocalization agt_localization_manager \
  agt_map_tracker agt_batch_lio_adapter agt_navigation_runtime agt_nav2_bringup agt_pointcloud_preprocessor \
  agt_base_control agt_mission_executor \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
# Mapping uses its own patched FAST-LIO2/PGO revision and driver version, separately built.
# Do not source the navigation overlay into this build.
cd /opt/mapping_ws
colcon build --executor sequential --packages-select livox_ros_driver2 interface fastlio2 pgo \
  agt_mapping_backend_api agt_mapping_frontend_api agt_mapping_interfaces agt_map_refinement_core agt_mapping_core agt_mid360_adapter agt_fastlio_backend agt_pgo_backend \
  agt_mapping_artifacts agt_mapping_exporter agt_pcd2grid_exporter agt_map_studio agt_mapping_bringup \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source /opt/nav_ws/install/setup.bash
cmake -S /opt/hmi -B /opt/hmi_build -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_TEST=OFF
cmake --build /opt/hmi_build --parallel 2
cmake -S /opt/hmi/tests/field -B /opt/hmi_field_tests
cmake --build /opt/hmi_field_tests --parallel 2
ctest --test-dir /opt/hmi_field_tests --output-on-failure
# Numeric native dependencies must be present; missing BBS is not a valid appliance build.
test -x /opt/nav_ws/install/agt_global_relocalization_native/lib/agt_global_relocalization_native/candidate_bbs_gicp_localizer
test -x /opt/mapping_ws/install/agt_map_studio/lib/agt_map_studio/map_viewer
