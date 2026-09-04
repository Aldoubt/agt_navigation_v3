#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found. Run this on Ubuntu 22.04 / ROS 2 Humble." >&2
  exit 2
fi
source /opt/ros/humble/setup.bash

if [[ -f install/setup.bash ]]; then
  source install/setup.bash
fi

# Livox's upstream build.sh creates package.xml/launch and then clears the whole
# workspace build/install directories. bootstrap_humble.sh prepares these links
# without invoking that destructive wrapper; repeat the harmless preparation
# here so the smoke check is robust when called directly.
if [[ -f src/external/livox_ros_driver2/package_ROS2.xml ]]; then
  ln -sfn package_ROS2.xml src/external/livox_ros_driver2/package.xml
  rm -rf src/external/livox_ros_driver2/launch
  ln -s launch_ROS2 src/external/livox_ros_driver2/launch
fi

have_ros_or_source_pkg() {
  local pkg="$1"
  if command -v colcon >/dev/null 2>&1 && colcon list --names-only 2>/dev/null | grep -qx "$pkg"; then
    return 0
  fi
  ros2 pkg prefix "$pkg" >/dev/null 2>&1
}

required_ros_pkgs=(
  livox_ros_driver2
  batch_lio
  fastlio2
  pgo
  hba
  camera_gimbal_interfaces
  agt_asensing_driver
)

missing=0
for pkg in "${required_ros_pkgs[@]}"; do
  if have_ros_or_source_pkg "$pkg"; then
    echo "PASS ROS dependency: $pkg"
  else
    echo "MISSING ROS dependency: $pkg" >&2
    missing=1
  fi
done

if [[ ! -f /usr/local/include/livox_lidar_api.h && ! -f /usr/include/livox_lidar_api.h ]]; then
  echo "MISSING native dependency: Livox-SDK2 headers" >&2
  missing=1
else
  echo "PASS native dependency: Livox-SDK2 headers"
fi
if ! ldconfig -p 2>/dev/null | grep -q 'liblivox_lidar_sdk'; then
  echo "MISSING native dependency: Livox-SDK2 library" >&2
  missing=1
else
  echo "PASS native dependency: Livox-SDK2 library"
fi

if [[ ! -f /usr/local/include/cpu_bbs3d/bbs3d.hpp && ! -f /usr/include/cpu_bbs3d/bbs3d.hpp ]]; then
  echo "MISSING native dependency: 3D-BBS headers" >&2
  missing=1
else
  echo "PASS native dependency: 3D-BBS headers"
fi
if ! ldconfig -p 2>/dev/null | grep -q 'libcpu_bbs3d'; then
  echo "MISSING native dependency: libcpu_bbs3d" >&2
  missing=1
else
  echo "PASS native dependency: libcpu_bbs3d"
fi
if [[ ! -f /usr/local/lib/cmake/small_gicp/small_gicp-config.cmake \
      && ! -f /usr/lib/cmake/small_gicp/small_gicp-config.cmake \
      && ! -f /usr/local/lib64/cmake/small_gicp/small_gicp-config.cmake ]]; then
  echo "MISSING native dependency: small_gicp CMake config" >&2
  missing=1
else
  echo "PASS native dependency: small_gicp CMake config"
fi

if [[ "$missing" -ne 0 ]]; then
  echo "FIELD BUILD PREFLIGHT FAILED: run scripts/bootstrap_humble.sh first." >&2
  exit 3
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "ERROR: colcon not installed" >&2
  exit 4
fi

# agt_system_bringup declares the complete current software chain, so building
# packages-up-to it is the field acceptance compile boundary. The explicit
# CMake variables are required by the upstream Livox ROS Driver 2 CMakeLists.
colcon build --symlink-install --event-handlers console_direct+ \
  --packages-up-to agt_system_bringup \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

source install/setup.bash

# Pure software tests that should not need sensors.
colcon test --event-handlers console_direct+ \
  --packages-select \
    agt_map_converter \
    agt_map_manager \
    agt_navigation_runtime
colcon test-result --verbose

# Parse launch descriptions without starting hardware. These commands catch
# package discovery/import/launch-file errors; real sensor/action availability
# is checked later by demo_preflight on the robot.
ros2 launch agt_mapping_bringup mapping_mode.launch.py --show-args >/dev/null
ros2 launch agt_mapping_bringup navigation_lio.launch.py --show-args >/dev/null
ros2 launch agt_global_relocalization global_relocalization.launch.py --show-args >/dev/null
ros2 launch agt_system_bringup rviz_field_demo.launch.py --show-args >/dev/null

echo "FIELD BUILD SMOKE PASS"
