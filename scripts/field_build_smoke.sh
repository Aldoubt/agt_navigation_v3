#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
repo_parent="$(dirname "${REPO_ROOT}")"
if [[ "$(basename "${repo_parent}")" == "src" ]]; then
  WS_ROOT="$(dirname "${repo_parent}")"
else
  WS_ROOT="$(pwd)"
fi
NATIVE_PREFIX="${AGT_NATIVE_PREFIX:-${WS_ROOT}/.agt_native}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found. Run this on Ubuntu 22.04 / ROS 2 Humble." >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
set -u

if [[ -f install/setup.bash ]]; then
  set +u
  source install/setup.bash
  set -u
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
  ros2_livox_simulation
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

if [[ ! -f "${NATIVE_PREFIX}/include/livox_lidar_api.h" \
      && ! -f /usr/local/include/livox_lidar_api.h \
      && ! -f /usr/include/livox_lidar_api.h ]]; then
  echo "MISSING native dependency: Livox-SDK2 headers" >&2
  missing=1
else
  echo "PASS native dependency: Livox-SDK2 headers"
fi
if [[ ! -f "${NATIVE_PREFIX}/lib/liblivox_lidar_sdk_shared.so" \
      && ! -f "${NATIVE_PREFIX}/lib/liblivox_lidar_sdk.so" ]] \
    && ! ldconfig -p 2>/dev/null | grep -q 'liblivox_lidar_sdk'; then
  echo "MISSING native dependency: Livox-SDK2 library" >&2
  missing=1
else
  echo "PASS native dependency: Livox-SDK2 library"
fi

if [[ ! -f "${NATIVE_PREFIX}/include/cpu_bbs3d/bbs3d.hpp" \
      && ! -f /usr/local/include/cpu_bbs3d/bbs3d.hpp \
      && ! -f /usr/include/cpu_bbs3d/bbs3d.hpp ]]; then
  echo "MISSING native dependency: 3D-BBS headers" >&2
  missing=1
else
  echo "PASS native dependency: 3D-BBS headers"
fi
if [[ ! -f "${NATIVE_PREFIX}/lib/libcpu_bbs3d.so" ]] \
    && ! ldconfig -p 2>/dev/null | grep -q 'libcpu_bbs3d'; then
  echo "MISSING native dependency: libcpu_bbs3d" >&2
  missing=1
else
  echo "PASS native dependency: libcpu_bbs3d"
fi
if [[ ! -f "${NATIVE_PREFIX}/lib/cmake/small_gicp/small_gicp-config.cmake" \
      && ! -f "${NATIVE_PREFIX}/lib64/cmake/small_gicp/small_gicp-config.cmake" \
      && ! -f /usr/local/lib/cmake/small_gicp/small_gicp-config.cmake \
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
# packages-up-to it plus the Gazebo harness is the migration compile boundary.
# The explicit CMake variables are required by the upstream Livox ROS Driver 2
# CMakeLists and the workspace-local native prefix.
export CMAKE_PREFIX_PATH="${NATIVE_PREFIX}:${CMAKE_PREFIX_PATH:-}"
colcon build --symlink-install --event-handlers console_direct+ \
  --packages-up-to agt_system_bringup agt_gazebo_sim \
  --cmake-args -DCMAKE_PREFIX_PATH="${NATIVE_PREFIX}:${CMAKE_PREFIX_PATH}" \
    -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

set +u
source install/setup.bash
set -u

# Pure software tests that should not need sensors.
colcon test --event-handlers console_direct+ \
  --packages-select \
    agt_base_control \
    agt_map_converter \
    agt_map_manager \
    agt_navigation_runtime
colcon test-result --verbose

# These ament_python packages currently do not register their pytest modules
# with colcon, so run the behavior tests explicitly. This keeps the migration
# gate meaningful instead of accepting a misleading "0 tests" result.
python3 -m pytest -q \
  "${REPO_ROOT}/src/agt_map_converter/test/test_converter.py" \
  "${REPO_ROOT}/src/agt_navigation_runtime/test/test_stationary_motion.py"

# Parse launch descriptions without starting hardware. These commands catch
# package discovery/import/launch-file errors; real sensor/action availability
# is checked later by demo_preflight on the robot.
ros2 launch agt_mapping_bringup mapping_mode.launch.py --show-args >/dev/null
ros2 launch agt_mapping_bringup navigation_lio.launch.py --show-args >/dev/null
ros2 launch agt_global_relocalization global_relocalization.launch.py --show-args >/dev/null
ros2 launch agt_system_bringup rviz_field_demo.launch.py --show-args >/dev/null
ros2 launch agt_gazebo_sim mapping_demo.launch.py --show-args >/dev/null
ros2 launch agt_gazebo_sim navigation_demo.launch.py --show-args >/dev/null

# Safety behavior is a migration gate too: LOST must hard-stop and reopening the
# localization gate must not replay old velocity intent.
ROS_DOMAIN_ID="${AGT_SMOKE_DOMAIN_ID:-149}" python3 \
  "${REPO_ROOT}/src/agt_base_control/test/guard_fail_closed_acceptance.py" \
  --ros-args --params-file \
  "${REPO_ROOT}/src/agt_base_control/config/cmd_vel_guard.yaml"

echo "FIELD BUILD SMOKE PASS"
