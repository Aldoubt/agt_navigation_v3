#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

WS_ROOT=""
DO_APT=1
DO_NATIVE=1
DO_BUILD=1
RUN_SMOKE=0
FORCE_NATIVE=0

usage() {
  cat <<'EOF'
Usage: bootstrap_humble.sh [options]

Bootstrap the current AGT RViz field-demo workspace after cloning agt_navigation_v3.

Options:
  --workspace PATH   ROS 2 workspace root. Auto-detected when repo is under <ws>/src/.
  --no-apt           Do not install Ubuntu/ROS apt dependencies.
  --no-native        Do not build/install Livox-SDK2, 3D-BBS or small_gicp.
  --no-build         Only fetch/install dependencies; do not colcon build.
  --smoke            Run scripts/field_build_smoke.sh after bootstrap build.
  --force-native     Rebuild native /usr/local dependencies even if already present.
  -h, --help         Show this help.

Base prerequisite:
  Ubuntu 22.04 with ROS 2 Humble already installed at /opt/ros/humble.

Typical first use:
  mkdir -p ~/agt_ws/src
  git clone https://github.com/Aldoubt/agt_navigation_v3.git ~/agt_ws/src/agt_navigation_v3
  cd ~/agt_ws
  bash src/agt_navigation_v3/scripts/bootstrap_humble.sh --smoke
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      WS_ROOT="${2:?--workspace requires a path}"
      shift 2
      ;;
    --no-apt) DO_APT=0; shift ;;
    --no-native) DO_NATIVE=0; shift ;;
    --no-build) DO_BUILD=0; shift ;;
    --smoke) RUN_SMOKE=1; shift ;;
    --force-native) FORCE_NATIVE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "${WS_ROOT}" ]]; then
  repo_parent="$(dirname "${REPO_ROOT}")"
  if [[ "$(basename "${repo_parent}")" == "src" ]]; then
    WS_ROOT="$(dirname "${repo_parent}")"
  else
    WS_ROOT="${AGT_WS:-${HOME}/agt_ws}"
  fi
fi
WS_ROOT="$(realpath -m "${WS_ROOT}")"
SRC_DIR="${WS_ROOT}/src"
mkdir -p "${SRC_DIR}"

EXPECTED_REPO="${SRC_DIR}/agt_navigation_v3"
if [[ "$(realpath -m "${REPO_ROOT}")" != "$(realpath -m "${EXPECTED_REPO}")" ]]; then
  if [[ -e "${EXPECTED_REPO}" || -L "${EXPECTED_REPO}" ]]; then
    echo "ERROR: ${EXPECTED_REPO} already exists and is not this checkout." >&2
    exit 3
  fi
  ln -s "${REPO_ROOT}" "${EXPECTED_REPO}"
  echo "Linked main repository into workspace: ${EXPECTED_REPO} -> ${REPO_ROOT}"
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  cat >&2 <<'EOF'
ERROR: ROS 2 Humble was not found at /opt/ros/humble/setup.bash.
Install the Ubuntu 22.04 ROS 2 Humble base first, then rerun this script.
The bootstrap intentionally does not rewrite the machine's ROS apt repository automatically.
EOF
  exit 4
fi
source /opt/ros/humble/setup.bash

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
    echo "WARNING: validated target is Ubuntu 22.04; detected ${ID:-unknown} ${VERSION_ID:-unknown}." >&2
  fi
fi

if [[ "${DO_APT}" -eq 1 ]]; then
  echo "==> Installing Ubuntu / ROS build dependencies"
  sudo apt-get update
  sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y \
    software-properties-common git ca-certificates curl \
    build-essential cmake ninja-build pkg-config \
    python3-pip python3-yaml python3-rosdep \
    python3-colcon-common-extensions \
    libeigen3-dev libpcl-dev libyaml-cpp-dev libboost-all-dev libtbb-dev \
    ros-humble-gtsam \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-pcl-conversions ros-humble-tf2-eigen

  # python3-vcstool lives in Ubuntu Universe on Jammy. ROS installation normally
  # enables Universe already, but make first-boot behavior explicit and provide
  # a pip fallback for minimal images.
  sudo add-apt-repository -y universe >/dev/null
  sudo apt-get update
  if ! sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y python3-vcstool; then
    python3 -m pip install --user vcstool
    export PATH="${HOME}/.local/bin:${PATH}"
  fi
fi

if ! command -v vcs >/dev/null 2>&1; then
  echo "ERROR: vcs is missing. Install vcstool or rerun without --no-apt." >&2
  exit 5
fi
if ! command -v rosdep >/dev/null 2>&1; then
  echo "ERROR: rosdep is missing. Install python3-rosdep or rerun without --no-apt." >&2
  exit 5
fi
if ! command -v colcon >/dev/null 2>&1; then
  echo "ERROR: colcon is missing. Install python3-colcon-common-extensions." >&2
  exit 5
fi

REPOS_FILE="${REPO_ROOT}/dependencies/field_demo.repos"
echo "==> Validating source dependency manifest"
vcs validate < "${REPOS_FILE}"

echo "==> Importing missing field-demo repositories into ${SRC_DIR}"
# Repeatable and non-destructive: existing repositories and local changes are not overwritten.
vcs import --skip-existing "${SRC_DIR}" < "${REPOS_FILE}"

prepare_livox_ros2_source() {
  local driver="${SRC_DIR}/external/livox_ros_driver2"
  if [[ ! -f "${driver}/package_ROS2.xml" ]]; then
    echo "ERROR: Livox ROS Driver 2 source is incomplete: ${driver}" >&2
    exit 6
  fi
  # Official build.sh also deletes workspace-level build/install. Only create
  # the ROS2 source links here and keep normal colcon ownership of the workspace.
  ln -sfn package_ROS2.xml "${driver}/package.xml"
  rm -rf "${driver}/launch"
  ln -s launch_ROS2 "${driver}/launch"
}
prepare_livox_ros2_source

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  echo "==> Initializing rosdep"
  sudo rosdep init
fi
echo "==> Updating rosdep"
rosdep update --rosdistro humble

echo "==> Installing package.xml dependencies"
# PGO uses uppercase GTSAM as a rosdep key; the actual library is installed above
# from the ROS Humble binary package.
rosdep install --from-paths "${SRC_DIR}" --ignore-src -r -y \
  --rosdistro humble --skip-keys "GTSAM"

install_native() {
  local name="$1"
  local present_cmd="$2"
  local source_dir="$3"
  shift 3

  if [[ "${FORCE_NATIVE}" -eq 0 ]] && eval "${present_cmd}"; then
    echo "PASS native dependency already installed: ${name}"
    return 0
  fi

  echo "==> Building native dependency: ${name}"
  cmake -S "${source_dir}" -B "${source_dir}/build-agt" "$@"
  cmake --build "${source_dir}/build-agt" --parallel "$(nproc)"
  sudo cmake --build "${source_dir}/build-agt" --target install
}

if [[ "${DO_NATIVE}" -eq 1 ]]; then
  install_native \
    "Livox-SDK2" \
    "test -f /usr/local/include/livox_lidar_api.h && ldconfig -p 2>/dev/null | grep -q liblivox_lidar_sdk" \
    "${SRC_DIR}/external/Livox-SDK2" \
    -DCMAKE_BUILD_TYPE=Release

  install_native \
    "3D-BBS CPU" \
    "test -f /usr/local/include/cpu_bbs3d/bbs3d.hpp && ldconfig -p 2>/dev/null | grep -q libcpu_bbs3d" \
    "${SRC_DIR}/external/3d_bbs" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_CUDA=OFF

  install_native \
    "small_gicp" \
    "test -f /usr/local/lib/cmake/small_gicp/small_gicp-config.cmake || test -f /usr/local/lib64/cmake/small_gicp/small_gicp-config.cmake" \
    "${SRC_DIR}/external/small_gicp" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_HELPER=ON \
    -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF

  sudo ldconfig
fi

if [[ "${DO_BUILD}" -eq 1 ]]; then
  echo "==> Building current field-demo software chain"
  cd "${WS_ROOT}"
  colcon build --symlink-install --event-handlers console_direct+ \
    --packages-up-to agt_system_bringup \
    --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
  source "${WS_ROOT}/install/setup.bash"
fi

if [[ "${RUN_SMOKE}" -eq 1 ]]; then
  echo "==> Running field build smoke"
  cd "${WS_ROOT}"
  bash "${REPO_ROOT}/scripts/field_build_smoke.sh"
fi

cat <<EOF

BOOTSTRAP PASS
workspace: ${WS_ROOT}

Next:
  source /opt/ros/humble/setup.bash
  source ${WS_ROOT}/install/setup.bash

For the current RViz field acceptance flow see:
  ${REPO_ROOT}/docs/RVIZ_FIELD_ACCEPTANCE.md
EOF
