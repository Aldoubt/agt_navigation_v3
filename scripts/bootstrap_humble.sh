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
  --no-apt           Skip apt/rosdep network installation; use preinstalled system dependencies.
  --no-native        Do not build/install Livox-SDK2, 3D-BBS or small_gicp.
  --no-build         Only fetch/install dependencies; do not colcon build.
  --smoke            Run scripts/field_build_smoke.sh after bootstrap build.
  --force-native     Rebuild workspace-local native dependencies even if already present.
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
NATIVE_PREFIX="${WS_ROOT}/.agt_native"
mkdir -p "${SRC_DIR}"

ensure_single_livox_source() {
  local canonical="${SRC_DIR}/external/livox_ros_driver2"
  local legacy="${SRC_DIR}/livox_ros_driver2"
  # The repos manifest owns the canonical external/ checkout. A second clone
  # under src/ changes package discovery and can reuse the wrong CMake cache.
  # Fail early instead of importing another copy or silently choosing one.
  if [[ -d "${legacy}" ]]; then
    if [[ -d "${canonical}" ]]; then
      echo "ERROR: duplicate livox_ros_driver2 sources detected:" >&2
      echo "  canonical: ${canonical}" >&2
      echo "  legacy:    ${legacy}" >&2
      echo "Remove the legacy checkout, then rerun bootstrap_humble.sh." >&2
      exit 6
    fi
    echo "ERROR: livox_ros_driver2 exists at legacy path ${legacy}." >&2
    echo "Move it to ${canonical} or remove it before bootstrap." >&2
    exit 6
  fi
}

ensure_single_livox_source

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
# Humble's setup scripts read optional variables that are unset in a fresh
# shell; source them with nounset temporarily disabled.
set +u
source /opt/ros/humble/setup.bash
set -u

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
    python3-pip python3-yaml python3-pytest python3-rosdep \
    python3-colcon-common-extensions \
    libeigen3-dev libpcl-dev libyaml-cpp-dev libboost-all-dev libtbb-dev \
    gazebo \
    ros-humble-gtsam \
    ros-humble-ament-cmake-clang-format \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-pcl-conversions ros-humble-tf2-eigen

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
if [[ "${DO_APT}" -eq 1 ]] && ! command -v rosdep >/dev/null 2>&1; then
  echo "ERROR: rosdep is missing. Install python3-rosdep or rerun without --no-apt." >&2
  exit 5
fi
if ! command -v colcon >/dev/null 2>&1; then
  echo "ERROR: colcon is missing. Install python3-colcon-common-extensions." >&2
  exit 5
fi

REPOS_FILE="${REPO_ROOT}/dependencies/agt_navigation.repos"
echo "==> Validating source dependency manifest structure"
# vcstool 0.3.0 has a validate-only bug for exact SHA versions. Validate the
# .repos schema ourselves, then let `vcs import` perform the real checkout.
python3 - "${REPOS_FILE}" <<'PY'
import pathlib
import sys
import yaml

path = pathlib.Path(sys.argv[1])
data = yaml.safe_load(path.read_text(encoding='utf-8'))
repos = data.get('repositories') if isinstance(data, dict) else None
if not isinstance(repos, dict) or not repos:
    raise SystemExit(f'ERROR: invalid repositories mapping in {path}')
for relpath, spec in repos.items():
    if not isinstance(relpath, str) or not relpath or relpath.startswith('/') or '..' in pathlib.PurePosixPath(relpath).parts:
        raise SystemExit(f'ERROR: unsafe repository path: {relpath!r}')
    if not isinstance(spec, dict):
        raise SystemExit(f'ERROR: repository spec for {relpath} is not a mapping')
    if spec.get('type') != 'git':
        raise SystemExit(f'ERROR: unsupported repository type for {relpath}: {spec.get("type")!r}')
    for key in ('url', 'version'):
        if not isinstance(spec.get(key), str) or not spec[key].strip():
            raise SystemExit(f'ERROR: missing {key} for {relpath}')
print(f'REPOS MANIFEST PASS: {len(repos)} repositories')
PY

echo "==> Importing missing field-demo repositories into ${SRC_DIR}"
# Repeatable and non-destructive: existing repositories and local changes are not overwritten.
vcs import --skip-existing "${SRC_DIR}" < "${REPOS_FILE}"

echo "==> Verifying exact dependency revisions"
python3 - "${SRC_DIR}" "${REPOS_FILE}" <<'PY'
import pathlib
import subprocess
import sys
import yaml

src = pathlib.Path(sys.argv[1])
manifest = pathlib.Path(sys.argv[2])
repos = yaml.safe_load(manifest.read_text(encoding='utf-8'))['repositories']
errors = []
for relpath, spec in repos.items():
    expected = str(spec['version']).strip()
    # Tags/branches remain legal in manifests, but exact 40-char revisions are
    # the reproducibility contract checked here.
    if len(expected) != 40 or any(c not in '0123456789abcdefABCDEF' for c in expected):
        continue
    checkout = src / relpath
    if not (checkout / '.git').exists():
        errors.append(f'{relpath}: checkout missing after vcs import')
        continue
    actual = subprocess.check_output(
        ['git', '-C', str(checkout), 'rev-parse', 'HEAD'], text=True
    ).strip()
    if actual.lower() != expected.lower():
        errors.append(f'{relpath}: expected {expected}, found {actual}')
if errors:
    raise SystemExit('ERROR: dependency revision mismatch:\n  ' + '\n  '.join(errors))
print('DEPENDENCY REVISION PASS')
PY

prepare_livox_ros2_source() {
  local driver="${SRC_DIR}/external/livox_ros_driver2"
  if [[ ! -d "${driver}" && -d "${SRC_DIR}/livox_ros_driver2" ]]; then
    driver="${SRC_DIR}/livox_ros_driver2"
  fi
  if [[ ! -f "${driver}/package_ROS2.xml" ]]; then
    echo "ERROR: Livox ROS Driver 2 source is incomplete: ${driver}" >&2
    exit 7
  fi
  # Official build.sh also deletes workspace-level build/install. Only create
  # the ROS2 source links here and keep normal colcon ownership of the workspace.
  if [[ ! -e "${driver}/package.xml" ]]; then ln -s package_ROS2.xml "${driver}/package.xml"; fi
  if [[ ! -e "${driver}/launch" ]]; then ln -s launch_ROS2 "${driver}/launch"; fi
}
prepare_livox_ros2_source

if [[ "${DO_APT}" -eq 1 ]]; then
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
  --rosdistro humble \
  --skip-keys GTSAM --skip-keys ament_python --skip-keys ament_pytest --skip-keys libeigen3-dev
else
  echo "==> Skipping rosdep update/install (--no-apt; system dependencies must already be present)"
fi

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
  cmake -S "${source_dir}" -B "${source_dir}/build-agt" -DCMAKE_INSTALL_PREFIX="${NATIVE_PREFIX}" "$@"
  cmake --build "${source_dir}/build-agt" --parallel "$(nproc)"
  cmake --install "${source_dir}/build-agt"
}

install_sophus() {
  local source_dir="${SRC_DIR}/external/Sophus"
  if [[ ! -d "${source_dir}" ]]; then echo "ERROR: Sophus source missing: ${source_dir}" >&2; exit 6; fi
  if [[ ! -f "${NATIVE_PREFIX}/lib/cmake/Sophus/SophusConfig.cmake" ]]; then
    echo "==> Installing Sophus into ${NATIVE_PREFIX}"
    cmake -S "${source_dir}" -B "${source_dir}/build-agt" \
      -DCMAKE_INSTALL_PREFIX="${NATIVE_PREFIX}" -DBUILD_SOPHUS_TESTS=OFF
    cmake --build "${source_dir}/build-agt" --parallel "$(nproc)"
    cmake --install "${source_dir}/build-agt"
  else
    echo "PASS native dependency already installed: Sophus"
  fi
}

if [[ "${DO_NATIVE}" -eq 1 ]]; then
  install_sophus
  install_native \
    "Livox-SDK2" \
    "test -f '${NATIVE_PREFIX}/include/livox_lidar_api.h' && (test -f '${NATIVE_PREFIX}/lib/liblivox_lidar_sdk_shared.so' || test -f '${NATIVE_PREFIX}/lib/liblivox_lidar_sdk.so')" \
    "${SRC_DIR}/external/Livox-SDK2" \
    -DCMAKE_BUILD_TYPE=Release

  install_native \
    "3D-BBS CPU" \
    "test -f '${NATIVE_PREFIX}/include/cpu_bbs3d/bbs3d.hpp' && test -f '${NATIVE_PREFIX}/lib/libcpu_bbs3d.so'" \
    "${SRC_DIR}/external/3d_bbs" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_CUDA=OFF

  install_native \
    "small_gicp" \
    "test -f '${NATIVE_PREFIX}/lib/cmake/small_gicp/small_gicp-config.cmake' || test -f '${NATIVE_PREFIX}/lib64/cmake/small_gicp/small_gicp-config.cmake'" \
    "${SRC_DIR}/external/small_gicp" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_HELPER=ON \
    -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF

  export CMAKE_PREFIX_PATH="${NATIVE_PREFIX}:${CMAKE_PREFIX_PATH:-}"
fi

if [[ "${DO_BUILD}" -eq 1 ]]; then
  echo "==> Building current field-demo software chain"
  cd "${WS_ROOT}"
  # Several upstream CMake projects hard-code -j8. Keep the workspace build
  # deterministic on field laptops and avoid memory-pressure aborts.
  export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-1}"
  export CMAKE_PREFIX_PATH="${NATIVE_PREFIX}:${CMAKE_PREFIX_PATH:-}"
  colcon build --executor sequential --parallel-workers 1 --symlink-install --event-handlers console_direct+ \
    --packages-skip bbs3d livox_sdk2 \
    --packages-up-to agt_system_bringup agt_gazebo_sim \
    --cmake-args -DCMAKE_PREFIX_PATH="${NATIVE_PREFIX}:${CMAKE_PREFIX_PATH}" \
      -DCMAKE_CXX_FLAGS="-I${NATIVE_PREFIX}/include -L${NATIVE_PREFIX}/lib" \
      -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
  set +u
  source "${WS_ROOT}/install/setup.bash"
  set -u
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
