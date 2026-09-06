#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="${AGT_WS:-}"
FORCE=0

usage() {
  cat <<'EOF'
Usage: bootstrap_terrain_mapping.sh [--workspace PATH] [--force]

Fetch and install terrain-map native dependencies without adding third-party
ROS packages to the workspace src tree.

The pinned source manifest is dependencies/terrain_native.repos. Sources are
stored under <workspace>/.agt_sources/terrain and native artifacts under
<workspace>/.agt_native, which is the same prefix used by bootstrap_humble.sh.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      WS_ROOT="${2:?--workspace requires a path}"
      shift 2
      ;;
    --force)
      FORCE=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${WS_ROOT}" ]]; then
  repo_parent="$(dirname "${REPO_ROOT}")"
  if [[ "$(basename "${repo_parent}")" == "src" ]]; then
    WS_ROOT="$(dirname "${repo_parent}")"
  else
    WS_ROOT="${HOME}/agt_ws"
  fi
fi

WS_ROOT="$(realpath -m "${WS_ROOT}")"
SOURCE_ROOT="${WS_ROOT}/.agt_sources/terrain"
NATIVE_PREFIX="${WS_ROOT}/.agt_native"
MANIFEST="${REPO_ROOT}/dependencies/terrain_native.repos"
mkdir -p "${SOURCE_ROOT}" "${NATIVE_PREFIX}"

for cmd in git cmake; do
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "ERROR: required command not found: ${cmd}" >&2
    exit 3
  fi
done
if ! command -v vcs >/dev/null 2>&1; then
  echo "ERROR: vcs is missing. Install python3-vcstool (or pip install vcstool)." >&2
  exit 3
fi
if [[ ! -f /usr/include/eigen3/Eigen/Core ]]; then
  echo "ERROR: system Eigen3 headers not found. Install libeigen3-dev." >&2
  exit 3
fi

python3 - "${MANIFEST}" <<'PY'
import pathlib
import sys
import yaml
path = pathlib.Path(sys.argv[1])
data = yaml.safe_load(path.read_text(encoding='utf-8'))
repos = data.get('repositories') if isinstance(data, dict) else None
if not isinstance(repos, dict) or not repos:
    raise SystemExit(f'ERROR: invalid terrain dependency manifest: {path}')
for relpath, spec in repos.items():
    if not isinstance(spec, dict) or spec.get('type') != 'git':
        raise SystemExit(f'ERROR: invalid repository spec: {relpath}')
    for key in ('url', 'version'):
        if not isinstance(spec.get(key), str) or not spec[key].strip():
            raise SystemExit(f'ERROR: missing {key} for {relpath}')
print(f'TERRAIN REPOS MANIFEST PASS: {len(repos)} repositories')
PY

if [[ "${FORCE}" -eq 1 ]]; then
  rm -rf "${SOURCE_ROOT}/patchwork-plusplus/build-agt"
fi

vcs import --skip-existing "${SOURCE_ROOT}" < "${MANIFEST}"

python3 - "${SOURCE_ROOT}" "${MANIFEST}" <<'PY'
import pathlib
import subprocess
import sys
import yaml
root = pathlib.Path(sys.argv[1])
manifest = pathlib.Path(sys.argv[2])
repos = yaml.safe_load(manifest.read_text(encoding='utf-8'))['repositories']
errors = []
for relpath, spec in repos.items():
    checkout = root / relpath
    expected = spec['version']
    if not (checkout / '.git').exists():
        errors.append(f'{relpath}: checkout missing')
        continue
    actual = subprocess.check_output(['git', '-C', str(checkout), 'rev-parse', 'HEAD'], text=True).strip()
    if len(expected) == 40 and actual.lower() != expected.lower():
        errors.append(f'{relpath}: expected {expected}, found {actual}')
if errors:
    raise SystemExit('ERROR: terrain dependency revision mismatch:\n  ' + '\n  '.join(errors))
print('TERRAIN DEPENDENCY REVISION PASS')
PY

PW_SRC="${SOURCE_ROOT}/patchwork-plusplus/cpp"
PW_BUILD="${SOURCE_ROOT}/patchwork-plusplus/build-agt"
PW_CONFIG="$(find "${NATIVE_PREFIX}" -type f \( -name 'patchworkppConfig.cmake' -o -name 'patchworkpp-config.cmake' \) -print -quit 2>/dev/null || true)"

if [[ -n "${PW_CONFIG}" && "${FORCE}" -eq 0 ]]; then
  echo "PASS Patchwork++ already installed: ${PW_CONFIG}"
else
  echo "==> Building Patchwork++ into ${NATIVE_PREFIX}"
  cmake -S "${PW_SRC}" -B "${PW_BUILD}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${NATIVE_PREFIX}" \
    -DUSE_SYSTEM_EIGEN3=ON \
    -DINCLUDE_CPP_EXAMPLES=OFF
  cmake --build "${PW_BUILD}" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
  cmake --install "${PW_BUILD}"
fi

PW_CONFIG="$(find "${NATIVE_PREFIX}" -type f \( -name 'patchworkppConfig.cmake' -o -name 'patchworkpp-config.cmake' \) -print -quit 2>/dev/null || true)"
if [[ -z "${PW_CONFIG}" ]]; then
  echo "ERROR: Patchwork++ install completed but no CMake package config was found under ${NATIVE_PREFIX}." >&2
  exit 4
fi

cat <<EOF

Terrain native bootstrap complete.
  workspace:     ${WS_ROOT}
  source root:   ${SOURCE_ROOT}
  native prefix: ${NATIVE_PREFIX}
  patchworkpp:   ${PW_CONFIG}

Before building AGT terrain adapters:
  export CMAKE_PREFIX_PATH="${NATIVE_PREFIX}:\${CMAKE_PREFIX_PATH:-}"
EOF
