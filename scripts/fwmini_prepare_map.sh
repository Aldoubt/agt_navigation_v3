#!/usr/bin/env bash
set -eo pipefail

ROS_WS="$HOME/ros2_ws"
MAPPING_OVERLAY="$ROS_WS/install_mapping_framework"
GRID_CONFIG="$ROS_WS/src/agt_mapping_framework/exporters/agt_pcd2grid_exporter/config/projection.yaml"
MAP_ID="fwmini_mid360"

usage() {
  echo "Usage: $0 LIVE_RUN_DIR [CLEANED_GLOBAL_MAP.pcd] [MAP_VERSION]"
  echo "Without CLEANED_GLOBAL_MAP.pcd, the grid exporter uses package keyframes for temporal filtering."
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then usage; exit 0; fi
if (( $# < 1 || $# > 3 )); then usage >&2; exit 2; fi

RUN="$(realpath "$1")"
SOURCE="$RUN/map_package"
PCD="${2:-$SOURCE/map.pcd}"
PCD="$(realpath "$PCD")"
USE_PACKAGE_GRID=0
if (( $# == 1 )); then USE_PACKAGE_GRID=1; fi

DEFAULT_VERSION="$(basename "$RUN")"
DEFAULT_VERSION="${DEFAULT_VERSION#fwmini_live_}"
MAP_VERSION="${3:-$DEFAULT_VERSION}"
MAP_ROOT="$ROS_WS/maps/$MAP_ID/$MAP_VERSION"
PREP="$RUN/prepared/$MAP_VERSION"
NAV="$PREP/navigation"
RELOC="$PREP/relocalization"

[[ -f "$ROS_WS/install/setup.bash" ]] || { echo "Missing ROS workspace install overlay" >&2; exit 2; }
[[ -f "$MAPPING_OVERLAY/setup.bash" ]] || { echo "Missing mapping overlay" >&2; exit 2; }
[[ -s "$SOURCE/map.pcd" && -s "$SOURCE/poses.txt" && -s "$SOURCE/poses_timed.txt" && -d "$SOURCE/patches" ]] || {
  echo "Live map package is incomplete: $SOURCE" >&2
  echo "Wait for the mapping launch to finish verified export before preparing it." >&2
  exit 2
}
[[ -s "$PCD" ]] || { echo "PCD is missing or empty: $PCD" >&2; exit 2; }
[[ -f "$GRID_CONFIG" ]] || { echo "Missing projection config: $GRID_CONFIG" >&2; exit 2; }
[[ "$MAP_VERSION" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "MAP_VERSION may contain only letters, digits, dot, underscore, hyphen." >&2; exit 2; }
[[ ! -e "$PREP" ]] || { echo "Refusing to reuse preparation directory: $PREP" >&2; exit 2; }
[[ ! -e "$MAP_ROOT" ]] || { echo "Refusing to overwrite existing map version: $MAP_ROOT" >&2; exit 2; }

source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
source "$MAPPING_OVERLAY/setup.bash"
mkdir -p "$NAV" "$RELOC"

echo "Using localization PCD: $PCD"
echo "Using keyframes/poses from: $SOURCE"
echo "Building 2D occupancy map..."
if (( USE_PACKAGE_GRID )); then
  ros2 run agt_pcd2grid_exporter pcd2grid_exporter \
    --package "$SOURCE" \
    --output "$NAV" \
    --config "$GRID_CONFIG"
else
  ros2 run agt_pcd2grid_exporter pcd2grid_exporter \
    --pcd "$PCD" \
    --output "$NAV" \
    --config "$GRID_CONFIG"
fi

echo "Building 3D-BBS assets from the same PCD..."
ros2 run agt_global_relocalization_native build_relocalization_assets \
  --map "$PCD" \
  --output "$RELOC" \
  --map-leaf 0.5 \
  --bbs-min-level-res 0.5 \
  --bbs-max-level 5

echo "Building Polar Context candidate database from the same live mapping run..."
ros2 run agt_global_relocalization_native build_relocalization_candidates \
  --map-dir "$SOURCE" \
  --output "$RELOC"

echo "Creating a new immutable experimental map package..."
ros2 run agt_map_manager create_map_package \
  --map-root "$ROS_WS/maps" \
  --map-id "$MAP_ID" \
  --map-version "$MAP_VERSION" \
  --source-pcd "$PCD" \
  --navigation-dir "$NAV" \
  --relocalization-assets-dir "$RELOC"

[[ -s "$MAP_ROOT/metadata.yaml" && -s "$MAP_ROOT/localization/global_map.pcd" && -s "$MAP_ROOT/navigation/map.yaml" && -s "$MAP_ROOT/localization/relocalization/polar_context.db" ]] || {
  echo "Map package post-check failed: $MAP_ROOT" >&2
  exit 1
}
echo
echo "MAP_PACKAGE_READY=$MAP_ROOT"
echo "MAP_ID=$MAP_ID"
echo "MAP_VERSION=$MAP_VERSION"
echo "Next command: bash $ROS_WS/scripts/fwmini_start_nav.sh $MAP_VERSION"
echo "This is an experimental LIO-only map (no PGO/global ground truth); inspect it in RViz before any motion."
