#!/usr/bin/env bash
# Save the live OctoMap 2D projection and publish one immutable AGT Map Package.
set -euo pipefail

DEFAULT_MAP_ROOT=/home/yangxuan/ros2_ws/agt_data/maps

if [[ $# -lt 4 || $# -gt 7 ]]; then
  echo "usage: $0 <navigation-output-dir> <global-map.pcd> <map-id> <map-version> [rear-filter-statistics.yaml] [relocalization-assets-dir]" >&2
  echo "       legacy: $0 <navigation-output-dir> <global-map.pcd> <map-root> <map-id> <map-version> [rear-filter-statistics.yaml] [relocalization-assets-dir]" >&2
  exit 2
fi

NAVIGATION_DIR=$1
SOURCE_PCD=$2
# New calls use the workspace map root by default. Keep the original positional
# form for existing field scripts while all new documentation uses this root.
if [[ $# -ge 5 && "$3" == /* ]]; then
  MAP_ROOT=$3
  MAP_ID=$4
  MAP_VERSION=$5
  FILTER_STATISTICS=${6:-}
  RELOCALIZATION_ASSETS=${7:-}
else
  MAP_ROOT=${AGT_MAP_ROOT:-$DEFAULT_MAP_ROOT}
  MAP_ID=$3
  MAP_VERSION=$4
  FILTER_STATISTICS=${5:-}
  RELOCALIZATION_ASSETS=${6:-}
fi

if [[ ! -f "$SOURCE_PCD" ]]; then
  echo "global localization PCD does not exist: $SOURCE_PCD" >&2
  exit 2
fi
if [[ -n "$FILTER_STATISTICS" && ! -f "$FILTER_STATISTICS" ]]; then
  echo "rear filter statistics do not exist: $FILTER_STATISTICS" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash

SHARE_DIR="$(ros2 pkg prefix agt_mapping_bringup)/share/agt_mapping_bringup"
mkdir -p "$NAVIGATION_DIR"
ros2 run nav2_map_server map_saver_cli \
  -t /projected_map -f "$NAVIGATION_DIR/map" --fmt pgm

cp "$SHARE_DIR/config/octomap_navigation_baseline.yaml" \
  "$NAVIGATION_DIR/octomap_baseline_parameters.yaml"
if [[ -n "$FILTER_STATISTICS" ]]; then
  cp "$FILTER_STATISTICS" "$NAVIGATION_DIR/rear_filter_statistics.yaml"
fi

PACKAGE_ARGS=(
  --map-root "$MAP_ROOT"
  --map-id "$MAP_ID"
  --map-version "$MAP_VERSION"
  --source-pcd "$SOURCE_PCD"
  --navigation-dir "$NAVIGATION_DIR"
)
if [[ -n "$RELOCALIZATION_ASSETS" ]]; then
  PACKAGE_ARGS+=(--relocalization-assets-dir "$RELOCALIZATION_ASSETS")
fi
ros2 run agt_map_manager create_map_package "${PACKAGE_ARGS[@]}"
