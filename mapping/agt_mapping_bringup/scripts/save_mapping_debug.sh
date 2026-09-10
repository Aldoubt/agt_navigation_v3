#!/usr/bin/env bash
# Replace the fixed debug map artifacts without creating a Map Package.
set -euo pipefail

WORKSPACE=${AGT_WORKSPACE:-/home/yangxuan/ros2_ws}
DEBUG_ROOT="$WORKSPACE/agt_data/debug_mapping/current"
PGO_DIR="$DEBUG_ROOT/pgo"
LOCALIZATION_DIR="$DEBUG_ROOT/localization"
NAVIGATION_DIR="$DEBUG_ROOT/navigation"
RELOCALIZATION_DIR="$DEBUG_ROOT/relocalization"
OCTOMAP_DIR="$DEBUG_ROOT/octomap"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

echo "[debug-mapping] replacing only $DEBUG_ROOT"
mkdir -p "$DEBUG_ROOT"
# These paths are constructed above from the fixed debug root. Never accept a
# caller-supplied deletion target in this overwrite-only helper.
rm -rf -- "$PGO_DIR" "$LOCALIZATION_DIR" "$NAVIGATION_DIR" "$RELOCALIZATION_DIR" "$OCTOMAP_DIR"
mkdir -p "$PGO_DIR" "$LOCALIZATION_DIR" "$NAVIGATION_DIR" "$RELOCALIZATION_DIR" "$OCTOMAP_DIR"

echo '[debug-mapping] requesting final PGO map and keyframe patches'
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$PGO_DIR', save_patches: true}"

map_pcd="$PGO_DIR/map.pcd"
poses_txt="$PGO_DIR/poses.txt"
deadline=$((SECONDS + 45))
previous=''
while (( SECONDS < deadline )); do
  if [[ -s "$map_pcd" && -s "$poses_txt" ]] && compgen -G "$PGO_DIR/patches/*.pcd" >/dev/null; then
    snapshot="$(stat -c '%s:%Y' "$map_pcd" "$poses_txt" | tr '\n' ' ')"
    if [[ "$snapshot" == "$previous" ]]; then
      break
    fi
    previous="$snapshot"
  fi
  sleep 1
done
if [[ ! -s "$map_pcd" || ! -s "$poses_txt" ]] || ! compgen -G "$PGO_DIR/patches/*.pcd" >/dev/null; then
  echo '[debug-mapping] PGO output did not stabilize: expected map.pcd, poses.txt, and patches/*.pcd' >&2
  exit 1
fi

cp "$map_pcd" "$LOCALIZATION_DIR/global_map.pcd"

echo '[debug-mapping] saving live OctoMap /projected_map as Nav2 PGM/YAML'
ros2 run nav2_map_server map_saver_cli \
  -t /projected_map -f "$NAVIGATION_DIR/map" --fmt pgm
if [[ ! -s "$NAVIGATION_DIR/map.pgm" || ! -s "$NAVIGATION_DIR/map.yaml" ]]; then
  echo '[debug-mapping] OctoMap projection did not produce navigation/map.pgm and map.yaml' >&2
  exit 1
fi
MAPPING_SHARE="$(ros2 pkg prefix agt_mapping_bringup)/share/agt_mapping_bringup"
cp "$MAPPING_SHARE/config/octomap_navigation_baseline.yaml" \
  "$OCTOMAP_DIR/octomap_navigation_baseline.yaml"

echo '[debug-mapping] building 3D-BBS assets from the optimized PGO map'
ros2 run agt_global_relocalization_native build_relocalization_assets \
  --map "$LOCALIZATION_DIR/global_map.pcd" \
  --output "$RELOCALIZATION_DIR"

echo '[debug-mapping] building Polar Context coarse-search database from PGO keyframes'
ros2 run agt_global_relocalization_native build_relocalization_candidates \
  --map-dir "$PGO_DIR" \
  --output "$RELOCALIZATION_DIR"
if [[ ! -s "$RELOCALIZATION_DIR/polar_context.db" ]] \
  || [[ ! -d "$RELOCALIZATION_DIR/voxelmaps_coords" ]]; then
  echo '[debug-mapping] relocalization output is incomplete' >&2
  exit 1
fi

stamp="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
cat > "$DEBUG_ROOT/debug_map.yaml" <<EOF
schema_version: 1
mode: overwrite_debug
generated_at: "$stamp"
map_manager: false
pgo_map: "$map_pcd"
localization_map: "$LOCALIZATION_DIR/global_map.pcd"
navigation_map: "$NAVIGATION_DIR/map.yaml"
relocalization_assets: "$RELOCALIZATION_DIR"
polar_context_database: "$RELOCALIZATION_DIR/polar_context.db"
bbs_voxelmaps: "$RELOCALIZATION_DIR/voxelmaps_coords"
octomap_parameters: "$OCTOMAP_DIR/octomap_navigation_baseline.yaml"
EOF

echo '[debug-mapping] READY'
printf '%s\n' "  PGO map:          $map_pcd" \
  "  Nav2 map:         $NAVIGATION_DIR/map.yaml" \
  "  Localization PCD: $LOCALIZATION_DIR/global_map.pcd" \
  "  Relocalization:   $RELOCALIZATION_DIR" \
  "  Manifest:         $DEBUG_ROOT/debug_map.yaml"
