# Field map finalization and HMI workflow

## Immutable map identity

Use one site/run identifier and an explicit version:

```text
<map_id>/<map_version>
example: bunker_mid360_mapping_20260901_205036/formal_pgo_v1
```

All published Map Packages are under
`/home/yangxuan/ros2_ws/agt_data/maps`. Do not overwrite an existing version.
A new mapping run, terrain/PGM rebuild, or HMI edit always creates a new
`map_version`.

## Finalize a mapping run

The live FAST-LIO2/OctoMap projection is useful for mapping-time preview. The
field navigation package is finalized only after PGO produces the final
`global_map.pcd` and the navigation PGM is generated in that same `map` frame.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Produce relocalization assets from the exact localization PCD.
ros2 run agt_global_relocalization_native build_relocalization_assets \
  --map <final-global_map.pcd> \
  --output <staging-relocalization-dir> \
  --map-leaf 0.5 --bbs-min-level-res 0.5 --bbs-max-level 5

# Publish an immutable, hash-validated package.
ros2 run agt_map_manager create_map_package \
  --map-id <site_or_run_id> --map-version <version> \
  --source-pcd <final-global_map.pcd> \
  --navigation-dir <pgo-consistent-navigation-dir> \
  --relocalization-assets-dir <staging-relocalization-dir>
```

For a PGO-backed OctoMap finalizer, replay/reconstruct the occupancy evidence
using final `patches/*.pcd + poses.txt`; do not package an uncorrected live
OctoMap PGM with a PGO-corrected localization PCD.

## Acceptance and activation

1. Validate that PCD XY bounds fit within `map.yaml` extent.
2. In RViz, overlay static scene features in the PCD and PGM.
3. Perform a stationary real-vehicle relocalization check before enabling
   autonomous motion.
4. Select the accepted version. Selection writes a pointer only; it does not
   hot-switch a running navigation stack.

```bash
ros2 run agt_map_manager list_map_packages
ros2 run agt_map_manager select_map_package \
  --map-id <map_id> --map-version <map_version>
```

## HMI map editing and inspection missions

```bash
ros2 launch agt_system_bringup hmi_field_demo.launch.py
```

HMI receives Nav2 `/map`; its target poses publish on `/goal_pose` and enter
the existing inspection queue. Save HMI map/topology edits first to
`/home/yangxuan/ros2_ws/agt_data/maps/hmi_staging/`, then promote them:

```bash
ros2 run agt_map_manager promote_hmi_navigation_edit \
  --base-metadata <active-package>/metadata.yaml \
  --edited-map-yaml <staging>/map.yaml \
  --map-id <map_id> --map-version <new_hmi_edit_version> --activate
```

HMI edit changes only the navigation PGM and optional `map.topology`.
Localization PCD and relocalization assets are copied unchanged into the new
package version. Once the queued targets are explicitly started through the
normal inspection service, mission YAML is recorded in:

```text
/home/yangxuan/ros2_ws/agt_data/missions/<map_id>/<map_version>/
```

This separates immutable map geometry, editable navigation/topology data, and
operational mission records.
