# HMI map editing and navigation targets

## Workspace map root

All versioned Map Packages and the selected-map pointer use:

```text
/home/yangxuan/ros2_ws/agt_data/maps/
├── <map_id>/<map_version>/
└── active_map.yaml
```

The HMI must never overwrite `navigation/map.pgm` inside an existing Map
Package: that would invalidate its hashes and leave its localization assets
without an auditable navigation-map version.

Before the first HMI session, select one **validated, coordinate-consistent**
package as active. This writes the workspace `active_map.yaml` pointer; it does
not hot-reload a running Nav2 instance.

```bash
ros2 run agt_map_manager list_map_packages
ros2 run agt_map_manager select_map_package \
  --map-id <map_id> --map-version <map_version>
```

`agt_map_manager` continues to expose `/agt/map/load` for an HMI/system-level
map picker, but the command above is the normal one-shot operator workflow.

## Editing workflow

1. Start `hmi_field_demo.launch.py`. The HMI receives the active Nav2 `/map`.
2. In HMI, use **另存为** into a staging directory, for example
   `/home/yangxuan/ros2_ws/agt_data/maps/hmi_staging/site_a_edit/map.yaml`.
   It writes `map.yaml`, its PGM image and optionally `map.topology`.
3. Promote that saved YAML as a new version. This copies the same verified
   localization PCD/relocalization assets and records HMI provenance.

```bash
ros2 run agt_map_manager promote_hmi_navigation_edit \
  --base-metadata /home/yangxuan/ros2_ws/agt_data/maps/site_a/v1/metadata.yaml \
  --edited-map-yaml /home/yangxuan/ros2_ws/agt_data/maps/hmi_staging/site_a_edit/map.yaml \
  --map-id site_a --map-version hmi_edit_v1 --activate
```

`--activate` atomically updates `active_map.yaml`. Start navigation again so
Nav2 loads the new PGM; online Nav2 map hot-reload is intentionally not used.

## Navigation and inspection points

```bash
ros2 launch agt_system_bringup hmi_field_demo.launch.py
```

The launch resolves the active Map Package automatically, starts Nav2 and
global relocalization using its paired assets, then starts `agt_robot_hmi`.
It also writes a version-scoped HMI runtime configuration under
`/home/yangxuan/ros2_ws/agt_data/hmi_runtime/`, so the HMI directly opens the
active `map.yaml` even before the live Nav2 `/map` update arrives.
The HMI subscribes to `/map` and emits `geometry_msgs/PoseStamped` on
`/goal_pose`. `agt_rviz_patrol` receives that goal and records it in the
existing inspection queue. A target is not autonomous motion until the normal
localization, measured-stop, Bunker command, and mission-runtime gates are all
healthy.

When the operator starts the queued inspection through `/agt/rviz_patrol/start`,
the generated mission YAML is stored outside the immutable Map Package at:

```text
/home/yangxuan/ros2_ws/agt_data/missions/<map_id>/<map_version>/
```

Each mission embeds its selected map identifier, so an edited map version and
the task points used with it remain traceable without modifying the localization
PCD or its package hashes.

## Native topology editor

The HMI's native route-point editor is the intended map-editing UI. Select
**编辑地图**, then use the vertical tools beside the map: **添加工位点** creates
named points and **连接工位点** links two points into a topology edge. The
**View → Task** dock can assemble an ordered task from those named points.

Use **另存为** after an edit and save into `hmi_staging`; do not use the disabled
**保存地图** control, because published Map Packages are immutable. The obsolete
`Inspection Task` plugin panel has been removed: it used a different YAML/task
protocol and was not connected to the AGT field mission runtime.
