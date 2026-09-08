# HMI map editing and navigation targets

## Ownership boundary

`agt_navigation_v3` owns the authoritative map lifecycle. `agt_robot_hmi` is a
replaceable presentation and 2D navigation-map editing client.

The contract is intentionally strict:

1. Released Map Packages are immutable.
2. HMI may edit occupancy cells and topology only.
3. HMI must not change grid width/height, resolution, origin, origin yaw, Nav2
   mode/negate/threshold interpretation, localization PCD, relocalization assets
   or RTK origin.
4. HMI never chooses an arbitrary released file as the system truth. Navigation
   mode follows `active_map.yaml`; edit mode follows a V3-created EditSession.
5. V1 edit sessions support only axis-aligned Nav2 maps (`origin[2] == 0`).

## Storage layout

```text
/home/yangxuan/ros2_ws/agt_data/
├── maps/
│   ├── <map_id>/<map_version>/       # immutable released packages
│   └── active_map.yaml               # selected package pointer
├── map_edits/
│   └── <session_id>/
│       ├── session.yaml              # immutable baseline contract + state
│       ├── map.yaml                  # HMI staging map
│       ├── map.pgm                   # HMI may change occupancy pixels
│       └── map.topology              # optional HMI topology
├── hmi_runtime/
└── missions/
```

The HMI must never write into an existing `maps/<map_id>/<map_version>` package.

## Preferred EditSession workflow

### 1. Start an edit session

The base package is fully revalidated before staging. Exact map ID and version
are mandatory; there is no implicit `latest` selection.

```bash
ros2 service call /agt/map/edit/start \
  agt_robot_interfaces/srv/StartMapEdit \
  "{map_id: site_a, map_version: v1}"
```

The response contains a `MapEditSession` with `session_id`, staging path,
`navigation_map_yaml`, and both geometry/contract fingerprints. V3 copies the
base navigation YAML/PGM and optional topology into the session. The released
package remains untouched.

The HMI should open the returned `navigation_map_yaml` (normally
`.../map_edits/<session_id>/map.yaml`) and save edits back into that session.

### 2. Publish a new immutable version

```bash
ros2 service call /agt/map/edit/publish \
  agt_robot_interfaces/srv/PublishMapEdit \
  "{session_id: edit_..., target_map_id: site_a, target_map_version: v2, activate: true}"
```

Before package creation V3 re-reads the edited PGM/YAML and rejects any change
to the immutable navigation contract:

```text
frame_id = map
PGM width
PGM height
resolution
origin x
origin y
origin yaw = 0
mode
negate
occupied_thresh
free_thresh
```

The contract is checked against the snapshot recorded when the session was
created. Publication then calls the same compatibility gate again against the
fully revalidated base package, so the rule cannot be bypassed by a stale or
modified session metadata file.

A successful publish creates a new package and copies the base localization
PCD, 3D-BBS/relocalization assets and RTK origin unchanged. If `activate: true`,
Map Manager then selects the new package and publishes a new `/agt/map/status`
generation.

If package publication succeeds but activation persistence fails, the service
returns that explicit partial result. The package remains immutable and the
operator can retry activation through `/agt/map/load`; the edit is never
silently recreated or overwritten.

### 3. Cancel an edit session

```bash
ros2 service call /agt/map/edit/cancel \
  agt_robot_interfaces/srv/CancelMapEdit \
  "{session_id: edit_...}"
```

Cancellation changes the session state to `cancelled` but deliberately keeps the
staging directory for audit/debugging.

## Compatibility CLI

`promote_hmi_navigation_edit` remains available for scripts and recovery work,
but it now uses the same non-bypassable navigation contract gate. It can no
longer publish an edited YAML/PGM with a changed origin, resolution, dimensions
or Nav2 interpretation.

```bash
ros2 run agt_map_manager promote_hmi_navigation_edit \
  --base-metadata /home/yangxuan/ros2_ws/agt_data/maps/site_a/v1/metadata.yaml \
  --edited-map-yaml /path/to/staging/map.yaml \
  --map-id site_a --map-version v2 --activate
```

The service workflow is preferred because Map Manager remains the single owner
of active-map state inside the running process.

## Navigation mode

```bash
ros2 launch agt_system_bringup hmi_field_demo.launch.py
```

The launch resolves the active Map Package, starts Nav2 and global
relocalization using paired assets, and writes a version-scoped HMI runtime
configuration under `/home/yangxuan/ros2_ws/agt_data/hmi_runtime/`. Therefore
navigation mode always opens the selected package instead of an HMI-local
"last opened file".

The HMI subscribes to `/map` and emits `geometry_msgs/PoseStamped` on
`/goal_pose`. `agt_rviz_patrol` can record that goal in the inspection queue.
Autonomous motion still depends on the normal localization, measured-stop,
Bunker command, and mission-runtime gates.

Mission files remain outside immutable Map Packages:

```text
/home/yangxuan/ros2_ws/agt_data/missions/<map_id>/<map_version>/
```

## Coordinate policy

The current HMI display uses continuous image/scene coordinates while Nav2 uses
world/grid coordinates. Do not introduce per-widget ad-hoc conversions.
`agt_robot_hmi` provides a canonical axis-aligned `MapCoordinateTransform` for
new V3 integration code, while existing continuous scene transforms are migrated
incrementally to avoid shifting current topology and robot overlays by half a
cell.

For occupancy indexing, use the standard grid rule:

```text
column = floor((world_x - origin_x) / resolution)
row    = floor((world_y - origin_y) / resolution)
```

For an actual cell centre:

```text
world_x = origin_x + (column + 0.5) * resolution
world_y = origin_y + (row    + 0.5) * resolution
```

Do not mix cell-corner and cell-centre semantics in the same API.
