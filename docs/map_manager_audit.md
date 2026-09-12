# AGT Navigation V3 Map Manager Audit

## Scope and snapshot

This is a read-only architecture audit of the local `agt_navigation_v3` tree.
No source, configuration, or existing document was changed. The inspected Git
baseline is `f5af4e6`; the working tree also contains uncommitted Map Package
pipeline, runtime-binding, and action-interface work. This report calls those
items **local candidate work**, not a released baseline.

The audit covers mapping, map conversion, Map Manager, relocalization, Nav2,
the system/HMI launch path, RViz patrol, and mission/camera execution. It does
not assess navigation algorithm quality or HMI source code outside this repo.

## Executive conclusion

The repository has the right architectural building blocks for immutable Map
Packages: exact-version selection, hash/path validation, `active_map.yaml`,
generation-bearing `MapStatus`, relocalization assets, HMI edit sessions, and
version-scoped mission storage. The HMI field demo can already launch Nav2 and
global relocalization from one verified active package.

The missing production boundary is the completion orchestrator. A completed PGO
run does not yet cause a deterministic sequence of PGO save, finalized asset
collection, navigation-map generation, relocalization-asset generation, quality
gate, immutable publication, and save-wizard handoff. The current OctoMap
baseline is a separately accumulated live projection; it must not be paired
with a post-PGO `global_map.pcd` unless their common `map` frame and source
revision are proven. Until that evidence exists, the package is structurally
valid but not necessarily spatially coherent.

The recommended first production path is a PGO-finalized PCD projection
pipeline with explicit source/pose provenance. OctoMap should be retained as a
reviewable baseline and optional comparison asset, rather than silently being
the authoritative PGM for a PGO-final package. `agt_map_converter` is a useful
V1 fallback; `agt_terrain_map_generator` is intentionally disabled and should
remain an offline candidate until its complete job and acceptance suite exist.

## A. Current package and launch topology

| Area | Package(s) | Current responsibility and important entry points |
| --- | --- | --- |
| Mapping front end and PGO | `agt_mapping_bringup` | `mapping_mode.launch.py` starts one `fastlio2/lio_node` and optionally one `pgo/pgo_node`; `mapping_mode.yaml` declares `/pgo/save_maps` and the PGO/HBA policy. |
| Mapping session API | `agt_mapping_session` | `mapping_session.py` exposes a small `Trigger` state machine. It is a stub: its start/stop/save callbacks do not call a mapping backend, PGO save, converter, or package publisher. |
| OctoMap baseline | `agt_mapping_bringup`, `agt_pointcloud_preprocessor`, `octomap_server` | `mapping_mode.launch.py` feeds post-LIO `/fastlio2/body_cloud` through the rear filter into `octomap_server`; `octomap_navigation_baseline.launch.py` provides a dedicated variant. |
| Point-cloud conversion | `agt_map_converter` | `pcd_to_nav_map.py` produces Nav2 PGM/YAML plus elevation, slope, obstacle images; `validate_nav_map.py` performs structural validation. |
| Map assets and lifecycle | `agt_map_manager` | Discovers/validates immutable packages, selects exact ID/version, persists active state, manages edit sessions. Local candidate files add PCD-to-package pipeline and runtime active-pointer validation. |
| Terrain path | `agt_terrain_map_generator` | Defines a terrain-aware offline architecture, but its configuration remains disabled and the README explicitly says the stages are not yet one production job. |
| Global localization | `agt_global_relocalization`, `agt_localization_manager` | Global relocalization can consume `/agt/map/status`; Localization Manager owns global `map -> odom` behavior from relocalization plus local odometry. |
| Navigation | `agt_nav2_bringup` | `navigation.launch.py` validates a passed YAML path and starts `nav2_map_server` with `yaml_filename`; it has no MapStatus subscription or hot-reload controller. |
| Field/HMI integration | `agt_system_bringup`, external `agt_robot_hmi` | `hmi_field_demo.launch.py` resolves `active_map.yaml`, validates the package, writes a generated HMI runtime JSON with the selected `map.yaml`, then includes the field demo and starts the external HMI. |
| HMI goals, tasks, camera | `agt_rviz_patrol`, `agt_navigation_runtime` | HMI/RViz goals reach `/goal_pose`; patrol queues inspection points; Mission Runtime invokes Nav2 `NavigateToPose` and `/camera_gimbal/acquire_view`. |

### Actual launch paths

```text
Mapping
  agt_mapping_bringup/mapping_mode.launch.py
    FAST-LIO2 -> /fastlio2/body_cloud -> rear filter -> OctoMap -> /projected_map
             \-> PGO -> final mapping files, saved via /pgo/save_maps

Navigation/HMI
  agt_system_bringup/hmi_field_demo.launch.py
    active_map.yaml -> hash-validated Map Package
       -> generated HMI config (selected navigation/map.yaml)
       -> rviz_field_demo.launch.py
          -> global_relocalization (explicit PCD/assets, Map Manager follow disabled)
          -> localization_manager
          -> agt_nav2_bringup/navigation.launch.py -> Nav2 map_server
          -> agt_navigation_runtime + agt_rviz_patrol -> Nav2/camera task flow
```

`system.launch.py` can also start `agt_map_manager`, but its Nav2 map remains a
manually supplied launch argument. It is not a complete active-map runtime
supervisor.

## B. Existing map flow and ownership boundaries

### Current map generation entry points

1. **PGO mapping result:** `mapping_mode.yaml` declares `/pgo/save_maps` as the
   save service. FAST-LIO2 plus PGO generate the final global mapping product,
   conventionally `global_map.pcd`, but no node in this repository observes
   completion or invokes that service as part of a Map Package workflow.
2. **OctoMap baseline export:** `scripts/export_octomap_navigation_map.sh`
   manually saves `/projected_map` with `nav2_map_server map_saver_cli`, copies
   OctoMap/filter evidence, and calls `create_map_package`. It requires an
   independently supplied `global_map.pcd`.
3. **PCD converter:** `ros2 run agt_map_converter pcd_to_nav_map` is an offline
   CLI conversion path. It supports ASCII and uncompressed binary PCD XYZ,
   occupancy from cell height span/slope, and optional `poses.txt` footprint
   carving.
4. **Local candidate pipeline:** uncommitted
   `agt_map_manager/map_pipeline.py` uses the converter, runs the structural
   validator, records source SHA-256/pipeline YAML/quality YAML, and publishes
   a package. The local candidate `GenerateMapPackage.action` exposes it at
   `/agt/map/generate`. It is not connected to Mapping Session or PGO output.
5. **Terrain generator:** this is an offline future route. Its README states
   `pipeline.enabled: false`; it must not be treated as an active generator.

### Current map loading entry points

1. **Map Manager:** `/agt/map/list` discovers packages and `/agt/map/load`
   accepts only an exact `map_id` plus `map_version`. Load revalidates the
   package and persists the selected state atomically.
2. **Compatibility CLI:** `select_map_package` can select a package outside a
   running manager process.
3. **HMI field demo:** `hmi_field_demo.launch.py` calls local-candidate
   `resolve_active_map(active_map.yaml)`, validates the pointed package and
   supplies its PGM/YAML and PCD to the field stack.
4. **Direct demo launch:** `rviz_field_demo.launch.py` and
   `agt_nav2_bringup/navigation.launch.py` still accept arbitrary absolute map
   paths. This is useful for bench/offline demos but bypasses Map Manager as an
   authority boundary.

### Active map mechanism

```text
/agt/map/load(map_id, map_version)
  -> discover and hash-validate one immutable package
  -> atomically write agt_data/maps/active_map.yaml
  -> increment generation
  -> publish transient-local /agt/map/status (MapStatus)
```

`MapStatus` carries package path, map ID/version, Nav2 YAML, localization PCD,
relocalization asset path, RTK origin, generation, and reason. Exact selection
is a strong property: there is no implicit "latest" version. Edit publication
can create and optionally activate a new version.

Global relocalization is already generation-aware when
`follow_map_manager=true`: it subscribes to `/agt/map/status` and re-arms when
the generation changes. In the field demo it is intentionally launched with
`follow_map_manager=false`; the selected PCD/assets are frozen into launch
arguments. Nav2 has the same static behavior because map_server receives a
single startup `yaml_filename`. Consequently an active-pointer update does not
mean that every runtime consumer has switched.

### HMI relationship to map files

The documented ownership rule is sound: HMI must not modify a released package
or choose an arbitrary file as system truth. Edit Sessions copy only the Nav2
map/topology into a staging directory and publication enforces width, height,
resolution, origin, yaw, mode, negate, and threshold fingerprints.

In implementation, the HMI executable is outside this repo. The V3 launch
creates a version-scoped JSON configuration containing
`map_config.path=<validated navigation/map.yaml>` plus ID/version/generation.
That stops a "last opened file" preference from deciding navigation, but it is
still a filesystem-path handoff, not a HMI client of Map Manager's ROS API.
There is no in-repo HMI adapter that lists packages, selects a package, starts
an edit session, publishes an edit, or consumes acknowledgement/state changes.

## ROS interfaces and configuration examined

| Type | Interfaces / behavior |
| --- | --- |
| Mapping topics/services | `/agt/mapping/status`, `/agt/mapping/result`, `/agt/mapping/start`, `/agt/mapping/stop`, `/agt/mapping/save`; `/pgo/save_maps`; `/fastlio2/body_cloud`; `/agt/octomap/rear_filtered_cloud`; `/projected_map`. Mapping Session payloads are currently plain `std_msgs/String` and `Trigger`. |
| Map Manager topic | `/agt/map/status` (`MapStatus`), reliable/transient-local. |
| Map Manager services | `/agt/map/list`, `/agt/map/load`, `/agt/map/edit/start`, `/agt/map/edit/publish`, `/agt/map/edit/cancel`. |
| Local candidate action | `/agt/map/generate` (`GenerateMapPackage`): map identity, PCD, pipeline config, relocalization assets, poses, RTK origin, preview; phase feedback. |
| Localization | `/agt/odometry/local`, `/agt/relocalization/pose`, `/agt/localization/status`, `/agt/relocalization/request`, `/agt/localization/relocalize`. |
| Navigation / HMI / mission | `/map`, `/goal_pose`, `/agt/task/request`, `/agt/task/status`, `/agt/task/start`, `/agt/task/pause`, `/agt/task/cancel`, `/agt/mission/execute`, `/agt/mission/status`, Nav2 `NavigateToPose`, camera `/camera_gimbal/acquire_view`. |
| Primary configuration | `mapping_mode.yaml`, `octomap_navigation_baseline.yaml`, FAST-LIO2/PGO YAML, `map_manager.yaml`, local `map_pipeline.example.yaml`, `global_relocalization.yaml`, `localization_manager.yaml`, and `nav2_params.yaml`. |

## Current architecture diagram

```text
             MID360 CustomMsg + IMU
                        |
                        v
               FAST-LIO2 front end
                  |              |
                  |              +----> body cloud -> rear filter -> OctoMap
                  |                                      |                |
                  |                                      +----> /projected_map
                  v
              PGO optimization
                  |
                  +----> /pgo/save_maps -> final global_map.pcd + poses/patches
                                                     |
              [NO completion/orchestration link]     |
                                                     v
   manual OctoMap script or PCD converter / local candidate map_pipeline
                    |                         |
                    +------> PGM/YAML -------+
                              |          relocal assets / RTK / provenance
                              v
                    immutable Map Package (metadata + hashes)
                              |
                  Map Manager: list/load/edit/active pointer/status
                      |                                       |
                      |                                       +--> Global relocalization
                      |                                             (supported, disabled in field demo)
                      v
      HMI field launch resolves package -> writes HMI map path -> Nav2 map_server
                                                         |
                                                   HMI / RViz /goal_pose
                                                         |
                                               Patrol -> mission -> Nav2 + camera
```

The brackets identify the primary missing control-plane connection. The two map
generation branches above should not be assumed equivalent merely because both
produce a `map`-frame-looking artifact.

## Implemented capabilities

### Released or present baseline capabilities

- FAST-LIO2 plus optional PGO mapping launch, with exactly one LIO front end.
- Global PCD / PGO save configuration and a clearly separate OctoMap O1-H2
  baseline branch with rear dynamic suppression.
- Immutable Map Package schema and package discovery; map identity, version,
  `frame_id=map`, package-contained paths, and hashes are validated.
- Required localization PCD and Nav2 map; optional relocalization, RTK,
  terrain layers, preview, generation, and quality assets.
- `active_map.yaml`, exact version selection, atomic persistence, and
  generation-bearing `MapStatus` publication.
- HMI edit sessions with immutable geometry/Nav2 interpretation contract and
  publish-as-new-version behavior.
- Global relocalization can bind its PCD/relocalization assets to MapStatus.
- HMI/RViz goal path, inspection mission execution through Nav2, and camera
  acquisition flow.
- OctoMap export helper and PCD-to-Nav2-map converter/structural validator.

### Local candidate capabilities that need commit, review, and integration

- `map_pipeline.py` turns a selected PCD into a package with pipeline and
  quality artifacts.
- `GenerateMapPackage.action` and its Map Manager action server expose that
  pipeline through ROS 2.
- `runtime_binding.py` resolves and revalidates an active pointer before the
  HMI field demo reads paths.
- Package validation additionally checks that a navigation YAML image remains
  inside the immutable package.

These candidate features close useful packaging gaps but do not yet close the
mapping-complete, coordinate-coherence, or runtime-application gaps below.

## Missing modules and architecture risks

| Priority | Missing module or risk | Evidence and required outcome |
| --- | --- | --- |
| P0 | Mapping completion orchestrator | Mapping Session explicitly says backend/save/converter connections are future work. Add one authoritative state machine that owns PGO save request/result, artifact stabilization, map generation, relocalization build, validation, publication, and wizard notification. |
| P0 | PGO-to-navigation coordinate contract | Live OctoMap accumulates from the FAST-LIO stream while PGO later optimizes map/patch poses. The export script accepts any PCD and current projected map. A final PGO PCD plus an earlier live OctoMap PGM can be misregistered. Require shared source-run ID, final pose revision/hash, `map` frame declaration, bounds/overlay check, and rejection on mismatch. |
| P0 | Activation transaction across consumers | Map Manager changes the pointer/status, but Nav2 map_server cannot hot reload and the field demo disables MapStatus-following relocalization. Define "selected", "prepared", "applied", and "active" acknowledgements; prohibit activation while mission/navigation is active; perform controlled restart for V1. |
| P0 | HMI Map Manager API adapter | Current HMI receives a generated file path. Provide a narrow HMI-facing client/bridge for package list/load/status and edit-session APIs. HMI must send IDs/session IDs, never package paths, PCD paths, or arbitrary map YAML paths. |
| P1 | PCD projection quality model | The converter uses vertical span and slope from raw cell extrema. It lacks completed-map ground segmentation, persistence/dynamic evidence, full robot geometry, frame/revision verification, and acceptance thresholds beyond nonempty pixels. Treat it as V1 fallback and record every parameter/provenance value. |
| P1 | OctoMap finalization policy | Decide whether OctoMap is an authoritative navigation source, a baseline comparison, or an optional package asset. If authoritative, rebuild/reproject after final PGO transform or preserve enough pose/OctoMap provenance to prove equivalence; save `.bt`/parameters/filter statistics in the package. |
| P1 | Save-wizard contract | No typed map-save progress/result includes final PCD path, package ID/version proposal, quality outcome, preview, and user decision. The existing String/Trigger Mapping Session API is insufficient for recoverable field workflow. |
| P1 | Quality gate | `validate_nav_map.py` checks files, dimensions, thresholds, and nonempty free/occupied cells. Add coordinate overlay, origin/bounds, PCD-to-PGM correspondence, robot footprint clearance, relocalization asset compatibility, Nav2 load, and replay/field acceptance gates. |
| P2 | Async job semantics | The local candidate action runs the offline conversion within a ROS action callback. Large PCD work needs a cancellable worker/job record, durable progress, disk-space checks, and restart recovery rather than blocking the node executor. |
| P2 | Terrain productionization | `agt_terrain_map_generator` has valuable staged components, but it is deliberately disabled. Do not select it in production until PatchSetSource through exporter is one reproducible job and it passes comparison/replay acceptance. |

## Recommended implementation locations

These are recommendations only; this audit makes no code change.

| Location | Recommended responsibility |
| --- | --- |
| `agt_mapping_session` or a new narrowly owned `agt_map_pipeline_orchestrator` | Replace the stub save path with a typed lifecycle/action state machine. It should be the only component allowed to declare a mapping run complete and initiate package creation. Keep FAST-LIO2/PGO algorithm processes unchanged. |
| `agt_mapping_bringup` | Export a machine-readable mapping-run manifest: LIO/PGO config hashes, run/session ID, PGO output directory, final PCD hash, optimized poses/patch hashes, frame, and OctoMap baseline provenance. Make save completion observable rather than inferred from files. |
| `agt_map_converter` | Keep the present converter as `projection_v1`; make input provenance mandatory for package use. Add a stable projection interface capable of accepting final optimized poses, ground/obstacle evidence, robot footprint and deterministic parameters. |
| `agt_map_manager` | Keep package validation and immutability here. Promote the local candidate pipeline only after it consumes a finalization manifest; store generator version/config/input hashes/quality evidence. Add durable job state and package publication only after all gates pass. |
| `agt_robot_interfaces` | Introduce typed mapping completion/save-wizard/job/activation acknowledgement messages or actions. Preserve current map list/load/edit interfaces for compatibility. |
| `agt_system_bringup` | Replace path resolution in HMI launch with an active-package runtime supervisor. For V1, stop/restart field components in a defined order and pass one locked generation; retain direct-path launches as explicit `bench_mode` only. |
| `agt_nav2_bringup` | Add an owned controlled-reload hook or supervisor protocol. Do not pretend map_server supports live package replacement. It must acknowledge the exact map ID/version/generation after restart. |
| `agt_global_relocalization` and `agt_localization_manager` | Keep the existing MapStatus binding, but add readiness acknowledgement and reject requests while a generation is changing. Enable manager following only when paired Nav2 lifecycle behavior is available. |
| External `agt_robot_hmi` integration layer | Implement API-driven package list/select/status and edit session flow. Restrict map rendering paths to server-issued responses; remove client-owned persistence of selected map paths. |

## Non-breaking migration plan

### Phase 0: freeze the current demo contract

- Preserve `rviz_field_demo.launch.py` direct `map`/`global_map` arguments and
  the manual OctoMap export script for bench recovery.
- Keep `agt_map_converter` as the regression generator and retain the current
  Map Package schema and HMI Edit Session contracts.
- Record each existing demo package's source PCD hash, map YAML/PGM hashes,
  mapping config, and known coordinate assumptions before changing activation.

### Phase 1: shadow finalization and package generation

- Add the orchestrator behind Mapping Session without changing FAST-LIO2, PGO,
  OctoMap, Nav2, or HMI launch behavior.
- After PGO save, generate a candidate package in a staging location only.
- Build relocalization assets from the same final PCD. Generate PGM through the
  PCD projection path, and optionally produce OctoMap baseline output for a
  side-by-side review, never auto-activate either result.
- Require a human save-wizard decision after machine validation; publish an
  immutable version only on approval.

### Phase 2: evidence gates and HMI read-only API adoption

- Add final-PGO manifest validation and an image/PCD coordinate-overlay check.
- Expose package list/status/load through a HMI adapter while keeping the
  generated HMI JSON path as a compatibility renderer input.
- HMI selection sends exact ID/version to Map Manager; the compatibility launch
  resolves the resulting locked generation. No free-form path is accepted.

### Phase 3: controlled runtime activation

- On selection, reject or defer when a mission is active. Stop/cancel Nav2
  work, bind relocalization and Nav2 to the same verified package generation,
  restart map_server/field consumers, request relocalization, then mark the
  generation applied only after acknowledgements.
- Keep live hot-switch disabled. A failed activation must leave the last known
  running generation intact and report the failed candidate clearly.

### Phase 4: Edit Session and terrain expansion

- Retain the existing edit contract: editing occupancy/topology creates a new
  package version while localization/relocalization assets remain unchanged.
- Add terrain generator outputs only as optional package layers after its
  offline acceptance suite proves behavior against `projection_v1` and the
  OctoMap baseline.

## Acceptance checklist for the planned end-to-end test

1. Run FAST-LIO2 plus PGO on a recorded/controlled mapping session; capture a
   final PCD, poses/patches, PGO config hash, and completion result.
2. Generate a candidate package without activation. Verify all hashes and that
   the PCD, relocalization assets, PGM/YAML, pipeline record, and quality report
   share one source revision.
3. Overlay final PCD obstacles/trajectory on PGM in the `map` frame. Confirm
   origin, extent, robot footprint corridor, and known-free/occupied regions;
   reject an OctoMap PGM if it cannot prove final-PGO alignment.
4. Start the existing direct field demo with the package's exact PCD/YAML.
   Verify map_server load, global relocalization, `map -> odom`, Nav2 planning,
   HMI/RViz goal handoff, mission execution, and camera action.
5. Select the same package through `/agt/map/load`, then start the HMI field
   demo. Confirm its runtime config ID/version/generation matches `MapStatus`.
6. Create an Edit Session, change only permitted navigation occupancy/topology,
   publish a new version, and verify immutable geometry plus localization asset
   reuse. Repeat controlled restart activation for that new version.
7. Inject failures: missing asset, hash mismatch, wrong PCD/PGM pairing,
   navigation map image escaping the package, relocalization build failure, and
   activation during a mission. Each must fail before the active running map is
   replaced.

## Decision record for the next design iteration

- **Authoritative V1 navigation source:** final PGO `global_map.pcd` projected
  under a recorded projection configuration and, where available, final
  optimized trajectory/robot geometry evidence.
- **OctoMap role:** retain as a separately versioned O1-H2 baseline/comparison
  artifact until a final-PGO coordinate equivalence/rebuild contract is
  implemented. Do not automatically combine a live projected map with a final
  PGO PCD.
- **Map Manager role:** authoritative identity, validation, immutable
  publication, selection, edit sessions, and generation state; it should not
  become a direct owner of FAST-LIO2 or PGO algorithms.
- **HMI role:** API client and presentation/editor for a server-issued session;
  it must not own active-map file paths or release-package mutation.
- **Runtime map switching:** controlled restart/acknowledged binding first;
  live hot switch is a later feature, not an implied property of
  `active_map.yaml`.
