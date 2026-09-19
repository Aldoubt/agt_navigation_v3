# Localization Contract Baseline Audit

**范围：** `/home/yangxuan/ros2_ws/src/agt_navigation_v3`  
**目的：** 为 `legacy -> shadow -> v1` 定位迁移冻结代码级接口基线。  
**方法：** 仅读取源码、package manifest、launch、YAML 与 interface 定义；未启动节点、未读取运行时 ROS graph、未修改任何代码/launch/YAML/topic/TF/package。

因此，本文证明的是**代码声明的合同**。实际 publisher 数量、频率、QoS 匹配、TF 连续性仍须在 shadow 前用 launch 与 rosbag 再验证。

## 1. 定位模块目录树

```text
navigation/localization/
├── agt_global_relocalization/                 # Python ROS orchestration
│   ├── agt_global_relocalization/
│   │   ├── global_relocalization.py
│   │   └── manual_seed_relocalization.py
│   ├── config/global_relocalization.yaml
│   ├── launch/global_relocalization.launch.py
│   └── test/test_query_frame_contract.py
├── agt_global_relocalization_native/          # 3D-BBS / Polar Context / small_gicp native tools
│   ├── src/bbs_gicp_localizer.cpp
│   ├── src/candidate_bbs_gicp_localizer.cpp
│   ├── src/map_gicp_tracker.cpp
│   ├── src/build_relocalization_assets.cpp
│   └── src/build_relocalization_candidates.cpp
├── agt_localization_manager/                  # map->odom authority and recovery state machine
│   ├── agt_localization_manager/localization_manager.py
│   ├── config/localization_manager.yaml
│   ├── launch/localization_manager.launch.py
│   └── test/test_correction_math.py
├── agt_map_tracker/                            # low-rate local-map registration orchestrator
│   ├── agt_map_tracker/map_tracker.py
│   ├── config/map_tracker.yaml
│   ├── launch/map_tracker.launch.py
│   └── diagnostic utilities
└── agt_relocalization_benchmark/              # offline capture/sweep tooling; not field runtime
    ├── agt_relocalization_benchmark/{capture_cases,import_pgo_cases,sweep}.py
    └── config/relocalization_sweep.yaml
```

`agt_global_relocalization_native` provides the protected native algorithms. `agt_relocalization_benchmark` is offline tooling and has no role in the field localization TF chain.

## 2. Runtime component responsibilities and dependencies

| Component | Main responsibility | Direct ROS inputs | Direct outputs | Notable implementation dependency |
| --- | --- | --- | --- | --- |
| `agt_localization_manager` | validates observations; matches odom by timestamp; maintains correction/recovery state; publishes `map -> odom` | local odom, global pose, tracking pose/status, global backend status, map events | TF, typed status, metrics, JSON debug, relocalization request/service | `agt_robot_interfaces` |
| `agt_global_relocalization` | accumulates query cloud; executes existing global backend; validates result and publishes `T_map_base` | cloud, local odom, request, optional `MapStatus`, static chassis TF | global pose, JSON status, debug clouds/pose | `agt_global_relocalization_native`, `agt_batch_lio_adapter.extrinsics` |
| `agt_map_tracker` | accumulates cloud and runs existing local map GICP around prediction | cloud, local odom, typed localization status, TF lookup | tracking pose, JSON status | native `map_gicp_tracker` executable |
| `agt_global_relocalization_native` | whole-map BBS or Polar Context candidate selection + BBS + small_gicp; asset generation | CLI PCD/files | CLI JSON/files | PCL, Eigen, Boost |
| Batch-LIO adapter | canonicalizes Batch-LIO odom and publishes `odom -> base_link` | `/aft_mapped_to_init`, static mount TF | `/agt/odometry/local`, TF, adapter diagnostics | Batch-LIO calibration YAML + robot_description TF |
| FAST-LIO adapter | republishes or composes FAST-LIO body pose into canonical local odom | selected FAST-LIO odom, static mount TF in conversion mode | `/agt/odometry/local`; TF only in body-to-base mode | frozen body/lidar calibration + robot_description TF |

Dependency boundary to preserve during v1 migration:

```text
Global / local map observations  --->  Localization Manager  --->  map -> odom
Local LIO odometry               --->  LIO adapter           --->  odom -> base_link
```

Neither global relocalization nor map tracker uses a `TransformBroadcaster` in the audited source tree.

## 3. Frozen ROS interface contract

### 3.1 Localization Manager

Source: `navigation/localization/agt_localization_manager/agt_localization_manager/localization_manager.py` and `config/localization_manager.yaml`.

| Direction | Name | Type | Contract |
| --- | --- | --- | --- |
| subscribe | `/agt/odometry/local` | `nav_msgs/msg/Odometry` | Must be `header.frame_id=odom`, `child_frame_id=base_link`; 30 s buffer; global match skew default `0.10 s` |
| subscribe | `/agt/relocalization/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Must be in `map`; non-zero stamp; covariance gates apply |
| subscribe | `/agt/map_tracking/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Tracking correction candidate in `map` |
| subscribe | `/agt/map_tracking/status` | `std_msgs/msg/String` | JSON status from local tracker |
| subscribe | `/agt/global_relocalization/status` | `std_msgs/msg/String` | JSON backend diagnostic status only |
| subscribe | `/agt/map/events` | `std_msgs/msg/String` | JSON `MAP_ACTIVATED` event can invalidate correction |
| publish | `/tf`: `map -> odom` | `geometry_msgs/msg/TransformStamped` | Dynamic transform; default 30 Hz; exclusive owner |
| publish | `/agt/localization/status` | `agt_robot_interfaces/msg/LocalizationStatus` | Typed machine status; 5 Hz |
| publish | `/agt/localization/metrics` | `agt_robot_interfaces/msg/LocalizationMetrics` | Typed diagnostics; 5 Hz |
| publish | `/agt/relocalization/status` | `std_msgs/msg/String` | JSON debug representation of richer internal state |
| publish | `/agt/relocalization/request` | `std_msgs/msg/Empty` | global recovery request |
| service | `/agt/localization/relocalize` | `std_srvs/srv/Trigger` | clears correction and requests global relocalization |

**Important type fact:** `/agt/localization/status` is **not JSON**. It is `agt_robot_interfaces/msg/LocalizationStatus`. Existing JSON compatibility/debug status is `/agt/relocalization/status`; tracker and global backend use their own JSON `String` status topics. Any migration document or bridge that treats `/agt/localization/status` as `String` is incompatible with the current code.

### 3.2 Global relocalization

Source: `agt_global_relocalization/global_relocalization.py` and `config/global_relocalization.yaml`.

| Direction | Name | Type | Contract |
| --- | --- | --- | --- |
| subscribe | `/agt/livox/points` by default | `sensor_msgs/msg/PointCloud2` | query cloud; live launch may override topic |
| subscribe | `/agt/odometry/local` | `nav_msgs/msg/Odometry` | freshness/stationary gating |
| subscribe | `/agt/relocalization/request` | `std_msgs/msg/Empty` | starts one global request |
| subscribe | `/agt/map/status` | `agt_robot_interfaces/msg/MapStatus` | transient-local/reliable; used only when `follow_map_manager=true` |
| publish | `/agt/relocalization/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | accepted `T_map_base`; stamp comes from latest query cloud; covariance derived from score |
| publish | `/agt/global_relocalization/status` | `std_msgs/msg/String` | JSON backend/progress/result status |
| publish | `/agt/relocalization/query_cloud` | `sensor_msgs/msg/PointCloud2` | debug query cloud |
| publish | `/agt/relocalization/coarse_aligned_cloud` | `sensor_msgs/msg/PointCloud2` | debug cloud |
| publish | `/agt/relocalization/aligned_cloud` | `sensor_msgs/msg/PointCloud2` | debug cloud |
| publish | `/agt/relocalization/coarse_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | coarse result diagnostic |

Global result acceptance keeps the current score/fitness/overlap gates. Query frame mode defaults to `mapping_body`; the wrapper converts the native mapping-body result to `T_map_base` exactly once before publishing. This conversion and `T_body_base` resolution use the Batch-LIO calibration plus robot static mount TF; do not move or duplicate it during baseline migration.

### 3.3 Local map tracker

Source: `agt_map_tracker/map_tracker.py` and `config/map_tracker.yaml`.

| Direction | Name | Type | Contract |
| --- | --- | --- | --- |
| subscribe | `/agt/livox/points` | `sensor_msgs/msg/PointCloud2` | cloud is matched to nearest local odom; default skew `0.10 s` |
| subscribe | `/agt/odometry/local` | `nav_msgs/msg/Odometry` | must be `odom -> base_link` semantic pose |
| subscribe | `/agt/localization/status` | `agt_robot_interfaces/msg/LocalizationStatus` | waits for typed localized/degraded state and valid correction |
| TF lookup | `base_link <- cloud frame` at cloud stamp | TF lookup | cloud is transformed to base frame |
| TF lookup | `map <- odom` at latest (`Time(0)`) | TF lookup | predicts `T_map_base` for local registration |
| publish | `/agt/map_tracking/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | accepted tracking observation in `map`; only emitted when `apply_correction=true` |
| publish | `/agt/map_tracking/status` | `std_msgs/msg/String` | JSON `TRACKING_OK`/`DEGRADED`/`RECOVERY_REQUIRED`/hold diagnostics |

The tracker does not publish TF. It invokes the existing native executable `agt_global_relocalization_native/map_gicp_tracker` with an explicit PCD map path and then relies on the manager to gate and smooth the observation.

### 3.4 Typed interfaces already available

`interfaces/agt_robot_interfaces/msg/LocalizationStatus.msg` freezes these wire states:

```text
BOOT, WAIT_LOCAL_ODOM, WAIT_GLOBAL, LOCALIZED, DEGRADED, LOST, RELOCALIZING
```

Internal manager state is richer:

```text
BOOT, WAIT_GLOBAL, LOCALIZED, TRACKING, DEGRADED, LOST,
RECOVERY_REQUESTED, RELOCALIZING
```

`TRACKING` maps to wire `LOCALIZED`; `RECOVERY_REQUESTED` maps to wire `RELOCALIZING`. The full string state/reason is carried by `LocalizationMetrics` and JSON debug status. This mapping must be reproduced in a v1 shadow comparison before a typed replacement is introduced.

## 4. TF ownership and frame contract

### Required navigation chain

```text
map --(agt_localization_manager)--> odom
odom --(selected LIO adapter)-----> base_link
base_link --(robot_state_publisher)-> livox_frame / sensors
```

| Edge | Code owner | Conditions / notes |
| --- | --- | --- |
| `map -> odom` | `agt_localization_manager` | only broadcaster in localization tree; publishes only when correction exists and state is LOCALIZED/TRACKING/DEGRADED; stops refreshing when LOST |
| `odom -> base_link` (Batch-LIO) | `agt_batch_lio_adapter` | converts `camera_init -> body` source odometry; canonical output `/agt/odometry/local`; publishes TF after accepted input |
| `odom -> base_link` (FAST-LIO) | `agt_fastlio_adapter` | publishes only with `convert_body_to_base=true`; navigation FAST-LIO profile uses this conversion boundary |
| `odom -> camera_init` optional alias | `agt_batch_lio_adapter` | static identity only when `allow_parent_alias=true`; not the navigation chain contract |
| lidar/body mount edges | robot_state_publisher plus adapter lookup | static physical calibration input; not localization correction ownership |

No direct `map -> base_link` broadcaster was found in the audited localization source. No `map -> odom` broadcaster was found in global relocalization or map tracker.

**v1 shadow invariant:** a shadow manager may publish a diagnostic `TransformStamped` on a non-TF topic, but it must not use `TransformBroadcaster`. Exactly one node may ever publish dynamic `map -> odom` in a launch profile.

## 5. Map Package and file dependency contract

### Existing typed map handoff

`agt_robot_interfaces/msg/MapStatus` carries:

```text
active, map_id, map_version, package_path,
navigation_map_yaml, localization_map_pcd,
relocalization_assets_path, rtk_origin_yaml, generation, reason
```

`agt_global_relocalization` can consume it through `/agt/map/status` when `follow_map_manager=true`. It resolves `localization_map_pcd` and `relocalization_assets_path`, snapshots map id/version/generation before the backend run, then rechecks the snapshot before publishing a result.

### Existing direct-path consumers

| Consumer | Path inputs | Runtime behavior |
| --- | --- | --- |
| `agt_global_relocalization` | `global_map`, `relocalization_assets` | direct fallback or `MapStatus` values; requires PCD file and optional asset directory |
| `agt_map_tracker` | `global_map` | direct PCD file only; no `MapStatus` subscription |
| `rviz_field_demo.launch.py` | Nav2 `map.yaml`, `global_map.pcd`, optional assets directory | checks paths before nodes start; passes `follow_map_manager=false` to global relocalizer |
| `fastlio_anchor_nav.launch.py` | same three explicit assets | checks paths before nodes start; passes `follow_map_manager=false`; map tracker absent |

The current field baseline therefore supports both map-manager and explicit-path modes. A future Map Runtime Server must initially resolve to the **same absolute paths** and preserve the explicit-path launch mode until all field profiles have passed replay/vehicle testing.

### Relocalization asset expectations

The protected native asset generator writes at least `global_map_downsampled.pcd` and `relocalization_assets.yaml`. Candidate/POLAR mode additionally uses `polar_context.db` and `polar_context.yaml`. The global wrapper selects the Polar Context candidate backend only when the configured assets directory contains `polar_context.db`; otherwise it falls back to whole-map BBS/GICP. This fallback is existing behavior and must not be changed in the migration baseline.

## 6. Launch composition baseline

| Launch/profile | Local odom path | Global relocalization | Local tracker | Manager/TF owner | Map input mode |
| --- | --- | --- | --- | --- | --- |
| `rviz_field_demo.launch.py` | `navigation_lio.launch.py` -> Batch-LIO adapter | enabled | optional, default disabled | legacy manager | explicit paths, map manager disabled for global node |
| `offline_relocalization_demo.launch.py` | replay input / adapter as configured | enabled | not default | legacy manager | explicit PCD/assets |
| `acceptance_offline_replay.launch.py` | replay configuration | enabled | optional | legacy manager | explicit PCD/assets |
| `fastlio_anchor_nav.launch.py` | FAST-LIO2 -> FAST-LIO adapter | enabled, one-shot auto request | deliberately absent | legacy manager with tracking topics isolated | explicit paths |
| `system.launch.py` | selectable adapters | optional by flags | not included directly | legacy manager optional by flag | map manager can be enabled |

The field default `rviz_field_demo` keeps the raw timing-preserving Livox CustomMsg path for Batch-LIO and creates a separate PointCloud2 branch for relocalization/tracking. The migration must not place a filter in front of the LIO front-end.

## 7. State, timing and correction baseline

- manager buffers local odometry for 30 s and selects the closest sample to a global pose; default maximum skew is `0.10 s`;
- global poses require `map` frame, non-zero stamp, covariance, position std <= `1.0 m`, yaw std <= `20 deg` by default;
- manager computes `T_map_odom = T_map_base * inverse(T_odom_base)`;
- tracking observation gate defaults: translation `0.50 m`, yaw `5 deg`, with two consistent accepts before target update;
- default smoothing: time constant `3.0 s`, translation rate `0.10 m/s`, yaw rate `2 deg/s`;
- local odometry freshness thresholds are `0.30 s` stale and `1.00 s` lost;
- recovery request cooldown defaults to `5.0 s`;
- tracker itself runs at `0.5 Hz` by default and treats repeated rejects as DEGRADED/RECOVERY_REQUIRED;
- manager, not tracker, owns correction acceptance, smoothing, recovery request and dynamic TF publication.

These values are baseline data, not v1 tuning recommendations. A v1 policy file must import them unchanged before parity is proved.

## 8. Migration constraints derived from the baseline

### Required compatibility rules

1. Retain all topic names and exact message types in Section 3.
2. Retain map/body/base conversion location and the global pose timestamp semantics.
3. Do not replace direct PCD access in the same patch that changes manager ownership.
4. Do not make `agt_map_tracker` publish TF or allow a v1 tracker to bypass manager gates.
5. Keep `MapStatus` generation snapshot checks when map resolution moves behind a runtime service.
6. Preserve the raw LIO path and the separate PointCloud2 query path.
7. Treat `LocalizationStatus` wire-state compression as an explicit bridge rule, not an accidental v1 behavior.

### Shadow comparison signals

Before v1 active mode, record and compare:

- accepted/rejected global pose decisions and reasons;
- nearest-odom stamp and global/odom skew;
- `T_map_odom` translation and yaw at every legacy TF publication;
- internal state, `LocalizationStatus` wire state, metrics state/reason;
- tracking innovation, consistency count, recovery request and cooldown decisions;
- map id/version/generation used for each global result;
- TF publisher identity for `map -> odom` and selected adapter identity for `odom -> base_link`.

### High-risk coupling to isolate, not change

| Coupling | Evidence | Migration treatment |
| --- | --- | --- |
| Global relocalizer imports Batch-LIO extrinsic helpers | `global_relocalization.py` imports `agt_batch_lio_adapter.extrinsics` | Keep import/path in first v1 phase; later extract only proven calibration utility without changing math |
| tracker consumes typed manager status | `map_tracker.py` subscribes `LocalizationStatus` | v1 manager must preserve this exact topic/type before tracker facade changes |
| tracker uses latest `map <- odom` plus cloud-time base TF | `map_tracker.py` TF buffer lookups | shadow must retain lookup time policy and report failures separately |
| field launch overrides map manager following | `rviz_field_demo.launch.py`, `fastlio_anchor_nav.launch.py` pass `follow_map_manager=false` | Map Runtime cannot become mandatory until explicit path profiles are accepted |
| FAST-LIO anchor deliberately disables tracker recovery | `fastlio_anchor_nav.launch.py` isolates tracker topics | v1 must retain this as a distinct profile, not a global default |

## 9. Baseline conclusion

The current architecture already has the essential ownership split required by `agt_localization_v1`:

```text
Global Relocalization / Local Tracker: observations only
Localization Manager:                  sole map -> odom authority
LIO adapter:                           sole odom -> base_link authority
```

The safe first migration patch is therefore a non-TF shadow path that consumes existing observations and local odom, calculates the same correction/state results, and emits only namespaced diagnostics. It must leave the legacy manager, native algorithms, Map Package paths, and launch defaults active.

No source, launch, YAML, topic, TF logic, or package was changed during this audit.
