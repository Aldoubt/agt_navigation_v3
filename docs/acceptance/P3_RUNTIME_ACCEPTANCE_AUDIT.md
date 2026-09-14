# P3 Runtime Acceptance Audit

**Audit date:** 2026-09-13  
**Milestone:** v0.3.0 localization contract freeze complete  
**Scope:** runtime-acceptance preparation only; this document changes no source,
launch parameter, map asset, or runtime result.

## 1. Authority and audit boundary

This audit follows [CODEX_CONTEXT.md](../CODEX_CONTEXT.md),
[AUTHORITY_MATRIX.md](../AUTHORITY_MATRIX.md), the frozen
[TF convention](../contracts/TF_CONVENTION.md), and the machine-readable
[backend contract matrix](../contracts/backend_contract_matrix.yaml).  Where
older design or acceptance documents disagree, the frozen contract and current
runtime capability take precedence.

The authoritative localization contract is:

```text
formal PGO pose / mapping cloud: T_map_body / mapping_body
native global-localization output: T_map_body
ROS localization input:           T_map_base_link
publication conversion:           T_map_base_link = T_map_body * T_body_base_link
map -> odom authority:            agt_localization_manager only
```

`mapping_body` is a point-cloud and pose semantic contract. It is not a new
navigation TF root or a second `map` frame. The runtime navigation tree remains
`map -> odom -> base_link`.

## 2. Launch topology

### 2.1 Field path: `rviz_field_demo.launch.py`

The field launch is orchestration for the AGT software chain. It requires an
explicit Nav2 map YAML and matching global PCD; map hot-switching is deliberately
not part of this first field path.

| Area | Nodes/processes started by AGT launch | Inputs/dependencies | Owner/output boundary |
| --- | --- | --- | --- |
| Local odometry | `navigation_lio.launch.py`: Batch-LIO plus `agt_batch_lio_adapter` | Live timing-preserving Livox CustomMsg and IMU | Adapter publishes `/agt/odometry/local` and navigation `odom -> base_link` semantics. |
| Secondary cloud | `agt_livox_tools` bridge; obstacle preprocessor | Same Livox stream, secondary PointCloud2 branch | `/agt/livox/points`; obstacle branch `/agt/navigation/points_obstacles`. It must not replace the raw LIO input. |
| Global localization | `agt_global_relocalization` (default automatic request) | `/agt/livox/points`, global PCD, BBS/Polar assets, stationary gate, mapping-body contract | `/agt/relocalization/pose` and JSON status; never `map -> odom`. |
| Global correction | `agt_localization_manager` | Local odom plus accepted `T_map_base_link` pose | Sole publisher of `map -> odom`; `/agt/localization/status`. |
| Optional local tracking | `agt_map_tracker` only when `enable_map_tracking:=true` | Localized state, local query cloud, localization PCD/TF | Measurement/status path; runtime correction authority must be verified separately in P3. |
| Nav2 | AGT map server/lifecycle wrapper plus upstream `navigation_launch.py` | Nav2 YAML, static map, `map -> odom -> base_link`, local odom, obstacle cloud | Planner/controller/lifecycle services and Nav2 command output. AMCL is intentionally not launched. |
| Motion safety / operator path | `cmd_vel_guard`, navigation runtime, RViz patrol, RViz | Nav2 output, fresh localization status, mission inputs | Guard is the AGT safety boundary before external Bunker command consumption. |
| Optional diagnostics | RTK manager | RTK hardware/topic if enabled | Record/diagnostic only; must not alter `map` or relocalization. |

Hardware processes are intentionally **not** started by this launch: the MID360
driver, Bunker CAN/base driver, `robot_state_publisher`/URDF static transforms,
and C1 camera capability. They are P3 preflight dependencies, not optional
software details.

### 2.2 Deterministic replay path: `acceptance_offline_replay.launch.py`

Replay starts the offline PCD publisher, Batch-LIO/adaptor, Livox format bridge,
global relocalizer, LocalizationManager, optional MapTracker, Nav2, optional
replay auditor, and `ros2 bag play --clock`. It starts no CAN, base, camera, or
field command chain. A deterministic seed injector and pose-chain recorder are
optional evidence nodes only.

The offline path is therefore suitable for LIO latency, localization handoff,
TF availability, planner lifecycle, and open-loop tracker measurement checks.
It cannot prove controller-to-chassis closed-loop behavior.

### 2.3 Dependency order to audit at runtime

```text
hardware TF + Livox/IMU
  -> Batch-LIO -> Batch-LIO adapter -> /agt/odometry/local
  -> stationary query accumulation -> global relocalization
  -> accepted /agt/relocalization/pose -> LocalizationManager
  -> map -> odom plus odom -> base_link available
  -> Nav2 map/lifecycle/costmaps
  -> planner availability
  -> guarded motion path (field only)
```

The startup test must fail closed at every missing predecessor. In particular,
neither a BBS/GICP backend nor MapTracker may publish `map -> odom` directly.

## 3. Runtime TF and frame audit target

| Entity | Runtime meaning | Expected relation / producer | P3 observation |
| --- | --- | --- | --- |
| `map` | Global navigation/map frame | Parent of `odom`; correction published only by `agt_localization_manager` | Verify exactly one broadcaster and continuity after anchor. |
| `odom` | Continuous local odometry frame | Parent of `base_link`; Batch-LIO adapter provides navigation semantics | Verify fresh `/agt/odometry/local`, TF timestamp freshness, and no competing base/CAN odom TF. |
| `base_link` | Robot navigation reference | Child of `odom`; Nav2 `robot_base_frame` | Verify it agrees with chassis/URDF and Nav2 footprint reference. |
| `body` / `mapping_body` | Batch-LIO/PGO body semantic | `body` is the LIO state frame; `mapping_body` denotes the frozen query-cloud/pose contract, not an additional TF root | Verify one calibrated `T_body_base_link` is used at publication and never applied twice. |
| `livox_frame` | Driver compatibility/sensor frame | Driver/URDF compatibility child; navigation/localization LiDAR reference is `lidar_link` | Verify real MID360 mount transform, including tilt, is present; do not software-level the cloud. |
| `lidar_link`, `imu_link` | Physical LiDAR and actual LIO IMU frames | Static chassis/URDF calibration chain | Verify parent/child direction and actual IMU identity before LIO initialization. |

Expected navigation path:

```text
map --(LocalizationManager only)--> odom
odom --(Batch-LIO adapter semantics)--> base_link
base_link --(URDF/static calibration)--> lidar_link --> livox_frame
                                      \-> imu_link
```

The Batch-LIO native `camera_init -> body` state and the static calibrated
`body -> base_link` relation are diagnostic/calibration relations. They do not
authorize a second `odom -> base_link` publisher. P3 must record `tf2` frame
authority, parent/child IDs, timestamps, and availability for the chain above.

## 4. Nav2 startup and handoff

1. `agt_nav2_bringup/navigation.launch.py` validates the requested map YAML and
   AGT Nav2 params file.
2. It starts `map_server` and `lifecycle_manager_map`; `map_server` supplies the
   static occupancy map.
3. It includes upstream Nav2 `navigation_launch.py` without composition. This
   supplies the navigation lifecycle path (BT navigator, planner, controller,
   smoother, behavior server, waypoint follower, velocity smoother, costmaps,
   and navigation lifecycle manager).
4. AMCL/localization launch is intentionally absent. Nav2 consumes the external
   `map -> odom` correction and `/agt/odometry/local` instead.
5. Global costmap uses `map`; local costmap uses `odom`; both use `base_link`.
   The planner is `SmacPlanner2D`; controller baseline is Regulated Pure Pursuit
   at 50 Hz; the velocity smoother is 50 Hz. The field command is subsequently
   gated by `cmd_vel_guard` before the external Bunker driver.

P3 should record, not infer: map server ever-active/final state, navigation
lifecycle active state, planner action availability, costmap TF readiness, a
known-free planner-only goal, and then a field-safe guarded motion trial. A
rosbag planner lifecycle PASS is not a controller or chassis PASS.

## 5. Existing acceptance evidence

All current evidence is external to the immutable v003 map package under:

```text
/home/yangxuan/ros2_ws/agt_data/acceptance_evidence/
  bunker_mid360_mapping_20260901_205036/v003-indexed/
```

| Gate/evidence | Result available | P3 interpretation |
| --- | --- | --- |
| P0.7 base-link Nav2-footprint map audit, `p0_7/existing_map_audit.yaml` | `MAP=REVIEW`: swept 13245, free 13229, unknown 11, occupied 5, regions 3 | Valid map provenance and QA evidence exists, but the five occupied boundary cells still require human decision; do not call MAP field PASS. |
| P1.5 manual-seed full Case A, `p1_5_case_a/.../summary.yaml` | Software runtime READY: LIO PASS; localization PASS; Nav2 PASS; tracker disabled; `field_acceptance_ready=false` | Confirms replay handoff, TF and lifecycle under manual seed. Planner-only goal was not requested. |
| P1.6/P1.7 legacy tracker diagnostic | Legacy/base-frame tracking failed; open-loop artifacts retained | Historical failure is explanatory only; it must not be used as evidence for the frozen mapping-body contract. |
| P2.20 body-aligned tracker open-loop | MapTracker measurements accepted in the body-aligned path; run artifact includes a shortened/retry scenario | Useful measurement evidence, not field correction-authority acceptance. |
| P2.21 candidate BBS A/B, `p2_21_candidate_bbs_frame_ab/comparison.yaml` | `mapping_body`: localization PASS, 192/192 tracker open-loop `TRACKING_OK`, zero `RECOVERY_REQUIRED`; legacy base-link case failed | Strong freeze evidence for the mapping-body contract. The two automatic stationary queries were not identical, so it is not a same-cloud performance benchmark. |
| P2.21 source regression | Manual-seed and candidate BBS mapping-body defaults; mixed modes rejected; no duplicate body-to-base conversion | Contract regression coverage exists, but it is not a physical runtime gate. |
| Earlier offline/Gazebo acceptance in `ACCEPTANCE.md` | Historical replay and Gazebo functional PASS | Supporting evidence only; predates the current v0.3.0 contract freeze and must be revalidated where P3 depends on it. |

## 6. Missing P3 gates

The following have not been proven by the frozen-contract evidence and remain
open before claiming P3 or field readiness:

1. **MAP decision:** human review/decision for the three P0.7 conflict regions
   and five occupied cells; MAP remains `REVIEW`, not PASS.
2. **Real field topology:** launch with live MID360, real Bunker driver, URDF
   TF, and C1 capability; verify no unexpected TF or odom publisher.
3. **LIO runtime health:** target-hardware IMU preflight, stable initialization,
   rate/latency/backlog sampling, vibration and rough-motion evidence.
4. **Global localization repeatability:** at least five materially different
   starts/headings, including repeated geometry/tree-shadow conditions, using
   the frozen `mapping_body/mapping_body` contract.
5. **TF handoff quality:** verify one `map -> odom` owner, accepted-pose/local
   odom time alignment, and sustained `map -> odom -> base_link` availability.
6. **MapTracker authority:** the P2.21 result is open-loop
   (`apply_correction=false`). Before enabling tracker correction in the field,
   validate correction-applied behavior, HOLD/DEGRADED/RECOVERY_REQUIRED
   transitions, and retain automatic global recovery disabled.
7. **Nav2 field behavior:** map and navigation lifecycle, a known-free
   planner-only goal, then guarded closed-loop short-route behavior using the
   actual chassis. Validate the controller/velocity-smoother/guard chain rather
   than treating rosbag output as motion evidence.
8. **Safety and operator workflow:** fail-closed command behavior on stale or
   invalid localization, E-stop/watchdog integration, measured stop before
   capture, mission record validation, and explicit no-auto-recovery procedure.
9. **Physical targets:** the accuracy, speed, slope, endurance, and repeated
   target criteria in `PRE_ACCEPTANCE_GATE.md` remain physical measurements.

## 7. Recommended P3 evidence set and exit rule

For each field run, preserve immutable report directories containing launch
arguments, active map hashes/provenance, node graph, TF authority/timestamp
samples, localization/tracker JSONL, Nav2 lifecycle/action results, guard state,
and operator decision notes. Do not overwrite P0–P2 evidence or map assets.

The runtime dashboard should report at minimum:

```text
MAP          PASS / REVIEW / FAIL
LIO          PASS / WARN / FAIL
LOCALIZATION PASS / WARN / FAIL
TF           PASS / WARN / FAIL
MAP TRACKER  DISABLED / PASS / WARN / FAIL
NAV2 PLAN    PASS / WARN / FAIL
MOTION GUARD PASS / FAIL
```

`MAP=REVIEW` and a tracker disabled/open-loop experiment may permit an
instrumented software run, but neither permits a `FIELD READY` claim. The P3
exit condition is the ordered pre-acceptance sequence in
[PRE_ACCEPTANCE_GATE.md](PRE_ACCEPTANCE_GATE.md): all software gates pass, the
map review is resolved, closed-loop simulation/field safety gates pass, and
physical acceptance measurements are recorded.
