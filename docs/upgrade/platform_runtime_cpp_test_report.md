# Platform Runtime C++ Test Report

Date: 2026-10-02
Overall V4 status: **PARTIAL** — P0–P7 software gates are recorded. P8 component checks, localization recorded-input replay, Nav2 mock-TF graph, and LIO-only bag replay passed. P8 full-stack readiness and every-layer restart remain `NOT_RUN`, so this branch is not field-ready.

## Workspace and source state

- Workspace: `/home/yangxuan/ros2_ws/src/agt_navigation_v3`
- Target branch: `refactor/navigation-runtime-v4`
- P8 baseline/parent commit: `88d7909` (`feat(nav): validate cross-platform Nav2 profiles`). The P8 implementation and this report are delivered by the commit that adds this report; use `git log -1 --oneline` for its SHA.
- ROS environment: Ubuntu 22.04, ROS 2 Humble (`/opt/ros/humble/setup.bash`).
- Initial P8 worktree was clean. P8 changes are limited to the LIO selector test, orderly shutdown handling and tests for the two localization owners, and the acceptance/report files. Build/install/log output was isolated under `/tmp/agt_runtime_v4_p*`; the shared workspace `build/` and `install/` were sourced as underlays and not written.
- A separate repository's isolated FAST-LIO parity test was active on `ROS_DOMAIN_ID=87` with `ROS_LOCALHOST_ONLY=1`. It was left running and untouched. Our ROS checks used localhost-only domains 225, 226, 227, 228, 229 and 230; no domain or physical topic was shared. The large bag replay started only after that external test exited. A full-stack graph was later attempted but did not reach readiness; complete full-stack acceptance remains `NOT_RUN`.

## Phase results

| Phase | Status | Result |
|---|---|---|
| P0 Source ownership and migration audit | PASS | Full navigation/workspace source audit recorded in `platform_runtime_cpp_audit.md`; ownership and migration dispositions are explicit. |
| P1 Robot Profiles | PASS | Bunker profile preserved; missing Ackermann geometry fails closed; YHS remains blocked. |
| P2 C++ Motion Guard | PASS | C++ build, Python baseline, guard unit tests and isolated Python/C++ shadow parity passed. Python rollback remains available. |
| P3 Base adapters | PASS | Common API, fake Bunker adapter, and Ackermann software conversion tests passed. No physical command path was exercised. |
| P4 Atomic launch | PASS | Isolated builds, source ownership tests and `--show-args` checks passed; live graph and restart remain unverified. |
| P5 Localization Manager | PASS | Feasibility audit selected `KEEP_PYTHON_THIS_RELEASE`; no migration was presumed. Correction math and owner-source tests passed. |
| P6 Sensor layer | PASS | Raw Livox timing path and separate PointCloud2 branch were audited; fake bridge graph passed. |
| P7 Nav2 profiles | PASS | Bunker contract and fail-closed Ackermann profile validation passed; no Nav2 graph or field profile was run. |
| P8 Integration | NOT_RUN | Component checks, localization recorded-input replay, Nav2 mock-TF graph, and LIO-only bag replay passed; full-stack readiness and every-layer restart are incomplete. |

Builds were performed in isolated `/tmp` directories. Packages built across phases were: `agt_system_bringup`, `agt_global_relocalization`, `agt_robot_description`, `agt_robot_bringup`, `agt_base_control`, `agt_base_runtime`, `agt_localization_manager`, `agt_livox_tools`, `agt_pointcloud_preprocessor`, and `agt_navigation_runtime`, plus the P4 set of 11 runtime/platform/description packages. Exact commands and result counts for P0–P7 are recorded in the corresponding phase sections of [`PLATFORM_RUNTIME_CPP_ACCEPTANCE.md`](../acceptance/PLATFORM_RUNTIME_CPP_ACCEPTANCE.md).

## Startup ownership before and after the refactor

| Responsibility | P0 source baseline | P4+ intended production ownership | P8 runtime evidence |
|---|---|---|---|
| Robot and sensor fixed TF | Description started through platform bringup or a legacy display path; duplicate entry points needed coordination. | `agt_robot_description/description.launch.py` is included once; `robot_state_publisher` owns fixed `/tf_static` links. | Started in LIO, localization, and Nav2 mock scenarios; no duplicate fixed-TF process was observed. Full production TF inventory remains `NOT_RUN`. |
| MID360 to navigation odometry | One selected FAST-LIO2 or Batch-LIO frontend, but legacy and standalone launch routes could collide. | One atomic `lio.launch.py` selects one C++ frontend; raw `/livox/lidar` and `/livox/imu` remain direct inputs; the selected adapter alone publishes `/agt/odometry/local` and `odom -> base_footprint`. | Full raw MID360 replay observed one canonical publisher (`agt_fastlio_adapter`), FRESH at about 10 Hz. |
| Global localization | Manager is the intended only `map -> odom` owner; global relocalization produces pose proposals. | One `agt_localization_manager` owns `map -> odom`; the global relocalizer has no TF authority. | Recorded-input test observed Manager freshness but intentionally produced no correction. A combined graph with no global pose stayed `WAIT_SENSORS`; live production owner counts remain `NOT_RUN`. |
| Motion command | System navigation directly instantiated Python guard; standalone guard/driver paths could duplicate `/mux/cmd_vel`. | One selected C++ guard by default (Python rollback retained) publishes canonical guarded command; one Bunker adapter forwards to `/mux/cmd_vel`; the separate physical Bunker driver remains the only CAN owner. | Fake Bunker graph observed one `/mux/cmd_vel` publisher. In the incomplete full graph, the fake driver saw 14,324 zero samples. Physical command/driver graph is `NOT_RUN`. |
| Nav2 | C++ Nav2 launch was reached through the system navigation launch; direct wrapper and aggregator combinations needed owner discipline. | Atomic `agt_nav2_bringup/nav2.launch.py` is included once; it starts no hardware. | Separate mock graph reached active lifecycle on five servers with map/action/TF/odom inputs. No goal was sent and `/cmd_vel` had no subscriber. |

P0 recorded the baseline risk: system aggregators directly constructed runtime
nodes while standalone launches offered alternate paths. P4 split these into
atomic launch files and tests require aggregators to include those atomics
without re-instantiating the same `Node`. Source/launch ownership tests passed.
P8 graph evidence is deliberately separated by scenario above; no mock TF or
bag publisher was counted as the production authority. The complete integrated
graph with the real selected LIO adapter, Localization Manager, guard, adapter,
Nav2 and fake Bunker driver did not reach readiness and remains `NOT_RUN`.

## P8 commands and results

### Isolated package build

`agt_navigation_runtime` build: **PASS**, 1 package.

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
source /tmp/agt_runtime_v4_p4/install/local_setup.bash
source /tmp/agt_runtime_v4_p6/install/local_setup.bash
source /tmp/agt_runtime_v4_p7/install/local_setup.bash
nice -n 19 colcon --log-base /tmp/agt_runtime_v4_p8/log build --base-paths /home/yangxuan/ros2_ws/src/agt_navigation_v3 --packages-select agt_navigation_runtime --build-base /tmp/agt_runtime_v4_p8/build --install-base /tmp/agt_runtime_v4_p8/install --merge-install --event-handlers console_direct+
```

### Unit/mock and isolated ROS evidence

The initial selected localization/LIO/supervisor regression subset: **PASS**, 57 passed. After shutdown hardening, the final selected P8 suite passed 59 tests; see the localization section below.

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
source /tmp/agt_runtime_v4_p4/install/local_setup.bash
source /tmp/agt_runtime_v4_p6/install/local_setup.bash
source /tmp/agt_runtime_v4_p7/install/local_setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="navigation/localization/agt_localization_manager:navigation/localization/agt_localization_core:navigation/state_estimation/agt_fastlio_adapter:navigation/state_estimation/agt_batch_lio_adapter:navigation/nav2/agt_navigation_runtime:runtime/agt_navigation_supervisor:${PYTHONPATH}"
ROS_DOMAIN_ID=225 python3 -m pytest -q navigation/localization/agt_localization_manager/test/test_correction_math.py navigation/localization/agt_localization_core/test/test_pose_math_legacy_parity.py navigation/state_estimation/agt_fastlio_adapter/test/test_frame_conversion.py navigation/state_estimation/agt_batch_lio_adapter/test/test_lio_calibration.py navigation/nav2/agt_navigation_runtime/test/test_lio_modes.py runtime/agt_navigation_supervisor/test/test_health_model.py
```

The LIO selector test was corrected to match the atomic `lio.launch.py` wrapper introduced in P4. It now verifies exactly one selected backend include. The final six-file run passed all 57 tests.

Fake Bunker base adapter graph: **PASS**, 1 test in isolated domain 226. This used the P4-installed adapter executable and fake test driver. It verified software adapter behavior; it did not connect to CAN or a physical Bunker.

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
source /tmp/agt_runtime_v4_p4/install/local_setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=226
export BUNKER_ADAPTER_EXECUTABLE=/tmp/agt_runtime_v4_p4/install/agt_base_runtime/lib/agt_base_runtime/bunker_adapter
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
python3 -m pytest -q platform/agt_base_runtime/test/test_bunker_adapter_integration.py
```

Synthetic FAST-LIO adapter graph: **PASS** in isolated domain 227. Six synthetic fresh inputs produced six canonical odometry messages paired with six `odom -> base_footprint` transforms. Original timestamps were preserved; 12 stale samples were rejected. All test topics were under `/test/fastlio_adapter`; no frontend, sensor, base or command publisher started.

MID360 bag to selected FAST-LIO2 and adapter: **PASS**, rosbag-replay evidence
in isolated domain 228. The full 265.213 s bag completed with exit 0. Only raw
`/livox/lidar` CustomMsg and `/livox/imu` were replayed. The canonical output
rate was about 10.0 Hz in 100-message windows. A near-end status sample was
`FRESH`, `publish_rate=10.0`, `accepted_count=2418`, `rejected_count=0`;
`/agt/odometry/local` had exactly one runtime publisher, `agt_fastlio_adapter`.
Only robot_state_publisher, the FAST-LIO2 frontend and adapter were started;
no base, guard, localization or Nav2 nodes ran. The exact command sequence is
in the P8 section of the linked acceptance matrix. This completes J02 only.

LIO atomic launch shutdown: **PASS** for two controlled start/stop cycles.
Both frontend and adapter started in each cycle; sending one SIGINT to the
launch parent yielded exit 0 without tracebacks or launch errors. This does
not cover all atomic layers, graph publisher counts after restart or orphan
inspection, so E12/J07 remain `NOT_RUN`. An earlier interactive terminal
interrupt produced an adapter traceback during `destroy_node()`; the
single-signal controlled run did not reproduce it.

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
source /tmp/agt_runtime_v4_p4/install/local_setup.bash
source /tmp/agt_runtime_v4_p6/install/local_setup.bash
source /tmp/agt_runtime_v4_p7/install/local_setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=227 AGT_RUN_ISOLATED_FASTLIO_TEST=1
export PYTHONPATH="/home/yangxuan/ros2_ws/src/agt_navigation_v3/navigation/state_estimation/agt_fastlio_adapter:${PYTHONPATH}"
python3 navigation/state_estimation/agt_fastlio_adapter/test/isolated_adapter_smoke.py
```

`colcon test --packages-select agt_navigation_runtime` returned exit code 0 but ran zero registered tests for this `ament_python` package. Registered package tests are therefore `NOT_RUN`; the direct pytest run above is the test evidence.

### Localization recorded-input and shutdown regression

The first active-cloud replay shutdown exposed an executor exception: rclpy's
default SIGINT handler shut down the context during `PointCloud2` subscription
processing. The first attempt failed this shutdown check. Both Python
localization owners now catch SIGINT and SIGTERM as `KeyboardInterrupt`, unwind
the executor, destroy the node, and then shut down the still-live context once.

The final isolated rebuild of `agt_localization_manager` and
`agt_global_relocalization` passed (2 packages):

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
source /tmp/agt_runtime_v4_p4/install/local_setup.bash
source /tmp/agt_runtime_v4_p6/install/local_setup.bash
source /tmp/agt_runtime_v4_p7/install/local_setup.bash
source /tmp/agt_runtime_v4_p8/install/local_setup.bash
nice -n 19 colcon --log-base /tmp/agt_runtime_v4_p8/rebuild2-log build --base-paths /home/yangxuan/ros2_ws/src/agt_navigation_v3 --packages-select agt_localization_manager agt_global_relocalization --build-base /tmp/agt_runtime_v4_p8/build --install-base /tmp/agt_runtime_v4_p8/install --merge-install --allow-overriding agt_localization_manager agt_global_relocalization --event-handlers console_direct+
```

The final direct regression run passed 59 tests:

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
source /tmp/agt_runtime_v4_p4/install/local_setup.bash
source /tmp/agt_runtime_v4_p6/install/local_setup.bash
source /tmp/agt_runtime_v4_p7/install/local_setup.bash
source /tmp/agt_runtime_v4_p8/install/local_setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="navigation/localization/agt_localization_manager:navigation/localization/agt_global_relocalization:navigation/localization/agt_localization_core:navigation/state_estimation/agt_fastlio_adapter:navigation/state_estimation/agt_batch_lio_adapter:navigation/nav2/agt_navigation_runtime:runtime/agt_navigation_supervisor:${PYTHONPATH}"
ROS_DOMAIN_ID=230 python3 -m pytest -q navigation/localization/agt_localization_manager/test/test_correction_math.py navigation/localization/agt_localization_manager/test/test_localization_manager_shutdown.py navigation/localization/agt_global_relocalization/test/test_global_relocalization_shutdown.py navigation/localization/agt_localization_core/test/test_pose_math_legacy_parity.py navigation/state_estimation/agt_fastlio_adapter/test/test_frame_conversion.py navigation/state_estimation/agt_batch_lio_adapter/test/test_lio_calibration.py navigation/nav2/agt_navigation_runtime/test/test_lio_modes.py runtime/agt_navigation_supervisor/test/test_health_model.py
```

Localization-only recorded-input replay passed with
`ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=229`. The graph launched Bunker
`robot_state_publisher`, `agt_localization_manager` with
`map_id:=bunker_mid360 map_version:=20260924-trav-integration-v1`, and
`agt_global_relocalization` with `auto_request:=false` and matching map
PCD/assets. It replayed only canonical odometry and PointCloud2:

```bash
nice -n 19 ros2 launch agt_robot_description description.launch.py robot:=bunker_v1
nice -n 19 ros2 launch agt_localization_manager localization_manager.launch.py use_sim_time:=true map_id:=bunker_mid360 map_version:=20260924-trav-integration-v1
nice -n 19 ros2 launch agt_global_relocalization global_relocalization.launch.py use_sim_time:=true auto_request:=false follow_map_manager:=false global_map:=/home/yangxuan/ros2_ws/maps/bunker_mid360/20260924-trav-integration-v1/localization/global_map.pcd relocalization_assets:=/home/yangxuan/ros2_ws/maps/bunker_mid360/20260924-trav-integration-v1/localization/relocalization selected_map_id:=bunker_mid360 selected_map_version:=20260924-trav-integration-v1 body_to_base_calibration_file:=/home/yangxuan/ros2_ws/src/agt_navigation_v3/navigation/nav2/agt_navigation_runtime/config/fastlio2_mid360_navigation.yaml
nice -n 19 ionice -c 3 ros2 bag play /home/yangxuan/ros2_ws/experiments/data/rosbag/nav_full_baseline_20260917_114327 --loop --clock 20 --disable-loan-message --topics /agt/odometry/local /agt/livox/points
ros2 topic info -v /agt/odometry/local
ros2 topic info -v /agt/livox/points
ros2 topic hz /agt/odometry/local --window 20
ros2 topic hz /agt/livox/points --window 20
ros2 topic echo --once /agt/localization/status
```

`ros2 topic info -v` showed one bag publisher per input, two subscriptions on
canonical odometry (Manager and Relocalization), and one point-cloud
subscription (Relocalization). `ros2 topic hz` measured approximately 10 Hz
on both. `/agt/localization/status` reported
`local_odom_fresh=true`, `global_correction_valid=false`,
`map_id=bunker_mid360`, `map_version=20260924-trav-integration-v1`, and
`reason=waiting_global_pose`. Automatic request was disabled and no request or
service was invoked. This passed the input contract only; it did not produce a
global pose or validate a production `map -> odom` transform.

After rebuilding, an active-input shutdown scenario passed. While the bag was
still publishing both input topics, the Relocalization launch parent received
SIGINT and exited 0; the Manager launch parent then received SIGINT while
canonical odometry remained active and also exited 0. Neither output contained
traceback, `[ERROR]`, `KeyboardInterrupt`, or message-conversion RuntimeError.
The bag player and description launch then exited 0. No base, Nav2, goal,
guard, hardware or physical command was present.

### Nav2-only mocked TF/odom graph

**PASS**, isolated domain 229. A test-only mock published 20 Hz
`map -> odom`, `odom -> base_footprint`, and zero `/agt/odometry/local` while
Bunker robot_state_publisher and standalone `agt_nav2_bringup/nav2.launch.py`
used the selected Bunker map and resolved P7 parameters. `/map_server`,
`/controller_server`, `/planner_server`, `/bt_navigator`, and
`/velocity_smoother` all reached lifecycle state `active [3]`;
`/navigate_to_pose` was available; a two-second TF probe saw 41 samples on
each dynamic edge; canonical odometry had one mock publisher. Four internal
Nav2 `/cmd_vel` publishers had zero subscribers. No goal, guard, base,
hardware or command was started. The mock TF owners were confined to this
Nav2-only graph.

### Exploratory full stack attempt

An isolated domain-231 graph launched description, localization with FAST-LIO2
and selected map assets, Nav2, a fake Bunker driver, and raw MID360 bag replay.
Automatic relocalization was deliberately disabled to avoid asking a moving
recorded trajectory to create a global pose. With no valid `map -> odom`
correction, the supervisor stayed `WAIT_SENSORS` and the global costmap stayed
`activating`. The fake driver counted 14,324 `/mux/cmd_vel` samples and all
were zero. No alternate transform or relocation request was injected. This
did not complete full-stack readiness, so J01 remains `NOT_RUN`. The first
active-cloud shutdown attempt emitted the executor error described above; the
post-fix active-input signal check passed.

## P8 scenario ledger

| Required scenario | Status | Evidence |
|---|---|---|
| Full Bunker profile + fake base + MID360 bag + LIO + localization + Nav2 | NOT_RUN | An exploratory combined graph did not reach readiness without a valid global correction; details above. |
| Sensor-only startup | PASS | P6 fake Livox CustomMsg bridge graph, no Nav2/device; matrix G05. |
| Base-only fake startup | PASS | P8 fake Bunker adapter graph, 1 test; no CAN/device. |
| LIO-only MID360 bag replay | PASS | Full 265.213 s raw MID360 replay; FAST-LIO2 adapter FRESH at 10 Hz, 2,418 accepted / 0 rejected in a near-end snapshot; canonical odometry publisher count 1. |
| Localization-only with recorded canonical odometry/cloud | PASS | Domain-229 replay, one publisher per input, expected subscriptions and `local_odom_fresh=true`; no request/correction was attempted. |
| Nav2-only with mocked TF/odom | PASS | Five Nav2 lifecycle servers active; map/action/TF/odom contracts observed; no goal or command subscriber. |
| C++ guard stale-command and health-loss injection | PASS | P2 shadow/core evidence: stale input produces zero/`STALE_COMMAND`; localization health loss produces zero/`LOCALIZATION_BLOCKED`. |
| Ackermann conversion | PASS | P3 synthetic-geometry software unit tests cover forward/reverse, turns, limits and singularity behavior. Geometry is test-only. |
| Duplicate publisher/TF ownership audit | PASS | P4 static source ownership tests passed. Runtime TF/publisher inspection remains `NOT_RUN`. |
| Shutdown/restart of each atomic layer | NOT_RUN | Controlled LIO and localization owner shutdown probes passed, but not every atomic layer was repeated with a post-stop orphan and publisher-owner audit; E12 remains `NOT_RUN`. |

## Acceptance status ledger

| Matrix section | Status |
|---|---|
| A01–A05 source/ownership | PASS |
| B01–B04, B06 Robot Profile | PASS |
| B05 YHS profile | BLOCKED |
| C01–C10 Motion Guard | PASS |
| D01–D08 base adapters | PASS |
| D09 YHS adapter | BLOCKED |
| E01–E11 atomic launch source/show-args/ownership | PASS |
| E12 shutdown/restart | NOT_RUN |
| F01–F05 live TF/odom publisher invariants | NOT_RUN |
| G01–G05 sensor contracts/mock bridge | PASS |
| H01–H02 and H05 localization audit/math/decision | PASS |
| H03–H04 localization parity/live authority graph | NOT_RUN |
| I01–I04 Nav2 profile contract | PASS |
| I05 YHS Nav2 profile | BLOCKED |
| J01 full stack and J07 full-layer restart | NOT_RUN |
| J03 localization recorded-input graph; J04 Nav2 mock-TF graph | PASS |
| J02 MID360 bag-to-LIO replay | PASS |
| J05–J06 guard fault injection; J08 compatibility launch | PASS |
| K01–K06 real hardware | NOT_RUN |
| K07 real YHS | BLOCKED |
| L01–L05 performance, soak and integrated graph latency | NOT_RUN |

There are no current acceptance gates with a `FAIL` result. Earlier test
assertion/import failures were fixed within their phase and the final stated
commands passed. The first active-point-cloud SIGINT attempt exposed an
executor shutdown exception; it was repaired and its post-fix active-stream
repeat passed. Full E12/J07 restart remains `NOT_RUN`.

## Platform compatibility and safety evidence

- **Bunker compatibility: PASS (software/mock only).** Existing 0.55 m/s and
  0.65 rad/s limits, skid-steer profile, `/wheel/odom`, 50 Hz command chain,
  Python guard rollback and `/mux/cmd_vel` adapter route are retained. The
  fake adapter graph and Python/C++ guard parity tests passed. Physical Bunker
  driver/CAN, remote priority, E-stop and motion were not tested.
- **Ackermann: PASS (software only), field readiness NOT_RUN.** Conversion and
  Nav2 contract tests use explicitly synthetic test geometry. No such values
  are installed as a production Robot Profile, and no real Ackermann vehicle
  or measured geometry was tested.
- **YHS: BLOCKED.** `YHS_BASE_PROTOCOL_AUDIT.md` records conflicting command
  gear/field semantics and missing vehicle-specific kinematics, units,
  watchdog and arbitration confirmation. No YHS profile/adapter was enabled
  and no command was sent.
- **TF evidence:** static source ownership tests identify the selected LIO
  adapter as the intended `odom -> base_footprint` owner and
  `agt_localization_manager` as the intended `map -> odom` owner. Bunker
  `publish_odom_tf=false` remains in the launch contract. P8 observed the
  expected dynamic edges only in the separate Nav2 mock graph; production
  TF-owner counts with LIO plus Manager remain `NOT_RUN`.
- No automatic real chassis or arm process/action ran. No motion command was
  sent to a real base.

## Rollback

For field validation, keep the robot stopped and select the retained Python
guard through the validated wrapper option:

```bash
bash scripts/run_field_stack.sh --mode navigation --motion-guard-backend python
```

This selection was verified by the P2/P3 wrapper dry-run tests; the command
above itself was not run against hardware. To roll back the P8 implementation,
stop the runtime and revert the P8 commit as a unit. Reverting earlier runtime
phases must be done in reverse commit order, stopping the runtime first; the
P4 description/platform commits are in separate repositories and must be
reverted there as separate changes. Keep the Python guard and compatibility
launches until a supervised field release authorizes their retirement.

## Remaining result lists

- `FAIL`: none currently. The initial active-cloud shutdown exception was
  resolved and its final repeat passed.
- `NOT_RUN`: P8 full-stack readiness; atomic restart/orphan check; live
  production TF/publisher counts;
  all physical Bunker, Ackermann and sensor acceptance; performance, CPU/RSS,
  latency and soak gates.
- `BLOCKED`: YHS protocol/kinematics and YHS Nav2/adapter/hardware gates.
- Detailed phase evidence and commands: [`PLATFORM_RUNTIME_CPP_ACCEPTANCE.md`](../acceptance/PLATFORM_RUNTIME_CPP_ACCEPTANCE.md).
