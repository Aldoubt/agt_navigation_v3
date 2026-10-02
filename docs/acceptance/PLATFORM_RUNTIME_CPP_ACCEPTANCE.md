# Platform Runtime C++ Acceptance Matrix

Target branch: `refactor/navigation-runtime-v4`

This matrix defines the evidence required before the platform-runtime refactor can be called complete. Build success alone is insufficient. Each row must be marked `PASS`, `FAIL`, `NOT_RUN`, or `BLOCKED`, with a command/log/report reference.

## A. Source and ownership audit

| ID | Gate | Evidence required | Result |
|---|---|---|---|
| A01 | Runtime node inventory | `platform_runtime_cpp_audit.md` lists production nodes, language, interfaces and migration class | PASS — `docs/upgrade/platform_runtime_cpp_audit.md`; source inventory check below |
| A02 | Singleton ownership inventory | unique intended owners for robot_state_publisher, MID360, base driver, LIO, localization manager, motion guard, Nav2 | PASS — intended owners and alternate launch paths documented; `test_unique_owner_inclusions` passed. Runtime graph inspection remains NOT_RUN. |
| A03 | No unreviewed Python mass rewrite | migration matrix contains rationale per node | PASS — per-node dispositions and P5 deferral are documented in the audit |
| A04 | Baseline command/TF contract frozen | documented current Bunker command chain and TF ownership | PASS — Bunker command, odom, and TF contract table is in the audit; no runtime/hardware claim is made |
| A05 | V1 field initialization excludes `/initialpose` and RTK seeding | field entry point selects automatic global relocalization only | PASS — `run_field_stack.sh` accepts only `auto`; system localization selects only `global_relocalization`; manual seed tools are outside the field launch. Targeted tests below passed. |

P0 static evidence actually executed:

- `python3 scripts/check_v4_source_ownership.py --source-root .` — PASS; 35 ROS packages, one Git root, all manifests tracked.
- `source /opt/ros/humble/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="map_data_manager/agt_map_manager:${PYTHONPATH}" python3 -m pytest -q bringup/agt_system_bringup/test/test_launch_ownership_static.py bringup/agt_system_bringup/test/test_yhs_nav_config_guard.py` — PASS; 10 passed.
- `source /opt/ros/humble/setup.bash && source /home/yangxuan/ros2_ws/install/setup.bash && colcon --log-base /tmp/agt_nav_runtime_v4_p0_log build --base-paths . --packages-select agt_system_bringup agt_global_relocalization --build-base /tmp/agt_nav_runtime_v4_p0_build --install-base /tmp/agt_nav_runtime_v4_p0_install --merge-install` — PASS; 2 packages built into isolated `/tmp` directories, leaving workspace build/install untouched.
- `source /opt/ros/humble/setup.bash && source /home/yangxuan/ros2_ws/install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="navigation/localization/agt_global_relocalization:map_data_manager/agt_map_manager:${PYTHONPATH}" python3 -m pytest -q navigation/localization/agt_global_relocalization/test/test_initialization_modes.py bringup/agt_system_bringup/test/test_run_field_stack_robot_config.py bringup/agt_system_bringup/test/test_launch_ownership_static.py bringup/agt_system_bringup/test/test_yhs_nav_config_guard.py` — PASS; 61 passed. Includes dry-run configuration checks only; no ROS graph or device launch.
- `bash -n scripts/run_field_stack.sh scripts/localization_initialization.sh && git diff --check` — PASS.
- Runtime graph and real hardware checks — NOT_RUN.

Execution notes: two earlier pytest invocations were collection/import environment failures before the sourced static-test command; the first let ROS `launch_testing` auto-collect an uninstalled package, and the second omitted ROS/workspace Python paths. The combined P0 test command initially had 60 passes and one overbroad assertion failure because its static check treated the help text's negative `/initialpose` warning as an active runtime path. The assertion was narrowed to check executable fallback wiring, then the full 61-test command above passed. The first isolated colcon attempt put `--log-base` after the `build` subcommand and was rejected by colcon argument parsing; the corrected command above built both selected packages successfully.

## B. Robot Profile and kinematics

| ID | Gate | Evidence required | Result |
|---|---|---|---|
| B01 | `bunker_v1` validates | existing verified Bunker fields preserved | PASS — profile schema test and whole-robot resolver preserve the existing 0.55 m/s and 0.65 rad/s limits and launch argument set |
| B02 | Bunker is classified skid-steer | explicit kinematic type, not inferred by vendor branch at runtime | PASS — `bunker_v1.base.kinematics=skid_steer`; resolver returns the profile unchanged |
| B03 | Ackermann required fields enforced | invalid/missing wheelbase or steering limits rejected | PASS — schema rejects missing wheelbase, steering bounds/rate and minimum turning radius; test-only geometry is not installed as a profile |
| B04 | Ackermann rotate-in-place disabled | schema and navigation profile reject impossible spin assumption | PASS — Ackermann schema requires `rotate_in_place=false` and tests reject `true` |
| B05 | YHS classification evidence-based | protocol/geometry audit or explicit BLOCKED state | BLOCKED — `yhs_harvesting` remains blocked, no `yhs_v1.yaml` or kinematic class exists, and the resolver test verifies no fallback. See `docs/upgrade/YHS_BASE_PROTOCOL_AUDIT.md`. |
| B06 | No guessed safety-critical defaults | tests demonstrate missing physical values fail closed | PASS — Ackermann requires wheelbase, steering bounds/rate, turning radius, footprint dimensions, safety margin and positive motion limits |

P1 execution evidence (2026-10-02; unit/mock only, no ROS graph or hardware launch):

- `agt_robot_description` build — PASS, 1 package:

  ```bash
  source /opt/ros/humble/setup.bash
  colcon --log-base /tmp/agt_runtime_v4_p1_description_log build --base-paths /home/yangxuan/ros2_ws/src/agt_robot_description --packages-select agt_robot_description --build-base /tmp/agt_runtime_v4_p1_description_build --install-base /tmp/agt_runtime_v4_p1_description_install --merge-install
  ```

- `agt_robot_description` tests — PASS, 22 tests, 0 failures:

  ```bash
  source /opt/ros/humble/setup.bash
  colcon --log-base /tmp/agt_runtime_v4_p1_description_log test --base-paths /home/yangxuan/ros2_ws/src/agt_robot_description --packages-select agt_robot_description --build-base /tmp/agt_runtime_v4_p1_description_build --install-base /tmp/agt_runtime_v4_p1_description_install --merge-install --event-handlers console_direct+
  colcon test-result --test-result-base /tmp/agt_runtime_v4_p1_description_build --verbose
  ```

- `agt_robot_bringup` build — PASS, 1 package. The existing workspace underlay supplied its dependencies; build/install/log outputs went to `/tmp`:

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  colcon --log-base /tmp/agt_runtime_v4_p1_platform_log build --base-paths /home/yangxuan/ros2_ws/src/agt_robot_platform --packages-select agt_robot_bringup --build-base /tmp/agt_runtime_v4_p1_platform_build --install-base /tmp/agt_runtime_v4_p1_platform_install --merge-install
  ```

- `agt_robot_bringup` tests — PASS, 38 tests, 0 failures:

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  colcon --log-base /tmp/agt_runtime_v4_p1_platform_log test --base-paths /home/yangxuan/ros2_ws/src/agt_robot_platform --packages-select agt_robot_bringup --build-base /tmp/agt_runtime_v4_p1_platform_build --install-base /tmp/agt_runtime_v4_p1_platform_install --merge-install --event-handlers console_direct+
  colcon test-result --test-result-base /tmp/agt_runtime_v4_p1_platform_build --verbose
  ```

No Ackermann bag, live ROS graph, physical chassis, CAN command or robot motion test was run in P1. Real Bunker remote/E-stop acceptance remains `NOT_RUN`; YHS remains `BLOCKED`.

## C. C++ Motion Guard

| ID | Gate | Evidence required | Result |
|---|---|---|---|
| C01 | C++ package builds | clean/isolated colcon build log | PASS — `agt_base_runtime` built in `/tmp/agt_runtime_v4_p2_runtime_build` |
| C02 | velocity clamp parity | old/new test or shadow comparison | PASS — forward, reverse and angular clamps match Python in the isolated shadow trace |
| C03 | slew-limit parity | positive/negative acceleration cases | PASS — positive/negative slew and separate linear deceleration cases pass in Python baseline and C++ core tests |
| C04 | output refresh | measured ~50 Hz software target under test load | PASS — C++ shadow output median period 20.01 ms (50 Hz target) |
| C05 | stale command -> zero | deterministic automated test | PASS — stale output is zero and state is `STALE_COMMAND` |
| C06 | localization block -> zero | deterministic automated test | PASS — `STATE_LOST` stops immediately and state is `LOCALIZATION_BLOCKED` |
| C07 | health/payload block -> zero | deterministic automated test | PASS — stale local odom and missing/false/stale payload permission stop output; state is `HEALTH_BLOCKED` |
| C08 | recovery state transition | blocked -> ready/active only after valid inputs | PASS — localization and payload recovery require a fresh command before output resumes |
| C09 | exactly one vendor command publisher | ROS graph or static owner test | PASS — system launch has one selected guard writing `/agt/base/cmd_vel` and one Bunker adapter writing `/mux/cmd_vel`; isolated fake-driver graph saw one `/mux` publisher. Real field graph inspection is NOT_RUN. |
| C10 | Python rollback path documented | explicit command/config for rollback during field test | PASS — `--motion-guard-backend python` is validated and shown by the field-wrapper dry-run; the navigation launch receives it through `AGT_MOTION_GUARD_BACKEND` and selects the retained `agt_base_control/cmd_vel_guard` action |

P2 freeze and verification evidence (2026-10-02):

The old Python guard semantics were frozen first with deterministic isolated-topic tests. The matrix covered forward/reverse/angular clamp, positive and negative slew, distinct linear deceleration, 50 Hz timer target, startup with no command/status, command reception-time freshness, stale timeout, localization and payload interlocks, health loss, no replay after recovery, fresh-command recovery, zero-command slew behavior and shutdown zero publication. `CmdVelGuard.main()` was inspected for its three shutdown zero publishes; the C++ shadow integration test observed at least three final zero samples on SIGINT.

The Python/C++ shadow test runs both guards in a dedicated `ROS_DOMAIN_ID`, consumes the same synthetic Twist/status/permission stream, and writes only to namespaced shadow topics. Its sample matcher pairs nearest timestamps within 25 ms and allows at most 0.035 m/s linear and 0.035 rad/s angular error. The recorded run aligned 103 samples; maximum errors were 0.009026 m/s and 0.016045 rad/s; C++ steady active-output median period was 20.01 ms. It verified `READY`, `ACTIVE`, `STALE_COMMAND`, `LOCALIZATION_BLOCKED` and `HEALTH_BLOCKED`. No base driver subscribed in that isolated domain.

- `agt_base_control` build — PASS, 1 package, isolated build/install/log directories:

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  colcon --log-base /tmp/agt_runtime_v4_p2_base_control_log build --base-paths /home/yangxuan/ros2_ws/src/agt_navigation_v3 --packages-select agt_base_control --build-base /tmp/agt_runtime_v4_p2_base_control_build --install-base /tmp/agt_runtime_v4_p2_base_control_install --merge-install
  ```

- Python baseline and payload-interlock tests — PASS, 16 tests:

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  ROS_DOMAIN_ID=208 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="bringup/agt_base_control:${PYTHONPATH}" python3 -m pytest -q bringup/agt_base_control/test/test_guard_semantics.py bringup/agt_base_control/test/test_guard_payload_interlock.py
  ```

- Existing in-process guard acceptance — PASS; stop latency 1.6 ms, zero after 80 ms while LOST, no stale replay, fresh recovery and manual mode handoff all passed. It used `ROS_DOMAIN_ID=209`; `/mux/cmd_vel` had no driver subscriber:

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  ROS_DOMAIN_ID=209 PYTHONPATH="bringup/agt_base_control:${PYTHONPATH}" python3 bringup/agt_base_control/test/guard_fail_closed_acceptance.py
  ```

- `agt_base_runtime` build — PASS, 1 package, using the installed interface underlay:

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  colcon --log-base /tmp/agt_runtime_v4_p2_runtime_log build --base-paths /home/yangxuan/ros2_ws/src/agt_navigation_v3 --packages-select agt_base_runtime --build-base /tmp/agt_runtime_v4_p2_runtime_build --install-base /tmp/agt_runtime_v4_p2_runtime_install --merge-install
  ```

- `agt_base_runtime` tests — PASS, 9 C++ core tests plus the Python/C++ shadow scenario; `colcon test-result` reported 12 test cases, 0 failures. Shadow used `ROS_DOMAIN_ID=207`:

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  ROS_DOMAIN_ID=207 colcon --log-base /tmp/agt_runtime_v4_p2_runtime_log test --base-paths /home/yangxuan/ros2_ws/src/agt_navigation_v3 --packages-select agt_base_runtime --build-base /tmp/agt_runtime_v4_p2_runtime_build --install-base /tmp/agt_runtime_v4_p2_runtime_install --merge-install --event-handlers console_direct+
  colcon test-result --test-result-base /tmp/agt_runtime_v4_p2_runtime_build --verbose
  ```

- Shadow metrics captured with pytest output enabled — PASS, 1 scenario:

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  ROS_DOMAIN_ID=206 MOTION_GUARD_CPP_EXECUTABLE=/tmp/agt_runtime_v4_p2_runtime_build/agt_base_runtime/motion_guard PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="bringup/agt_base_control:${PYTHONPATH}" python3 -m pytest -s -q platform/agt_base_runtime/test/test_shadow_parity.py
  ```

- `agt_system_bringup` build — PASS, 1 package; C++ package install overlay supplied `agt_base_runtime`:

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  source /tmp/agt_runtime_v4_p2_runtime_install/setup.bash
  colcon --log-base /tmp/agt_runtime_v4_p2_system_log build --base-paths /home/yangxuan/ros2_ws/src/agt_navigation_v3 --packages-select agt_system_bringup --build-base /tmp/agt_runtime_v4_p2_system_build --install-base /tmp/agt_runtime_v4_p2_system_install --merge-install --allow-overriding agt_system_bringup
  ```

- Launch ownership/config and field-wrapper dry-run tests — PASS, 34 tests; no launch process or ROS graph was started:

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  source /tmp/agt_runtime_v4_p2_runtime_install/setup.bash
  source /tmp/agt_runtime_v4_p2_system_install/setup.bash
  ROS_DOMAIN_ID=212 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="bringup/agt_base_control:map_data_manager/agt_map_manager:${PYTHONPATH}" python3 -m pytest -q bringup/agt_system_bringup/test/test_payload_interlock_launch.py bringup/agt_system_bringup/test/test_launch_ownership_static.py bringup/agt_system_bringup/test/test_yhs_nav_config_guard.py bringup/agt_system_bringup/test/test_payload_interlock_resolution.py bringup/agt_system_bringup/test/test_run_field_stack_robot_config.py
  ```

- Field wrapper syntax and rollback selection — PASS. `bash -n scripts/run_field_stack.sh` passed. Dry-run coverage verified the C++ default, Python selection in inspection mode, and rejection of an unknown backend before stack startup. The wrapper exports the validated choice for nested launch processes. Example rollback command:

  ```bash
  bash scripts/run_field_stack.sh --mode navigation --motion-guard-backend python --dry-run
  ```

  The final `agt_system_bringup` rebuild used its isolated P2 build/install/log directories and finished with 1 package. The initial rebuild reported the existing underlay package override; the final command explicitly acknowledged it with `--allow-overriding agt_system_bringup`. The dry-run suite did not start `ros2 launch`, any hardware driver, or a motion command.

Execution notes: initial Python baseline test invocation had 2 fixture failures plus 9 ROS-context setup errors; the tests were changed to shut down the context between cases and set declared parameters before assertions, then all 16 passed. The first C++ build failed on a const ROS-clock accessor and a test namespace brace; both compile errors were corrected. The first C++ core run exposed a slew fixture that stopped refreshing the command after its 0.25 s timeout; the fixture now refreshes each synthetic cycle. Shadow testing also exposed that rclcpp's default SIGINT handler closed the ROS context before exit zeros were published; the node now handles SIGINT/SIGTERM, sends three zeros, then shuts down. The final build, core/shadow suite and system launch tests all passed.

P2 runtime graph and physical base tests — `NOT_RUN`. No real `/mux/cmd_vel` connection, CAN message, chassis command or robot motion was made. Bunker manual/remote/estop priority is unchanged in source and remains a real-hardware `NOT_RUN` gate. The live process graph for either launch backend remains `NOT_RUN`; single-action ownership is supported by launch source and static/dry-run tests.

## D. Base Runtime adapters

| ID | Gate | Evidence required | Result |
|---|---|---|---|
| D01 | Common adapter API contains no vendor packet types | code review/static inspection | PASS — `base_adapter.hpp` exposes standard Twist/Odometry and vendor-neutral profile/command types; Bunker messages are consumed only inside `bunker_adapter_node.cpp`. |
| D02 | Bunker command parity | fake/replay test maps canonical Twist to current `/mux/cmd_vel` behavior | PASS — isolated fake-driver graph verifies planar Twist passthrough, single `/mux` publisher, 50 Hz output, stale-command zero and shutdown zeros. Real CAN/driver behavior is NOT_RUN. |
| D03 | Bunker odom preserved | `/wheel/odom` remains available and no competing odom TF appears | PASS — fake `/wheel/odom` fields are preserved on `/agt/base/odom`; adapter graph has no `/tf` or `/tf_static` publisher. Physical graph inspection is NOT_RUN. |
| D04 | Remote/manual priority untouched | architecture/code evidence; real validation separately | PASS — adapter reports raw Bunker/remote feedback for diagnostics and does not gate or rewrite arbitration. Physical remote/estop priority remains NOT_RUN. |
| D05 | Ackermann straight/turn/reverse tests | unit tests with synthetic verified-in-test geometry | PASS — software unit tests cover forward/reverse, straight, left and right turns with a test-only profile. |
| D06 | Ackermann saturation/rate tests | steering angle and rate boundaries | PASS — minimum turn radius, steering bounds and steering-rate limiting are tested. |
| D07 | Ackermann low-speed singularity handled | zero/near-zero speed test | PASS — minimum effective speed bounds curvature and behavior stays continuous across the synthetic low-speed threshold. |
| D08 | Ackermann in-place request handled safely | no infinite/invalid steering output | PASS — zero-speed rotation stays stopped, finite and explicitly flagged as rejected. |
| D09 | YHS adapter only if protocol verified | otherwise explicit BLOCKED | BLOCKED — protocol and steering semantics still conflict; see `docs/upgrade/YHS_BASE_PROTOCOL_AUDIT.md`. |

P3 execution evidence (2026-10-02; isolated software/fake-driver tests only):

- `agt_base_runtime` build — PASS, 1 package; compiled the common adapter core, Bunker adapter node, Ackermann conversion library, C++ Motion Guard and tests. The first compile attempt found that this Humble `rclcpp::Time` has no `to_msg()` method; the diagnostic timestamp now uses Humble's supported conversion and the isolated rebuild passed.

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  colcon --log-base /tmp/agt_runtime_v4_p3_base_log build --base-paths /home/yangxuan/ros2_ws/src/agt_navigation_v3 --packages-select agt_base_runtime --build-base /tmp/agt_runtime_v4_p3_base_build --install-base /tmp/agt_runtime_v4_p3_base_install --merge-install --event-handlers console_direct+
  ```

- Runtime tests — PASS; 4/4 CTest targets and `colcon test-result` reported 22 tests, 0 errors/failures/skips. That includes 9 Motion Guard unit cases, 7 adapter unit cases, Python/C++ shadow parity and a fake Bunker driver ROS test. CTest assigns `ROS_DOMAIN_ID=217` to guard shadow and `ROS_DOMAIN_ID=216` to the fake base graph; the enclosing test command used domain 215. No vendor driver or hardware topic was launched.

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  ROS_DOMAIN_ID=215 colcon --log-base /tmp/agt_runtime_v4_p3_base_log test --base-paths /home/yangxuan/ros2_ws/src/agt_navigation_v3 --packages-select agt_base_runtime --build-base /tmp/agt_runtime_v4_p3_base_build --install-base /tmp/agt_runtime_v4_p3_base_install --merge-install --event-handlers console_direct+
  colcon test-result --test-result-base /tmp/agt_runtime_v4_p3_base_build --verbose
  ```

- System launch build and static/dry-run tests — PASS, 1 package built and 35 tests passed. `bash -n scripts/run_field_stack.sh` passed. The field wrapper exports the whole-robot resolver's `base_adapter`; navigation defaults to Bunker only for `bunker_v1`, puts the chosen guard output on `/agt/base/cmd_vel`, and starts exactly one Bunker adapter. Both guard implementations and the installed safety config now default to `/agt/base/cmd_vel`. Ackermann and YHS live adapters fail closed.

  ```bash
  bash -n scripts/run_field_stack.sh
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  source /tmp/agt_runtime_v4_p3_base_install/setup.bash
  colcon --log-base /tmp/agt_runtime_v4_p3_system_log build --base-paths /home/yangxuan/ros2_ws/src/agt_navigation_v3 --packages-select agt_system_bringup --build-base /tmp/agt_runtime_v4_p3_system_build --install-base /tmp/agt_runtime_v4_p3_system_install --merge-install --allow-overriding agt_system_bringup
  source /tmp/agt_runtime_v4_p3_system_install/setup.bash
  ROS_DOMAIN_ID=218 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="bringup/agt_base_control:map_data_manager/agt_map_manager:${PYTHONPATH}" python3 -m pytest -q bringup/agt_system_bringup/test/test_payload_interlock_launch.py bringup/agt_system_bringup/test/test_launch_ownership_static.py bringup/agt_system_bringup/test/test_yhs_nav_config_guard.py bringup/agt_system_bringup/test/test_payload_interlock_resolution.py bringup/agt_system_bringup/test/test_run_field_stack_robot_config.py
  ```

The Ackermann test-only profile uses explicit synthetic values (`wheelbase=1.0 m`, steering bounds `±0.6 rad`, steering rate `10 rad/s`, minimum turning radius `2.0 m`, minimum speed `0.1 m/s`). It is compiled only into the unit test and is not installed as a Robot Profile. The fake-driver integration initially exposed that graph endpoint names are unavailable in this DDS setup and that startup zero samples can precede a test command; the test now asserts one publisher and waits for the nonzero translated sample. Final runs passed. No `ros2 launch`, real `/mux/cmd_vel`, CAN packet, chassis movement, or remote/E-stop test was run. The Bunker driver remains the sole physical CAN owner; `publish_odom_tf=false` and the existing Bunker remote/manual policy are unchanged.

- Python guard fail-closed regression after its default output moved to the canonical base topic — PASS. Under isolated `ROS_DOMAIN_ID=219`, LOST still stopped within 6.5 ms; steady lost output, stale replay, and source-switch replay all remained zero. The synthetic manual path also passed. This did not launch an adapter or hardware driver.

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  ROS_DOMAIN_ID=219 PYTHONPATH="bringup/agt_base_control:${PYTHONPATH}" python3 bringup/agt_base_control/test/guard_fail_closed_acceptance.py
  ```

## E. Atomic launch ownership

| ID | Gate | Evidence required | Result |
|---|---|---|---|
| E01 | Description starts independently | launch/dry-run/static test | PASS — atomic `--show-args` exit 0; source scan finds one `robot_state_publisher` owner |
| E02 | LiDAR starts independently | launch/dry-run; one driver owner | PASS — LiDAR atomic `--show-args` exit 0; hardware plan selects one LiDAR atomic; raw CustomMsg branch preserved |
| E03 | Base starts independently | launch/dry-run; one driver owner | PASS — Bunker base atomic `--show-args` exit 0; source pins `publish_odom_tf=false`; YHS remains blocked |
| E04 | LIO starts independently | exactly one selected backend | PASS — LIO atomic `--show-args` exit 0; selector branches to exactly one FAST-LIO2 or Batch-LIO include |
| E05 | Local perception starts independently | no hardware duplication | PASS — local perception atomic `--show-args` exit 0; one obstacle-cloud node owner |
| E06 | Global relocalization starts independently | no Nav2/hardware hidden startup | PASS — relocalization atomic `--show-args` exit 0; system localization composes it separately from Nav2/hardware |
| E07 | Localization manager starts independently | one `map->odom` authority | PASS — manager atomic `--show-args` exit 0; parsed production launch scan finds one manager node declaration |
| E08 | Motion guard starts independently | no vendor driver duplication | PASS — guard atomic `--show-args` exit 0; one selected C++ default or Python rollback launch |
| E09 | Nav2 starts independently | does not own physical hardware | PASS — Nav2 atomic `--show-args` exit 0; no physical hardware launch is included |
| E10 | RViz starts independently | does not own TF/navigation state | PASS — RViz atomic `--show-args` exit 0; UI node only |
| E11 | Aggregators include, not duplicate | static launch ownership test | PASS — 12 static owner tests plus profile launch-plan tests; aggregators contain no direct `Node` actions |
| E12 | Shutdown leaves no orphan owner | repeated start/stop test in isolated ROS domain | NOT_RUN |

P4 execution evidence (2026-10-02; isolated build and ROS domain; unit/mock + static/show-args only):

- The build covered 11 changed runtime/platform/description packages. Build/install/log outputs used `/tmp/agt_runtime_v4_p4/{build,install,log}`; the shared workspace `build/` and `install/` were not written.

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  colcon --log-base /tmp/agt_runtime_v4_p4/log build --base-paths /home/yangxuan/ros2_ws/src/agt_navigation_v3 /home/yangxuan/ros2_ws/src/agt_robot_platform /home/yangxuan/ros2_ws/src/agt_robot_description --packages-select agt_base_runtime agt_base_control agt_system_bringup agt_navigation_runtime agt_rviz_patrol agt_nav2_bringup agt_pointcloud_preprocessor agt_navigation_supervisor agt_navigation_capability agt_robot_bringup agt_robot_description --build-base /tmp/agt_runtime_v4_p4/build --install-base /tmp/agt_runtime_v4_p4/install
  ```

  Result: PASS, 11 packages finished. An earlier invocation put `--log-base` after `build` and was rejected by colcon; the corrected command above completed.

- Selected package tests ran with `ROS_DOMAIN_ID=231` and plugin autoload disabled for these non-launch graph tests:

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  source /tmp/agt_runtime_v4_p4/install/local_setup.bash
  export ROS_DOMAIN_ID=231
  export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  colcon --log-base /tmp/agt_runtime_v4_p4/test-log test --base-paths /home/yangxuan/ros2_ws/src/agt_navigation_v3 /home/yangxuan/ros2_ws/src/agt_robot_platform /home/yangxuan/ros2_ws/src/agt_robot_description --packages-select agt_base_runtime agt_base_control agt_system_bringup agt_navigation_runtime agt_rviz_patrol agt_nav2_bringup agt_pointcloud_preprocessor agt_navigation_supervisor agt_navigation_capability agt_robot_bringup agt_robot_description --build-base /tmp/agt_runtime_v4_p4/build
  colcon --log-base /tmp/agt_runtime_v4_p4/test-log test-result --all --verbose --test-result-base /tmp/agt_runtime_v4_p4/build
  ```

  Result: PASS, zero errors/failures/skips. The result summary contains 100 entries including CTest aggregate XML and leaf reports; leaf suites report 90 cases: base runtime 18, pointcloud preprocessing 4, platform launch/config 36, robot description 20, and static launch ownership 12.

- Additional launch-policy tests:

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  source /tmp/agt_runtime_v4_p4/install/local_setup.bash
  export ROS_DOMAIN_ID=231
  export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  python3 -m pytest -q /home/yangxuan/ros2_ws/src/agt_navigation_v3/bringup/agt_system_bringup/test/test_payload_interlock_launch.py /home/yangxuan/ros2_ws/src/agt_navigation_v3/bringup/agt_system_bringup/test/test_payload_interlock_resolution.py /home/yangxuan/ros2_ws/src/agt_navigation_v3/bringup/agt_system_bringup/test/test_yhs_nav_config_guard.py
  ```

  Result: PASS, 14 passed.

- Independently executed `ros2 launch <package> <entry>.launch.py --show-args` with the same sourced ROS/workspace/P4 overlay and `ROS_DOMAIN_ID=231`; all returned exit code 0. Entries:

  ```text
  agt_robot_description description.launch.py
  agt_robot_bringup lidar.launch.py, base.launch.py, camera.launch.py, rtk.launch.py
  agt_navigation_runtime lio.launch.py
  agt_pointcloud_preprocessor local_perception.launch.py
  agt_global_relocalization global_relocalization.launch.py
  agt_localization_manager localization_manager.launch.py
  agt_base_runtime motion_guard.launch.py, base_adapter.launch.py
  agt_nav2_bringup nav2.launch.py
  agt_rviz_patrol rviz.launch.py
  agt_system_bringup localization.launch.py, navigation.launch.py, system.launch.py
  ```

  Compatibility entries also returned 0: `agt_system_bringup hardware.launch.py`, platform `robot_hardware.launch.py`/`robot_bringup.launch.py`/`sensors.launch.py`, `agt_nav2_bringup navigation.launch.py`, `agt_robot_description display.launch.py`/`rviz.launch.py`, `agt_pointcloud_preprocessor pointcloud_preprocessor.launch.py`, and Python rollback `agt_base_control cmd_vel_guard.launch.py`. These were show-args queries; no process or hardware driver was started.

- A direct source `pytest` attempt before sourcing the installed workspace failed collection because the package index lacked `agt_system_bringup`; after sourcing the build underlay and running the isolated CTest/targeted pytest commands above, all applicable tests passed.

E12 remains `NOT_RUN`: no launch graph restart/shutdown test was executed. No rosbag replay, real hardware, CAN command, chassis motion, or arm action was run. `J08` compatibility launch availability is `PASS` by isolated build and compatibility `--show-args` results.

P4 commits: `agt_robot_description` `a0e91268aed0f39cab6e1959b014f74fade2b750`; `agt_robot_platform` `079f8663629f3a11bf5464a68bffc406ff9264da`; one `agt_navigation_v3` commit recorded as the P4 phase commit in the branch history.

## F. TF and odometry invariants

| ID | Gate | Evidence required | Result |
|---|---|---|---|
| F01 | sole `map -> odom` owner | graph/static audit | NOT_RUN |
| F02 | sole navigation `odom -> base_footprint` owner | selected LIO adapter only | NOT_RUN |
| F03 | chassis `publish_odom_tf=false` | resolved profile/launch/driver config | NOT_RUN |
| F04 | no extra `odom -> base_link` publisher | TF graph evidence | NOT_RUN |
| F05 | wheel odom not silently promoted to localization authority | source/config audit | NOT_RUN |

## G. Sensor layer

| ID | Gate | Evidence required | Result |
|---|---|---|---|
| G01 | Livox raw timing path preserved | source/topic contract audit | NOT_RUN |
| G02 | no generic pre-LIO filtering introduced | launch/data-path inspection | NOT_RUN |
| G03 | secondary perception cloud remains available | topic/replay evidence | NOT_RUN |
| G04 | each physical sensor has one launch owner | static/runtime graph evidence | NOT_RUN |
| G05 | sensor-only stack runs without Nav2 | rosbag/mock or hardware-safe test | NOT_RUN |

## H. Localization Manager migration decision

| ID | Gate | Evidence required | Result |
|---|---|---|---|
| H01 | migration feasibility audit complete | rates, APIs, TF timing, state logic documented | PASS — `docs/upgrade/localization_manager_cpp_feasibility.md` documents source/config rates, interfaces, state machine, timing semantics, test coverage and decision. |
| H02 | correction math frozen by tests | `T_map_odom = T_map_base * inverse(T_odom_base)` tests | PASS — `test_correction_math.py`: 13 passed, including compose/inverse and the exact correction formula. |
| H03 | old/new parity if migrated | bag/shadow numeric comparison without dual `/tf` publication | NOT_RUN — P5 chose `KEEP_PYTHON_THIS_RELEASE`; no C++ candidate exists, so no parity result is claimed. |
| H04 | one authority after switch | runtime graph evidence | NOT_RUN — no authority switch or runtime graph inspection was performed. The source-level owner test passed; it does not establish the live graph. |
| H05 | if kept Python, reason documented | no false “all C++” claim | PASS — decision and measured-evidence gap are documented; Python remains the only production `map -> odom` publisher. |

P5 feasibility/build/test evidence (2026-10-02; unit/mock and static-source only):

- Decision: `KEEP_PYTHON_THIS_RELEASE`, documented in
  `docs/upgrade/localization_manager_cpp_feasibility.md`. The historic
  `NAV_TEST_002` bag audit is read-only archived observation, not a P5 replay,
  CPU benchmark, deadline measurement, or parity result.
- `agt_localization_manager` build — PASS, 1 package. Build/install/log outputs
  are isolated under `/tmp/agt_runtime_v4_p5`; the existing workspace install
  was only sourced as an underlay:

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  colcon --log-base /tmp/agt_runtime_v4_p5/log build --base-paths /home/yangxuan/ros2_ws/src/agt_navigation_v3 --packages-select agt_localization_manager --build-base /tmp/agt_runtime_v4_p5/build --install-base /tmp/agt_runtime_v4_p5/install
  ```

- Correction-math unit tests — PASS, 13 passed. No ROS process or node was
  started; a separate domain ID was set for isolation:

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  source /tmp/agt_runtime_v4_p5/install/local_setup.bash
  export ROS_DOMAIN_ID=231
  export PYTHONPATH=/home/yangxuan/ros2_ws/src/agt_navigation_v3/navigation/localization/agt_localization_manager${PYTHONPATH:+:$PYTHONPATH}
  export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  python3 -m pytest -q /home/yangxuan/ros2_ws/src/agt_navigation_v3/navigation/localization/agt_localization_manager/test/test_correction_math.py
  ```

- Static launch-owner test — PASS, 12 passed. This parses source only; it does
  not start a graph or prove live `/tf` ownership:

  ```bash
  source /opt/ros/humble/setup.bash
  source /home/yangxuan/ros2_ws/install/setup.bash
  source /tmp/agt_runtime_v4_p4/install/local_setup.bash
  export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  python3 -m pytest -q /home/yangxuan/ros2_ws/src/agt_navigation_v3/bringup/agt_system_bringup/test/test_launch_ownership_static.py
  ```

- Rosbag replay/shadow comparison and live ROS graph inspection — NOT_RUN.
- Real hardware — NOT_RUN; no launch, CAN, base command, camera, or arm action.

## I. Nav2 cross-platform contract

| ID | Gate | Evidence required | Result |
|---|---|---|---|
| I01 | tracked profile preserves Bunker assumptions | config diff/test | NOT_RUN |
| I02 | Ackermann profile disables spin assumptions | controller/recovery config audit | NOT_RUN |
| I03 | Ackermann turning radius represented | verified field required before field enable | NOT_RUN |
| I04 | velocity/smoother limits selected by profile | config resolution test | NOT_RUN |
| I05 | YHS navigation profile not marked verified without hardware facts | validator/test | BLOCKED |

## J. Integration tests without physical motion

| ID | Gate | Evidence required | Result |
|---|---|---|---|
| J01 | full Bunker software stack starts in isolated/mock/replay mode | launch log + graph | NOT_RUN |
| J02 | MID360 rosbag -> selected LIO remains functional | replay result, topic rates/latency | NOT_RUN |
| J03 | localization stack consumes canonical LIO output | replay/mock evidence | NOT_RUN |
| J04 | Nav2 receives expected TF/odom/map contracts | graph/check scripts | NOT_RUN |
| J05 | stale-command fault injection stops command output | automated log/test | NOT_RUN |
| J06 | localization-health loss blocks command output | automated log/test | NOT_RUN |
| J07 | atomic restart works | each layer restarted without duplicate owner/orphan | NOT_RUN |
| J08 | legacy compatibility launch still available | dry-run/build evidence | PASS — compatibility launch files build and all requested wrapper `--show-args` checks return 0 |

## K. Real hardware acceptance

These gates cannot be satisfied by unit tests, mocks or rosbag replay.

| ID | Gate | Evidence required | Result |
|---|---|---|---|
| K01 | Bunker physical remote/estop priority | supervised real chassis test | NOT_RUN |
| K02 | Bunker low-speed forward/reverse/turn | supervised real chassis test | NOT_RUN |
| K03 | command timeout physically stops chassis | supervised fault test | NOT_RUN |
| K04 | real MID360 + LIO + TF under new split startup | live sensor evidence | NOT_RUN |
| K05 | real autonomous Nav2 motion after all prechecks | supervised field acceptance | NOT_RUN |
| K06 | real Ackermann chassis | only when actual vehicle exists and parameters verified | NOT_RUN |
| K07 | real YHS/steering chassis | only after protocol/kinematics audit | BLOCKED |

## L. Performance/regression

| ID | Gate | Evidence required | Result |
|---|---|---|---|
| L01 | command loop rate | median/P95 period and missed deadlines | NOT_RUN |
| L02 | CPU/RSS guard runtime | before/after comparison | NOT_RUN |
| L03 | no unbounded process growth during software soak | >=30 min initial soak; longer field soak later | NOT_RUN |
| L04 | topic latency not materially regressed | selected critical topics compared | NOT_RUN |
| L05 | no new duplicate publishers | graph audit after integration | NOT_RUN |

## M. Required report format

`docs/upgrade/platform_runtime_cpp_test_report.md` must include:

1. branch, commit, workspace path and ROS environment;
2. dirty-tree baseline and how unrelated changes were protected;
3. packages built and exact commands;
4. tests run, counts and failures;
5. per-gate status for this matrix;
6. Bunker parity evidence;
7. Ackermann software-only evidence;
8. YHS verified facts and unresolved facts;
9. startup ownership graph before/after;
10. TF publisher evidence;
11. real hardware tests explicitly separated from mock/replay tests;
12. performance/soak observations;
13. rollback procedure;
14. remaining `FAIL`, `NOT_RUN`, `BLOCKED` items.

## Release rule

Do not call this refactor field-ready while any safety-critical Bunker gates in C/D/F/J are failing. `NOT_RUN` real-hardware gates may remain during software development but must be stated prominently. Ackermann/YHS support must be described as software-ready, blocked, or field-verified according to actual evidence; never infer physical readiness from compilation or unit tests.
