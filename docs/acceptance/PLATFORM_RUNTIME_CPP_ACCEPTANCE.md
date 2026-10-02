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
| B01 | `bunker_v1` validates | existing verified Bunker fields preserved | NOT_RUN |
| B02 | Bunker is classified skid-steer | explicit kinematic type, not inferred by vendor branch at runtime | NOT_RUN |
| B03 | Ackermann required fields enforced | invalid/missing wheelbase or steering limits rejected | NOT_RUN |
| B04 | Ackermann rotate-in-place disabled | schema and navigation profile reject impossible spin assumption | NOT_RUN |
| B05 | YHS classification evidence-based | protocol/geometry audit or explicit BLOCKED state | BLOCKED |
| B06 | No guessed safety-critical defaults | tests demonstrate missing physical values fail closed | NOT_RUN |

## C. C++ Motion Guard

| ID | Gate | Evidence required | Result |
|---|---|---|---|
| C01 | C++ package builds | clean/isolated colcon build log | NOT_RUN |
| C02 | velocity clamp parity | old/new test or shadow comparison | NOT_RUN |
| C03 | slew-limit parity | positive/negative acceleration cases | NOT_RUN |
| C04 | output refresh | measured ~50 Hz software target under test load | NOT_RUN |
| C05 | stale command -> zero | deterministic automated test | NOT_RUN |
| C06 | localization block -> zero | deterministic automated test | NOT_RUN |
| C07 | health/payload block -> zero | deterministic automated test | NOT_RUN |
| C08 | recovery state transition | blocked -> ready/active only after valid inputs | NOT_RUN |
| C09 | exactly one vendor command publisher | ROS graph or static owner test | NOT_RUN |
| C10 | Python rollback path documented | explicit command/config for rollback during field test | NOT_RUN |

## D. Base Runtime adapters

| ID | Gate | Evidence required | Result |
|---|---|---|---|
| D01 | Common adapter API contains no vendor packet types | code review/static inspection | NOT_RUN |
| D02 | Bunker command parity | fake/replay test maps canonical Twist to current `/mux/cmd_vel` behavior | NOT_RUN |
| D03 | Bunker odom preserved | `/wheel/odom` remains available and no competing odom TF appears | NOT_RUN |
| D04 | Remote/manual priority untouched | architecture/code evidence; real validation separately | NOT_RUN |
| D05 | Ackermann straight/turn/reverse tests | unit tests with synthetic verified-in-test geometry | NOT_RUN |
| D06 | Ackermann saturation/rate tests | steering angle and rate boundaries | NOT_RUN |
| D07 | Ackermann low-speed singularity handled | zero/near-zero speed test | NOT_RUN |
| D08 | Ackermann in-place request handled safely | no infinite/invalid steering output | NOT_RUN |
| D09 | YHS adapter only if protocol verified | otherwise explicit BLOCKED | BLOCKED |

## E. Atomic launch ownership

| ID | Gate | Evidence required | Result |
|---|---|---|---|
| E01 | Description starts independently | launch/dry-run/static test | NOT_RUN |
| E02 | LiDAR starts independently | launch/dry-run; one driver owner | NOT_RUN |
| E03 | Base starts independently | launch/dry-run; one driver owner | NOT_RUN |
| E04 | LIO starts independently | exactly one selected backend | NOT_RUN |
| E05 | Local perception starts independently | no hardware duplication | NOT_RUN |
| E06 | Global relocalization starts independently | no Nav2/hardware hidden startup | NOT_RUN |
| E07 | Localization manager starts independently | one `map->odom` authority | NOT_RUN |
| E08 | Motion guard starts independently | no vendor driver duplication | NOT_RUN |
| E09 | Nav2 starts independently | does not own physical hardware | NOT_RUN |
| E10 | RViz starts independently | does not own TF/navigation state | NOT_RUN |
| E11 | Aggregators include, not duplicate | static launch ownership test | NOT_RUN |
| E12 | Shutdown leaves no orphan owner | repeated start/stop test in isolated ROS domain | NOT_RUN |

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
| H01 | migration feasibility audit complete | rates, APIs, TF timing, state logic documented | NOT_RUN |
| H02 | correction math frozen by tests | `T_map_odom = T_map_base * inverse(T_odom_base)` tests | NOT_RUN |
| H03 | old/new parity if migrated | bag/shadow numeric comparison without dual `/tf` publication | NOT_RUN |
| H04 | one authority after switch | runtime graph evidence | NOT_RUN |
| H05 | if kept Python, reason documented | no false “all C++” claim | NOT_RUN |

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
| J08 | legacy compatibility launch still available | dry-run/build evidence | NOT_RUN |

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
