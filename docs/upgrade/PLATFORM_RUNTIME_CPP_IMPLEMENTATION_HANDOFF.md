# Platform Runtime C++ Implementation Handoff

Target branch: `refactor/navigation-runtime-v4`

Purpose: hand this document to Codex on the Ubuntu 22.04 / ROS 2 Humble development machine. The task is to audit and incrementally implement a platform-independent navigation runtime for Bunker, generic Ackermann and future YHS/steering bases, while splitting startup into atomic sensor/runtime launches.

This is **not** permission to perform a broad rewrite. Preserve working field behavior, keep a rollback path, and mark hardware-dependent items `NOT_RUN` when no real device evidence exists.

Primary design contract: `docs/architecture/ROBOT_PLATFORM_RUNTIME_CONTRACT.md`.
Acceptance gates: `docs/acceptance/PLATFORM_RUNTIME_CPP_ACCEPTANCE.md`.
Existing V4 invariants: `AGENTS.md`, `docs/architecture/NAVIGATION_RUNTIME_V4.md`, `docs/upgrade/WORKSPACE_CONSOLIDATION_PLAN.md`.

---

## 0. Non-negotiable execution rules

Before editing:

```bash
cd /home/yangxuan/ros2_ws/src/agt_navigation_v3
git status --short
git branch --show-current
git log -5 --oneline
```

Rules:

- do not `git reset --hard`;
- do not `git clean -fd`;
- do not discard user modifications;
- do not merge to `main`;
- do not change localization mathematics, Nav2 tuning, calibrated TF values or Bunker CAN semantics merely to make tests pass;
- do not invent YHS/Ackermann dimensions, steering limits, drive gear, protocol units or safety semantics;
- do not start a real chassis, arm or autonomous mission without explicit operator authorization;
- use isolated build/install directories if the normal workspace contains unrelated uncommitted work;
- every migration step must preserve an old entry point until parity is demonstrated.

If the working tree is dirty, record the baseline and avoid touching unrelated files. Prefer a worktree/isolated overlay instead of stashing unknown user work.

---

# Phase P0 — Source ownership and migration audit

Goal: understand the actual runtime before writing C++.

## P0.1 Inventory runtime packages and languages

Generate an inventory of all ROS packages and executable nodes in this repository. Search at least:

```bash
find . -name package.xml -print
find . -type f \( -name '*.py' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.launch.py' \) -print
rg -n "console_scripts|add_executable|rclcpp::Node|class .*\(Node\)|Node\(" .
```

Create:

```text
docs/upgrade/platform_runtime_cpp_audit.md
```

with one row per production-relevant node:

| package | executable/node | language | frequency/criticality | owns authoritative state? | current inputs | current outputs | classification | reason |
|---|---|---|---|---|---|---|---|---|

Classification must be exactly one of:

```text
KEEP_PYTHON
MIGRATE_CPP
TOOL_ONLY
REMOVE_AFTER_PARITY
UNKNOWN_NEEDS_AUDIT
```

At minimum explicitly audit:

- `bringup/agt_base_control/agt_base_control/cmd_vel_guard.py`;
- `navigation/localization/agt_localization_manager/...`;
- `runtime/agt_navigation_supervisor/...`;
- `capability/agt_navigation_capability/...`;
- point-cloud preprocessing runtime;
- LIO adapters;
- global relocalization Python/native boundary;
- mission/runtime/UI tooling that should remain out of the real-time core.

Do not assume Python is wrong. The audit must justify every proposed migration.

## P0.2 Runtime ownership graph

Produce a machine-readable or Markdown graph showing unique owners of:

```text
robot_state_publisher
MID360 driver
base driver
camera driver
RTK driver
odom -> base_footprint
map -> odom
/agt/odometry/local
Nav2
motion command guard
vendor command output
```

Search all launch files and scripts for duplicate startup paths. Explicitly detect whether an aggregator directly instantiates a Node that is also instantiated by an atomic launch.

## P0.3 Topic/TF baseline

Record the expected current Bunker contract before changing it:

```text
Nav2 output -> smoother/guard -> /mux/cmd_vel -> Bunker driver
Bunker driver -> /wheel/odom
LIO -> /agt/odometry/local + odom -> base_footprint
Localization Manager -> map -> odom
robot_state_publisher -> static vehicle/sensor TF
```

Create a baseline table of topic type, publisher owner and expected publisher count.

### P0 exit gate

Do not begin implementation until:

- audit document exists;
- duplicate ownership risks are listed;
- all candidate Python-to-C++ migrations have an explicit reason;
- Bunker current command/odom/TF path is documented.

Commit suggested:

```text
docs(runtime): audit platform ownership and cpp migration candidates
```

---

# Phase P1 — Robot Profile and kinematic contract

Goal: remove vendor-name logic from navigation/runtime decisions before adding additional bases.

## P1.1 Audit existing profiles

Inspect:

```text
config/robot_profiles/
agt_robot_description
agt_robot_bringup
run_field_stack.sh
navigation.launch.py
```

Do not create a second parallel profile system if one already exists. Extend the existing profile/resolver contract.

## P1.2 Add schema/validator

Required conceptual fields are defined in `ROBOT_PLATFORM_RUNTIME_CONTRACT.md`.

Validation rules:

- `bunker_v1`: `kinematics=skid_steer`, `publish_odom_tf=false`, 50 Hz baseline, existing verified limits preserved;
- generic Ackermann profile/template: must require wheelbase, steering bounds/rate, minimum turning radius and `rotate_in_place=false` before field enablement;
- YHS: do **not** assign Ackermann/skid/swerve type until actual chassis protocol and geometry prove it;
- missing safety-critical physical values fail validation rather than receive guessed defaults.

Create tests for valid/invalid profiles.

## P1.3 Separate capability from vendor

No production controller, guard or supervisor code should use vendor-specific branches for kinematic behavior such as:

```text
if robot == bunker
if robot == yhs
```

Vendor selection may remain in bringup/driver resolution. Motion behavior must use `kinematics` and verified capabilities.

### P1 exit gate

- Bunker profile resolves identically to current validated behavior;
- invalid Ackermann profile fails closed;
- YHS remains explicitly blocked if its kinematics are unresolved;
- tests pass without hardware.

Commit suggested:

```text
feat(platform): define validated robot kinematic profiles
```

---

# Phase P2 — C++ Motion Guard parity migration

Goal: replace the Python command guard in the high-rate command path while preserving Bunker behavior.

## P2.1 Freeze current guard semantics

Before rewriting, inspect current `cmd_vel_guard.py` and its tests. Build a parity matrix covering at least:

- linear/angular clamp;
- positive/negative acceleration and slew limits;
- fixed output refresh;
- stale input timeout -> zero;
- startup with no valid command;
- localization interlock;
- payload/mission interlock currently added on V4;
- health loss;
- recovery from a blocked state;
- command timestamp behavior;
- zero-command behavior;
- shutdown behavior.

Add missing tests against the **old Python implementation first** where practical. This freezes intended semantics before C++ is introduced.

## P2.2 Implement `agt_base_runtime` / C++ guard

Prefer a new `ament_cmake` package rather than mixing runtime C++ into an unrelated Python package.

The C++ guard must:

- use `rclcpp`;
- publish at the configured 50 Hz baseline;
- use monotonic/ROS time consistently with current contract;
- fail closed on stale/invalid health;
- expose diagnostics sufficient to distinguish `READY`, `ACTIVE`, `STALE_COMMAND`, localization block and health block;
- not talk directly to CAN;
- not bypass vendor/manual safety arbitration.

## P2.3 Shadow parity mode

Before making C++ authoritative, allow the Python and C++ guards to process the same recorded/synthetic input under isolated output topics, e.g.:

```text
/mux/cmd_vel_python_shadow
/mux/cmd_vel_cpp_shadow
```

Never let both drive the physical vendor command topic simultaneously.

Compare sample-by-sample output and state transitions. Record allowed tolerance and every intentional difference.

## P2.4 Authority switch

Only after parity tests pass, switch the default Bunker command owner to C++. Keep the Python entry as rollback/legacy until an acceptance commit explicitly retires it.

### P2 exit gate

- unit tests cover old and new semantics;
- shadow parity result generated;
- exactly one authoritative publisher to the vendor command path;
- Bunker output rate remains 50 Hz target;
- stale/localization/health failures output zero and fail closed;
- no real motion required for software gate.

Commit suggestions:

```text
feat(platform): add cpp motion guard

test(platform): prove python cpp guard parity
```

---

# Phase P3 — Base Runtime adapter layer

Goal: navigation talks to one base runtime; vendor drivers remain behind adapters.

## P3.1 Define adapter API

Implement the interface from `ROBOT_PLATFORM_RUNTIME_CONTRACT.md` or a technically equivalent interface. Keep vendor packet types out of the common API.

The runtime must expose canonical command/feedback interfaces while preserving legacy topics through explicit bridge/remap during migration.

## P3.2 Bunker adapter first

Implement and validate Bunker before adding any new chassis.

Required properties:

- canonical guarded Twist maps to the existing Bunker `/mux/cmd_vel` path;
- current CAN driver remains the vendor owner;
- `publish_odom_tf=false` preserved;
- `/wheel/odom` remains available;
- adapter must not create a second `odom -> base_*` TF;
- remote/manual priority remains outside and above autonomous software.

Use bag/fake-driver tests to verify command and odom normalization. Do not send real chassis commands as part of automated test.

## P3.3 Generic Ackermann adapter

Implement a software-only Ackermann adapter and tests using a test profile with explicit synthetic dimensions. Do not present those synthetic values as a real robot profile.

Test at least:

- straight driving;
- left/right turn;
- reverse;
- steering saturation;
- steering-rate limit;
- near-zero speed handling;
- requested in-place rotation rejected/converted according to contract;
- curvature continuity around low speed;
- invalid wheelbase/limits rejected.

The adapter may output `ackermann_msgs/AckermannDriveStamped` if that matches the selected driver contract, but do not add an unnecessary dependency if no actual Ackermann driver is yet integrated. A pure conversion library plus tests is acceptable for this phase.

## P3.4 YHS / steering placeholder

Audit the actual YHS source/protocol available in the local workspace. Produce:

```text
docs/upgrade/YHS_BASE_PROTOCOL_AUDIT.md
```

with:

- actual model;
- command transport (CAN/serial/UDP/etc.);
- command message/fields;
- steering architecture;
- odometry source;
- units/sign conventions;
- drive gear semantics;
- timeout/brake/manual priority;
- known wheel geometry;
- unresolved items.

Only implement a steering/YHS adapter after this evidence exists. Otherwise create a blocked stub/profile validation state, not fake runtime behavior.

### P3 exit gate

- Bunker adapter parity PASS;
- Ackermann conversion unit tests PASS with synthetic test data;
- YHS classification is evidence-based or remains BLOCKED;
- common runtime contains no vendor packet types above adapter boundary.

---

# Phase P4 — Atomic startup refactor

Goal: sensors and runtime components can be launched independently; aggregate launches only compose them.

## P4.1 Inventory current launch ownership

Audit all current launch files under:

```text
bringup/
navigation/
sensor/
```

and external `agt_robot_platform/agt_robot_bringup` if available in the workspace.

Record where each physical driver and each authoritative runtime node is instantiated.

## P4.2 Create/normalize atomic launches

Provide independently runnable entries for the applicable components:

```text
description.launch.py
lidar.launch.py
camera.launch.py
rtk.launch.py
base.launch.py
lio.launch.py
local_perception.launch.py
global_relocalization.launch.py
localization_manager.launch.py
motion_guard.launch.py
nav2.launch.py
rviz.launch.py
```

Do not force all files into one package if existing ownership says otherwise. Prefer each owning package to expose its own atomic launch; system bringup should compose them.

Requirements:

- atomic launch starts only its declared responsibility;
- no hidden duplicate `robot_state_publisher`;
- no hidden duplicate LiDAR/base driver;
- `lio.launch.py` selects exactly one backend;
- localization launch does not start Nav2;
- Nav2 launch does not start physical hardware;
- RViz launch does not own TF/navigation state.

## P4.3 Aggregators

Normalize convenience launches:

```text
sensors.launch.py
localization.launch.py
navigation.launch.py
system.launch.py
```

Aggregators should include atomic launches and pass profiles/arguments. Do not duplicate Node declarations already owned elsewhere.

Maintain compatibility wrappers such as current `hardware.launch.py` while migration is under test.

## P4.4 Static owner tests

Add tests that parse/search launch sources and fail when known singleton nodes have more than one production owner.

At minimum guard:

```text
robot_state_publisher
selected LIO adapter
localization_manager
motion_guard
Nav2 bringup
base driver
MID360 driver
```

### P4 exit gate

Run dry-run/show-args/static tests proving every layer is independently invocable and the aggregate stack contains one owner of each singleton.

---

# Phase P5 — Localization Manager C++ feasibility and staged migration

Goal: decide with evidence whether the authoritative localization state machine should migrate to C++ now.

This phase is **not automatically mandatory** just because the desired runtime is C++-heavy.

## P5.1 Audit first

Document:

- message rates;
- correction math;
- state transitions;
- TF publication timing;
- service/action API;
- dependency on Python-only logic;
- existing test coverage.

Classify:

```text
MIGRATE_CPP_NOW
KEEP_PYTHON_THIS_RELEASE
```

with justification.

## P5.2 If migrating

Freeze correction math tests before implementation. C++ must preserve:

```text
T_map_odom = T_map_base * inverse(T_odom_base)
```

and the sole-authority rule for `map -> odom`.

Run old/new shadow replay from a rosbag without publishing both to `/tf`. Compare state transitions and transforms numerically on isolated topics/logs.

Do not combine this phase with new row-localization mathematics. First achieve parity; new localization states belong in a subsequent feature phase.

### P5 exit gate

Either:

- C++ parity is proven and authority switches safely; or
- a written decision keeps Python with no false claim that the repository is fully C++.

---

# Phase P6 — Sensor interface cleanup without breaking LIO

Goal: establish explicit sensor-layer ownership while preserving the validated MID360 raw path.

Rules:

- Livox `CustomMsg` timing path to FAST-LIO2/Batch-LIO must remain untouched unless separately validated;
- `/agt/livox/points` or successor PointCloud2 is the secondary perception/relocalization branch;
- do not put generic height/voxel/self filters in front of LIO;
- sensor launch split must not start a second driver.

Document vendor input and canonical downstream output for LiDAR, IMU, RTK and cameras. Introduce remaps/adapters only where they clarify ownership and do not add unnecessary copies/latency.

### P6 exit gate

- LIO raw topic/type/timing unchanged or parity-proven;
- local perception/relocalization receive their intended secondary cloud;
- each sensor driver has one owner;
- sensor layer can run without Nav2.

---

# Phase P7 — Cross-platform Nav2 configuration contract

Goal: make the runtime actually compatible with different kinematics rather than only changing drivers.

Create separate navigation profiles, reusing shared parameters where valid:

```text
tracked/skid
ackermann
steering_or_swerve (only after verified)
```

Audit and explicitly configure:

- controller assumptions;
- `rotate_to_heading` behavior;
- spin recovery;
- minimum turning radius;
- reversing;
- velocity/acceleration limits;
- footprint source;
- progress checker;
- smoother limits.

Do not copy Bunker RPP values into Ackermann and call them verified. Software schema/tests may exist before real tuning, but real parameters remain `NOT_RUN` until measured.

### P7 exit gate

- tracked profile reproduces existing Bunker software behavior;
- Ackermann profile is internally consistent and rejects impossible in-place rotation assumptions;
- no unverified YHS field profile is marked ready.

---

# Phase P8 — Integration and rollback

Goal: prove the refactor did not destroy the known V4 path.

Required software integration scenarios:

1. Bunker profile + fake/base replay + MID360 bag + selected LIO + localization + Nav2 startup, no physical motion.
2. Sensor-only startup.
3. Base-only fake startup.
4. LIO-only rosbag replay.
5. Localization-only using recorded canonical odometry/cloud.
6. Nav2-only with mocked required TF/odom.
7. C++ guard stale-command and health-loss injection.
8. Ackermann adapter conversion tests.
9. Duplicate publisher/TF ownership audit.
10. shutdown/restart each atomic layer without orphan processes.

Rollback proof:

- document previous branch/SHA;
- preserve compatibility launch until final acceptance;
- show how to select legacy Python guard if C++ guard causes a regression during field validation.

---

# Required generated artifacts

Codex must leave the following evidence in the repository or an experiment directory and link it from the final report:

```text
docs/upgrade/platform_runtime_cpp_audit.md
docs/upgrade/YHS_BASE_PROTOCOL_AUDIT.md            # if YHS sources are available
docs/upgrade/platform_runtime_cpp_test_report.md
```

Test report must separate:

```text
PASS
FAIL
NOT_RUN
BLOCKED
```

and distinguish software/mock/rosbag evidence from real chassis evidence.

---

# Recommended commit sequence

Keep commits small enough to review and revert:

```text
docs(runtime): audit platform ownership and cpp migration candidates
feat(platform): define validated robot kinematic profiles
feat(platform): add cpp motion guard
test(platform): prove python cpp guard parity
feat(platform): add common base adapter runtime
feat(platform): add bunker adapter parity path
feat(platform): add ackermann conversion library
refactor(bringup): expose atomic runtime launches
test(bringup): enforce singleton startup ownership
docs(platform): audit yhs base protocol
refactor(localization): migrate manager to cpp   # only if P5 decides yes
```

Do not squash all phases into one commit before local debugging.

---

# Final Codex response format

Return:

```text
STATUS: PASS / PARTIAL / FAIL
BRANCH:
HEAD:

P0 audit: PASS/FAIL
P1 profiles: PASS/FAIL
P2 C++ guard: PASS/FAIL/NOT_RUN
P3 adapters: PASS/FAIL/PARTIAL
P4 atomic launch: PASS/FAIL
P5 localization manager: MIGRATED / KEPT_PYTHON + reason
P6 sensor layer: PASS/FAIL
P7 Nav profiles: PASS/FAIL/PARTIAL
P8 integration: PASS/FAIL/NOT_RUN

Key changed files:
...

Tests/builds actually executed:
...

Bunker compatibility evidence:
...

Ackermann software evidence:
...

YHS status:
VERIFIED KINEMATICS / BLOCKED + missing facts

Real hardware tests:
PASS / NOT_RUN (list exactly what was physically tested)

Known regressions / unresolved items:
...

Rollback:
...
```

The task is complete only when the acceptance matrix contains evidence, not when source files merely compile.
