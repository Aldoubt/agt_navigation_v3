# Robot Platform Runtime Contract

Status: design contract for `refactor/navigation-runtime-v4`. This document does **not** claim field acceptance. It defines the target boundary for tracked, Ackermann and steering/swerve-capable bases while preserving the current localization and TF invariants.

## 1. Design goal

Navigation algorithms must not depend directly on a vendor chassis protocol. Platform differences are absorbed by **Robot Profile + Base Runtime Adapter**. Sensor drivers are likewise separated from downstream navigation interfaces.

The target split is:

```text
Mission / operator
        ↓
      Nav2
        ↓ motion intent
  Motion Guard (C++)
        ↓ canonical base command
  Base Runtime (C++)
   ┌────┼───────────────┐
   │    │               │
tracked ackermann   steering/swerve
adapter adapter       adapter
   │    │               │
 vendor chassis drivers / SDK

MID360 / IMU / ToF / RTK / camera
        ↓ vendor interfaces
    Sensor adapters
        ↓ canonical sensor interfaces
LIO / local perception / relocalization / recording
```

The runtime/data plane should prefer C++ where timing, arbitration or authoritative state is involved. Launch orchestration, profile selection and experiment tooling remain Python/YAML unless there is a demonstrated reason to migrate them.

## 2. Frozen invariants

The following existing V4 rules remain authoritative:

1. `agt_localization_manager` is the sole owner of `map -> odom` until an explicitly accepted replacement preserves the same ownership contract.
2. Exactly one navigation LIO backend owns canonical local odometry and navigation `odom -> base_footprint`.
3. Chassis drivers must not publish a competing odom TF.
4. Vendor raw LiDAR timing is preserved for LIO. Local-obstacle filtering is a separate branch.
5. Autonomous motion must remain fail-closed when command freshness, platform readiness or navigation health is invalid.
6. Physical remote / emergency-stop priority must never be bypassed by software.
7. Existing Bunker behavior is the compatibility baseline; Ackermann/YHS support must not be obtained by weakening Bunker safety gates.
8. Unknown physical dimensions, steering geometry, protocol units, drive gear or hardware limits are `NOT_VERIFIED`; they must not be invented in source or profile defaults.

## 3. Canonical platform interfaces

### 3.1 Motion intent

Nav2-facing motion intent remains representable as `geometry_msgs/msg/Twist`. The canonical input to the platform runtime is:

```text
/agt/motion/request   geometry_msgs/Twist
```

This topic expresses desired planar motion, not necessarily the vendor wire command. A compatibility remap from the current Nav2 output is allowed during migration.

### 3.2 Guarded base command

Only the motion guard may forward an autonomous command into the platform adapter:

```text
/agt/base/cmd_vel     geometry_msgs/Twist
```

For a skid-steer/differential platform this can map directly to a Twist-consuming vendor driver. For Ackermann and steering/swerve bases this is an intermediate canonical command that must be converted by the adapter.

### 3.3 Canonical base feedback

All chassis adapters must normalize their feedback to:

```text
/agt/base/odom        nav_msgs/Odometry
/agt/base/state       platform state message or diagnostic contract
/agt/base/health      diagnostic/health contract
```

During staged migration `/wheel/odom` may remain as the legacy vendor/diagnostic source. The canonical `/agt/base/odom` must not silently become the localization authority; local navigation odometry remains `/agt/odometry/local` unless a separately reviewed fusion design changes that contract.

## 4. Kinematic types

The runtime must use an explicit kinematic type instead of vendor-name conditionals:

```cpp
enum class KinematicType {
  kSkidSteer,
  kDifferential,
  kAckermann,
  kFourWheelSteer,
  kOmni,
  kSwerve,
};
```

A vendor such as Bunker or YHS is a Robot Profile / driver selection, not the kinematic definition itself.

### 4.1 Skid steer / differential

Canonical command:

```text
(v_x, omega_z)
```

In-place rotation may be supported when the physical platform and profile permit it.

### 4.2 Ackermann

The adapter converts planar Twist intent into speed plus steering angle, using verified wheelbase and limits. A nominal relation is:

```text
delta = atan(L * omega / v)
```

but implementation must explicitly handle near-zero speed, reverse motion, steering saturation and rate limits. `rotate_in_place` must be false. Unknown `L`, steering limits or protocol units block field enablement.

### 4.3 Four-wheel-steer / swerve

Do not assume these are Ackermann. The adapter must be implemented only after the actual chassis kinematics and command protocol are verified. If the platform supports lateral velocity, the contract must state how Nav2/controller configuration exposes or intentionally suppresses `v_y`.

## 5. C++ runtime packages

The target runtime core is:

```text
platform/
└── agt_base_runtime/
    ├── include/agt_base_runtime/
    │   ├── base_adapter.hpp
    │   ├── chassis_profile.hpp
    │   ├── motion_limits.hpp
    │   └── motion_guard.hpp
    ├── src/
    │   ├── base_runtime_node.cpp
    │   ├── motion_guard_node.cpp
    │   ├── tracked_adapter.cpp
    │   ├── ackermann_adapter.cpp
    │   └── steering_adapter.cpp
    └── test/
```

Exact source ownership may be adjusted after workspace audit, but there must be only one participating package for the runtime responsibility.

Recommended adapter interface:

```cpp
class BaseAdapter {
public:
  virtual ~BaseAdapter() = default;
  virtual bool Configure(const ChassisProfile& profile) = 0;
  virtual AdapterCommand Convert(const geometry_msgs::msg::Twist& request) = 0;
  virtual nav_msgs::msg::Odometry NormalizeOdometry(const RawBaseState& state) = 0;
};
```

Do not expose vendor CAN/UDP packet types above the adapter boundary.

## 6. Motion Guard contract

The current Python `cmd_vel_guard` semantics are the migration baseline. The C++ guard must preserve or explicitly improve, with tests, all currently relied-on behavior:

- velocity clamp;
- acceleration / slew limiting;
- fixed-rate output refresh (current target 50 Hz);
- stale-command timeout to zero;
- localization / payload / health interlocks already present on V4;
- fail-closed startup;
- no direct driver output from Nav2 or mission nodes.

Target internal states:

```text
DISABLED
READY
ACTIVE
DEGRADED
STALE_COMMAND
LOCALIZATION_BLOCKED
HEALTH_BLOCKED
ESTOP_OR_PLATFORM_BLOCKED
```

The exact public health interface may reuse existing V4 health messages if suitable; do not create a duplicate health framework without an interface audit.

## 7. Robot Profile contract

Robot-specific configuration belongs in one validated profile, not scattered launch `if robot == ...` logic.

Target fields include:

```yaml
robot:
  id: bunker_v1

base:
  vendor: agilex_bunker
  kinematics: skid_steer
  driver_package: ...
  command_interface: twist
  odom_source: /wheel/odom
  publish_odom_tf: false
  control_rate_hz: 50
  max_linear_mps: ...
  max_angular_rps: ...
  rotate_in_place: true

frames:
  footprint_frame: base_footprint
  base_frame: base_link

navigation:
  controller_profile: tracked_rpp
  allow_spin_recovery: true

sensors:
  lidar:
    type: livox_mid360
    enabled: true
```

Ackermann profiles additionally require verified `wheelbase`, steering angle/rate bounds and minimum turning radius. Steering/swerve profiles require a separately reviewed kinematic block. Missing required physical fields must fail validation rather than assume defaults.

## 8. Sensor-layer contract

Vendor topics are allowed at the driver boundary. Downstream packages should migrate toward canonical names where doing so does not break Livox timing or existing validated algorithms.

Conceptual outputs:

```text
/agt/sensors/lidar/raw
/agt/sensors/lidar/points
/agt/sensors/imu/data
/agt/sensors/rtk/fix
/agt/sensors/camera/<name>/...
```

The raw LIO path may keep its native Livox `CustomMsg` contract if required for point timing. Canonicalization must not insert generic filters ahead of LIO.

## 9. Atomic launch ownership

Each functional owner must have an independently runnable launch entry. Aggregators may only include atomic entries; they must not recreate the same `Node` definitions.

Target launch ownership:

```text
description.launch.py
lidar.launch.py
imu.launch.py          # only when separate from lidar driver ownership
camera.launch.py
rtk.launch.py
base.launch.py
lio.launch.py
local_perception.launch.py
row_perception.launch.py
global_relocalization.launch.py
localization_manager.launch.py
motion_guard.launch.py
nav2.launch.py
rviz.launch.py
```

Convenience aggregators:

```text
sensors.launch.py
localization.launch.py
navigation.launch.py
system.launch.py
```

Atomic entries must support `--show-args` / dry-run style inspection where practical, and must not silently start another owner of hardware, TF, LIO or Nav2.

## 10. Migration rule: C++ versus Python

Use the following default classification:

- `MIGRATE_CPP`: hard real-time-ish command path, authoritative TF/state, high-rate perception, low-level adapters.
- `KEEP_PYTHON`: launch orchestration, profile/map resolution, offline tools, experiment scripts, UI glue where latency is non-critical.
- `TOOL_ONLY`: benchmark/analysis/debug code that must never become a production runtime dependency.
- `REMOVE_AFTER_PARITY`: legacy runtime only after replacement passes parity and rollback evidence exists.

The migration must begin with a source inventory; do not mechanically rewrite every Python node.

## 11. Required compatibility outcomes

A successful implementation must demonstrate:

1. Bunker can run through the new runtime without changing the external mission/navigation behavior or bypassing remote priority.
2. Ackermann support exists as a validated software contract and unit-tested conversion path; field enablement remains blocked until physical parameters/driver protocol are verified.
3. YHS/steering support is not mislabeled Ackermann. It remains blocked until its real kinematic model and command interface are confirmed.
4. Sensors, base, LIO, localization, perception, Nav2 and RViz can be started and stopped independently without duplicate owners.
5. Existing V4 compatibility aggregators continue to work until the new atomic startup is accepted.
