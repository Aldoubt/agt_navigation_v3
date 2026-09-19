# AGT Localization v1 — Patch 1 Core Layer

## Purpose

Patch 1 adds the first two non-runtime packages for the staged `legacy -> shadow -> v1` migration:

- `agt_localization_interfaces`: v1-internal typed contracts;
- `agt_localization_core`: ROS-independent localization primitives.

No v1 manager, TF broadcaster, launch inclusion, topic replacement, or parameter-default change is introduced in this patch.

## Added interfaces

| Interface | Purpose | Runtime migration status |
| --- | --- | --- |
| `Keyframe.msg` | timestamped local pose/cloud keyframe contract | defined only; no publisher/subscriber started |
| `MapMetadata.msg` | Map Package identity and resolved asset paths | defined only; no Map Runtime node started |
| `LocalizationState.msg` | v1 internal state description | defined only; does not replace `agt_robot_interfaces/LocalizationStatus` |
| `GetLocalMap.srv` | future local map query contract | defined only; no service server started |

The existing `agt_robot_interfaces/msg/LocalizationStatus`, `/agt/localization/status`, `/agt/relocalization/status`, and `LocalizationMetrics` interfaces are unchanged.

## Extracted pure logic

`agt_localization_core` mirrors, without ROS dependencies:

- SE(3) pose composition, inversion, yaw/wrap helpers, and the frozen formula
  `T_map_odom = T_map_base * inverse(T_odom_base)`;
- tracking translation/yaw innovation calculation and strict `>` gate;
- correction interpolation, translation rate limit, yaw rate limit, and disabled-smoothing behavior;
- timestamped nearest-odom buffer behavior;
- legacy recovery state/cooldown transitions for `BOOT`, `WAIT_GLOBAL`,
  `LOCALIZED`, `TRACKING`, `DEGRADED`, `LOST`, `RECOVERY_REQUESTED`, and
  `RELOCALIZING`.

The legacy manager remains the runtime implementation and sole `map -> odom` owner. The core module neither imports ROS libraries nor publishes anything.

## Legacy parity verification

The core tests import the existing manager only as a test oracle. They compare:

1. `T_map_odom` composition and compose/inverse round trips;
2. boundary behavior of the correction innovation gate, measurement geometry,
   and rate-limited SE(3) smoothing;
3. state names, LOST/recovery/cooldown transitions, and global pose acceptance.

These tests do not instantiate `LocalizationManager`, start ROS, publish TF, or alter legacy code. They establish a fixed reference before Patch 2 adds any compatibility bridge or shadow runtime.

## Explicit non-changes

- `agt_global_relocalization`, its native BBS/Polar Context/small_gicp tools, and `agt_map_tracker` are untouched.
- Batch-LIO and FAST-LIO adapters are untouched.
- Nav2, field launch defaults, topic names, TF frames, TF ownership, and YAML defaults are untouched.
- No legacy file/package is moved or deleted.
