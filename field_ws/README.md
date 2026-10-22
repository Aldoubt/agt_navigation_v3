# AGT Field Workspace P0 (2026-10-22)

This directory is a **version-pinned ROS 2 Humble source workspace manager**,
not a finished deployable robot image. It is intentionally part of
`agt_navigation_v3`, so **no fourth repository** is created.

## Bootstrap one workspace on Ubuntu 22.04

```bash
git clone -b feature/field-ws-p0-20261022 \
  https://github.com/Aldoubt/agt_navigation_v3.git ~/agt_field_ws_bootstrap
python3 ~/agt_field_ws_bootstrap/field_ws/field_ws.py \
  --workspace ~/agt_field_ws init
python3 ~/agt_field_ws_bootstrap/field_ws/field_ws.py \
  --workspace ~/agt_field_ws doctor
```

`init` clones three specific commit IDs to `~/agt_field_ws/src`:
- `agt_navigation_v3` – navigation and operator UI (2026-10-22 V3 FW-mini baseline)
- `fwmini_navigation_lab` – chassis, vendor CAN protocol, sensor wiring
- `agt-lio-pgo-mapping` – mapping and PCD→PGM/map asset pipeline

It **never** resets an existing clone or updates it behind your back.
The source workspace is a pinned field test release; the separate bootstrap
checkout is a maintenance tool, not a fourth repository. To edit the code,
edit a feature branch of the cloned V3/FW-mini/mapping repo as appropriate.

Run `doctor` before attempting ROS build. It reports missing ROS tooling,
mismatched pinned commits, existing local modifications, and duplicate ROS
package names. On this hardware, the `yhs_can_interfaces` from FW-mini has
**four fields**, while legacy ROS workspaces expose an incompatible type with
three fields. **Never run an unrestricted colcon build over all source trees.**

```bash
python3 ~/agt_field_ws_bootstrap/field_ws/field_ws.py \
  --workspace ~/agt_field_ws build-plan navigation
python3 ~/agt_field_ws_bootstrap/field_ws/field_ws.py \
  --workspace ~/agt_field_ws build-plan mapping
```

These print the **explicit ROS package paths**. They are not auto-builds:
external source dependencies from V3 `dependencies/*.repos`, mapping
`.repos`, and platform driver bringup must be installed and built against
compatible Humble interfaces first. Mapping and navigation should use
separate build/install roots inside the **same** `agt_field_ws`.

## Offline distance geometry check (never moves a robot)

```bash
python3 ~/agt_field_ws_bootstrap/field_ws/field_ws.py line \
  --x 0 --y 0 --yaw-deg 90 --distance 7
python3 -m unittest discover -s ~/agt_field_ws_bootstrap/field_ws/tests -v
```

The geometry generator has no ROS Action client, and never sends commands.
The live RelativeLineMission -> FollowPath adapter, Collision Monitor,
GNSS/ground-factor smoother, unified one-click launch, simulation and
field acceptance reports are **not implemented in P0**. They require
separate integration and safety verification before motion authorization.

## Distance preview and externally surveyed acceptance

The ROS2 `/agt/field/prepare_distance` service now creates a **draft preview**, not a motion goal. See `docs/ROS_DISTANCE_SERVICE.md` for the map-locked call and separate execution confirmation. The generated interface requires a new Humble build before use.

After actual field measurements, use `acceptance.py` with the independently surveyed navigation and speed CSV files. See `docs/ACCEPTANCE_REPORT.md`. Missing or internal SLAM-only measurements never count as PASS.

## Operator profiles

`config/scenarios/field_acceptance.yaml` contains official 5/7/10 m
repeat-five end-point targets (0.12/0.15/0.20 m), 30 m / 1 m/s speed
acceptance targets and fail-closed requirements. These are user-facing
**contracts**, not Nav2's ROS parameters. They do not override low-speed
firmware, Nav2, velocity smoother or navigation guard limits.

`config/scenarios/greenhouse_coverage.yaml` describes the later P3 route
coverage goal. `config/robots/fwmini.yaml` records manufacturer ratings:
nominal 1.0 m/s, **15° max full-load climb** (not the 30° field
acceptance requirement). Body dimensions and current MID360 extrinsics
are marked usable for supervised field integration, while independent
precision validation remains pending.

## Ownership and handoff

| Component | Source of truth |
|---|---|
| 3D mapping + map pack publishing | `agt-lio-pgo-mapping` |
| localization, route UI, nav mission, health | `agt_navigation_v3` |
| CAN, FW-mini chassis, platform Nav2 profiles | `fwmini_navigation_lab` |

The copy in `fwmini_navigation_lab/integrations/v3_fwmini_ui` is a
read-only historical snapshot. Do not build it alongside the V3 originals
or edit both copies separately.

See `docs/FIELD_WS_ROADMAP.md` for the exact P1/P2/P3 gates.

## P1b FW-mini front-obstacle safety candidate

The pinned FW-mini SHA in `workspace.lock.json` now includes a real
ROS 2 parameter file `src/fwmini_multi_nav_bringup/config/nav2/fwmini_obstacle_guard.yaml`
and safety-scan code. This applies to **both** Nav2 and operator local search
before the CAN adapter; it stops on unknown front obstacles and stale scan.
The separate RPP configuration `fwmini_mid360_nav2_rpp_candidate.yaml`
remains **simulation-only**; DWB remains the active field baseline.

GitHub no-CAN safety unit tests PASS. This is **not** a measured or certified
human-detection system and the hardware remains blocked at 1m/s until
stopping-distance tests. Review [FW-mini P1b Draft PR](https://github.com/Aldoubt/fwmini_navigation_lab/pull/3)
with `docs/P1B_SCAN_GUARD_HANDOFF.md` before field use.
