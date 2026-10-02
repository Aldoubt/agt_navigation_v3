# P7 Nav2 profile contract audit

Date: 2026-10-02

Branch: `refactor/navigation-runtime-v4`

This audit covers Nav2 profile selection and parameter fan-out for the existing
Bunker baseline and the future Ackermann contract. It does not certify a new
vehicle or claim field tuning.

## Profile inventory and ownership

| Profile | Source | Planner | Controller / recovery | Physical geometry and limits | State |
|---|---|---|---|---|---|
| `bunker_v1` / `bunker_tracked` | `config/navigation_profiles/bunker_v1.yaml` plus the six canonical YAML files in `config/` | Smac 2D | Regulated Pure Pursuit; rotate-to-heading enabled; reversing disabled; Spin, BackUp and Wait enabled | Footprint comes from `agt_robot_description/config/robot_profiles/bunker_v1.yaml` and `config/robot.yaml`; numerical motion limits come from `config/safety.yaml` | Existing behavior retained and checked before selection |
| Ackermann contract | `config/navigation_profiles/ackermann.schema.json`; no runnable profile is shipped | Smac Hybrid, `DUBIN`; minimum radius comes from the Robot Profile | Regulated Pure Pursuit; rotate-to-heading, reversing, Spin and BackUp disabled; explicit progress checker and lookahead settings required | Requires measured wheelbase, steering bounds/rate, minimum turning radius, footprint and positive motion limits | Software contract tested with an unverified temporary fixture; field selection rejects it |
| YHS / steering | `docs/upgrade/YHS_BASE_PROTOCOL_AUDIT.md` | None | None | Chassis kinematics, protocol and limits remain unresolved | `BLOCKED`; resolver refuses explicit config and Bunker fallback |

`navigation.launch.py` owns Nav2 profile selection and builds one merged runtime
parameter file. The `agt_nav2_bringup` atomic launch owns Nav2 node creation;
the navigation aggregator includes it and does not instantiate Nav2 nodes.
Nav2 reads `/agt/odometry/local`, uses `map` and `base_link`, and leaves TF
ownership with the existing localization/LIO owners. This phase adds no TF or
command publisher.

## Preserved Bunker baseline

The tracked manifest and selector verify these existing sources before using
the canonical Bunker directory:

- `navigation.yaml`: `nav2_smac_planner/SmacPlanner2D`; `behavior_server`
  retains `spin`, `backup` and `wait`.
- `controller.yaml`: `nav2_regulated_pure_pursuit_controller`,
  `use_rotate_to_heading: true`, `allow_reversing: false`, and an explicit
  `nav2_controller::PoseProgressChecker` (0.05 m, 0.05 rad, 15 s).
- `robot.yaml`: footprint `[[0.52, 0.40], [0.52, -0.40], [-0.52, -0.40],
  [-0.52, 0.40]]`, 0.03 m padding, `base_link` and `base_footprint` frames.
  Runtime construction injects this footprint into both costmaps.
- `safety.yaml`: 0.55 m/s forward, 0.20 m/s reverse and 0.65 rad/s angular
  limits; 0.40 m/s controller cruise, 0.05 m/s approach and 0.10 m/s regulated
  minimum. The same file supplies acceleration limits to the smoother and
  motion guard.
- The Robot Profile remains `skid_steer`, allows in-place rotation, has
  `publish_odom_tf: false`, and records a 50 Hz chassis rate. Controller,
  velocity smoother and command guard each retain at least 50 Hz.

The runtime Bunker validator compares the planner/controller/recovery choices,
footprint, padding and profile motion limits to their declared sources. A
change to the Bunker contract fails selection instead of silently changing the
compatibility baseline.

## Ackermann fail-closed contract

The schema has no synthetic default dimensions. A future Ackermann Robot Profile
must provide measured `wheelbase_m`, steering angle bounds, steering rate and
`minimum_turning_radius_m`, footprint width/length and safety margin, and must
set `rotate_in_place: false`. The runtime
validator checks that the declared minimum radius is not tighter than
`wheelbase / tan(max(abs(steering angle bounds)))`, uses that same radius in the
Smac Hybrid planner, and requires the DUBIN motion model while reversing is
disabled.

The profile validator also requires:

- RPP with `use_rotate_to_heading: false` and `allow_reversing: false`;
- no Spin or BackUp recovery; Wait is allowed;
- an explicit progress checker with positive movement/time thresholds and
  positive RPP lookahead/collision horizon settings;
- Nav2 footprint and padding matching the Robot Profile, and frame names
  matching its base/footprint frames;
- safety limits sourced from `safety.yaml`, reverse speed zero, finite positive
  acceleration/deceleration limits, angular speed within both the Robot Profile
  cap and `forward speed / minimum turning radius`, and controller speeds within
  the forward-speed cap;
- controller, smoother, motion guard and Robot Profile control rate at or above
  50 Hz.

The selector additionally requires a measured profile manifest with
`field_verified: true` and a reviewer. No such profile exists, so there is no
field-usable Ackermann Nav2 bundle in this repository. The test fixture uses
explicitly synthetic numbers only in a temporary directory; its manifest stays
`field_verified: false`, and selection is asserted to reject it. The fixture is
not installed as a Robot Profile or Nav2 profile.

These planner/controller choices follow the Humble Nav2 guidance: Smac Hybrid
uses a vehicle turning radius and kinematically feasible motion model, and RPP
can pair with a kinematically feasible planner for Ackermann motion. See the
[Humble Smac planner documentation](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_smac_planner/README.md)
and the [Humble Regulated Pure Pursuit documentation](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_regulated_pure_pursuit_controller/README.md).

## Verification and limits

P7 ran source-level configuration tests and built `agt_system_bringup` in
`/tmp/agt_runtime_v4_p7`. The tests do not start Nav2, query a ROS graph, replay
a bag, or send a base command. The installed profile manifest and schema were
confirmed under that isolated install prefix.

No physical Ackermann or YHS profile, measured turning-radius evidence, rosbag
replay or hardware test was available. Those claims remain `NOT_RUN`; YHS
hardware enablement remains `BLOCKED`.

## Rollback

The canonical six Bunker YAML files and their values remain the active default.
Before any field rollout of this phase, roll back the P7 commit with
`git revert <P7-commit-SHA>`; this removes the profile selector changes and
returns the launch code to its previous Bunker/YHS selection behavior. The
Motion Guard's existing Python rollback remains
`--motion-guard-backend python`. No Ackermann or YHS runtime adapter is started
by this profile work.
