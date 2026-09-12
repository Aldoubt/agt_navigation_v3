# Functional-domain completion record

## Scope

All registered AGT ROS packages formerly directly below `src/` have been
relocated without changing package names, launch-file names, ROS topics, or TF
frame contracts.

| Domain | Contents |
|---|---|
| `sensor/` | Livox tools/simulation, RTK manager, optional camera capability |
| `mapping/` | FAST-LIO2 + PGO bringup, mapping session, backend prototype |
| `cleaning/` | point-cloud preprocessing, map conversion, terrain/map cleaning |
| `navigation/state_estimation/` | Batch-LIO, FAST-LIO and simulation odometry adapters |
| `navigation/localization/` | BBS/GICP relocalization, localization manager, tracker, benchmark |
| `navigation/nav2/` | Nav2 bringup, runtime, RViz patrol, optional demo task |
| `map_data_manager/` | versioned map package, validation, edit and publish lifecycle |
| `bringup/` | system, base-control and operator-console composition |
| `interfaces/` | AGT messages, services and actions |
| `tests/simulation/` | Gazebo simulation test package |

The unregistered `agt_mapping_backend_adapter` prototype was also relocated
into `mapping/`. It has no `package.xml`, is not installed or launched, and
is not a runtime dependency.

## Test-stage map policy

The default mapping run is FAST-LIO2 + PGO on the fixed test route. Its PGO
output is the full relocalization source map; navigation-grid conversion and
relocalization-asset generation are explicit post-run operations. The default
`navigation_debug.launch.py` uses the fixed
`bunker_mid360_mapping_20260901_205036/v003-indexed` assets directly.

`agt_map_manager` stays available for draft/edit/publish/reseal and
production active-map lifecycle. Neither the test mapping launch nor
navigation-debug launch starts it automatically.

## Compatibility evidence

All 27 registered AGT packages passed discovery and a
`--symlink-install --cmake-clean-cache` build after relocation. Four
representative mapping, hardware-debug, simulation-debug and Nav2 launch files
also passed `--show-args`. See
`docs/refactor/TEST_REPORT.md` for the command scope and unverified runtime
items.
