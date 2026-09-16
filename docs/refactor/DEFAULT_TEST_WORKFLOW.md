# Default test mapping and navigation workflow

## Boundary

The mapping front end is `FAST-LIO2 + PGO`, launched and exported by the
separate mapping-producer workspace. `agt_navigation_v3` does not launch the
mapping front end or the OctoMap branch. The PGO result is the authoritative
full-map PCD, trajectory and keyframe set consumed through a validated Map
Package.

The test route is deliberately fixed. A mapping run must cover that route
before its artifacts are accepted as a replacement candidate. It does not
automatically activate or publish a map package. After the run, explicitly
generate the 3D-BBS/Polar Context assets and a Nav2 grid, then
create/validate/reseal the PCD + navigation assets using `agt_map_manager`.
This keeps the versioned map infrastructure intact without making map lifecycle
a hidden navigation-start side effect.

## Default debug navigation asset

The default debug-navigation assets remain the fixed test package:

```text
agt_data/maps/bunker_mid360_mapping_20260901_205036/v003-indexed/
  localization/global_map.pcd
  navigation/map.yaml
```

`navigation_debug.launch.py` uses those explicit paths by default and bypasses
active-map, SHA validation and publish lifecycle. It accepts replacements only
through its explicit `localization_map:=...` and `navigation_map:=...`
arguments. Production HMI retains its separate Map Manager lifecycle path; the
Map Manager package is installed but never auto-started by the test mapping or
debug-navigation workflows.
