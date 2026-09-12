# Target functional architecture

The first migration preserves package names; directories become domain-owned
without making a directory imply a process boundary.

```text
agt_navigation_v3/
  sensor/                 # Livox/IMU/RTK wrappers and adapters
  mapping/                # LIO mapping, PGO, session, export
  cleaning/               # offline map cleaning and online cloud filtering
  navigation/
    state_estimation/     # LIO navigation adapters/wrappers
    localization/         # BBS/GICP, monitor, map correction, tracker
    nav2/                 # Nav2 launch/config integration
  map_data_manager/       # map package lifecycle
  bringup/                # orchestration only
  interfaces/             # ROS IDL
  common/                 # only deliberately small shared helpers
  third_party/            # policy/repo manifests, not copied upstream sources
  tests/
  docs/
```

Compatibility aliases remain installed under existing package names. The target
top-level `agt_bringup` package is a later additive wrapper: it must include,
not replace, the proven `agt_system_bringup` field launch until field regression
has passed.
