# Migration report

| Old path | New path | Status |
|---|---|---|
| all source package paths | unchanged | Phase 0: audit only; no runtime path changed |
| `src/agt_batch_lio_adapter` | `navigation/state_estimation/agt_batch_lio_adapter` | Phase 3: package, executable, launch, config, `/agt/odometry/local`, and `odom -> base_link` contract unchanged. |
| `src/agt_fastlio_adapter` | `navigation/state_estimation/agt_fastlio_adapter` | Phase 3: package, executable, launch, config and local-odometry topic contract unchanged. |
| `src/agt_global_relocalization` | `navigation/localization/agt_global_relocalization` | Phase 4: Python relocalization wrapper moved; ROS package/launch interface unchanged. |
| `src/agt_global_relocalization_native` | `navigation/localization/agt_global_relocalization_native` | Phase 4: native Polar Context/BBS/GICP code moved without modification. |
| `src/agt_localization_manager`, `src/agt_map_tracker`, `src/agt_relocalization_benchmark` | `navigation/localization/<same package>` | Phase 4: correction authority, tracker and benchmarks moved; package names unchanged. |
| remaining `src/agt_*` and `src/ros2_livox_simulation` packages | `sensor/`, `mapping/`, `cleaning/`, `navigation/nav2/`, `map_data_manager/`, `bringup/`, `interfaces/`, `tests/simulation/` | Functional-domain completion: ROS package names, installed launch names, topics and TF contracts unchanged. |

This report is intentionally initialized before relocation so every subsequent
commit can record old path, new path, reason, affected package/launch, and
compatibility result.
