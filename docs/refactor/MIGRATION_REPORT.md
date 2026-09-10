# Migration report

| Old path | New path | Status |
|---|---|---|
| all source package paths | unchanged | Phase 0: audit only; no runtime path changed |
| `src/agt_batch_lio_adapter` | `navigation/state_estimation/agt_batch_lio_adapter` | Phase 3: package, executable, launch, config, `/agt/odometry/local`, and `odom -> base_link` contract unchanged. |
| `src/agt_fastlio_adapter` | `navigation/state_estimation/agt_fastlio_adapter` | Phase 3: package, executable, launch, config and local-odometry topic contract unchanged. |

This report is intentionally initialized before relocation so every subsequent
commit can record old path, new path, reason, affected package/launch, and
compatibility result.
