# Localization Shadow Metrics

This is an offline comparison only. It does not publish TF, change the TF owner, or activate v1.

## Inputs

- Legacy bag: `/home/yangxuan/ros2_ws/nav_full_baseline_20260917_114327`
- Shadow bag: `/home/yangxuan/ros2_ws/experiments/results/localization_shadow_validation/nav_full_baseline_20260917_114327_shadow_clean`
- Synchronization: nearest message timestamp, maximum offset `0.250s`.

## Recorded Localization Contract

| Topic | Messages |
| --- | ---: |
| `/agt/odometry/local` | 2600 |
| `/agt/relocalization/pose` | 0 |
| `/agt/map_tracking/pose` | 0 |
| `/agt/map_tracking/status` | 0 |

A full correction-parity replay requires local odometry, at least one global relocalization pose, and map-tracking observations. This input does not meet that complete-replay precondition.

## Stream Counts

- Legacy metrics: 1325
- Legacy status: 1325
- Shadow state: 265
- Shadow diagnostics: 265

## Time Synchronization

- Matched rows: 265
- Unmatched shadow samples: 0
- Mean absolute timestamp offset: 0.039626 s
- Maximum absolute timestamp offset: 0.046661 s

## State and Correction Comparison

- Exact state matches: 0/265
- Translation error samples: 0; mean: N/A m; max: N/A m
- Yaw error samples: 0; mean: N/A rad; max: N/A rad
- State pairs: `DEGRADED` -> `SEARCHING`: 1, `LOCALIZED` -> `SEARCHING`: 192, `LOCALIZED` -> `UNINITIALIZED`: 4, `LOST` -> `SEARCHING`: 68.
- Shadow correction decisions: `none`: 265.
- `innovation` in the CSV is JSON with `translation_m` and `yaw_rad`; it is retained even when no correction was accepted.

## Validation Result

**BLOCKED_INPUT_CONTRACT** — neither global relocalization nor map-tracking pose messages were recorded. The legacy correction present in metrics cannot be replayed into shadow, so numeric correction parity is not established.

## Warnings

- None
