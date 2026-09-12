# P2.21 Global Localization Frame-Contract Freeze

## Frozen default

Formal PGO map packages use `mapping_body` for every global-localization
backend.  This is now the default for `relocalization_query_frame_mode` and
`bbs_query_frame_mode` in the node configuration and all replay/demo launches.

```text
formal pose / mapping cloud: T_map_body / body
query cloud:                 T_body_livox(raw Livox cloud)
native coarse/refined pose:  T_map_body
ROS published anchor:        T_map_base_link = T_map_body * T_body_base_link
```

`agt_localization_manager` remains the sole `map -> odom` publisher and still
receives only `T_map_base_link` on `/agt/relocalization/pose`.

## Migration

Existing integrations which explicitly rely on the historical base-frame
candidate backend remain supported only with both settings explicit:

```bash
relocalization_query_frame_mode:=base_link \
bbs_query_frame_mode:=base_link
```

That path emits a deprecated-compatibility warning.  Mixed settings are a
hard launch-time/runtime contract error before a backend is started:

```text
mapping_body + base_link  -> rejected
base_link + mapping_body  -> rejected
```

This freeze changes no GICP option, BBS candidate/search setting, map asset,
LocalizationManager calculation, or MapTracker calculation.

## Evidence

The runtime A/B evidence is stored outside the immutable map package at:

`agt_data/acceptance_evidence/.../v003-indexed/p2_21_candidate_bbs_frame_ab/comparison.yaml`.

With correction application disabled, the `mapping_body/mapping_body` case
accepted 192/192 MapTracker measurements with zero `RECOVERY_REQUIRED`; the
deprecated base-link case accepted 6/152 and produced 142
`RECOVERY_REQUIRED` states.  The runs used different automatic stationary
queries, so this is operational safety evidence rather than a same-cloud
registration benchmark.
