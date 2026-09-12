# Candidate BBS `mapping_body` Contract

## Modes

| `bbs_query_frame_mode` | Query cloud | Descriptor candidate pose | Native result | ROS publication |
| --- | --- | --- | --- | --- |
| `mapping_body` (formal default) | raw Livox transformed with mapping-era `T_body_livox` | unchanged formal `T_map_body` | `T_map_body` | wrapper publishes `T_map_base=T_map_body*T_body_base` |
| `base_link` (deprecated compatibility) | `base_link` via timestamped TF | formal `T_map_body` converted once to `T_map_base` | `T_map_base` | unchanged |

The BBS search ranges, descriptor retrieval, and GICP settings are identical
between modes. Only the explicit source/candidate/result frame contract differs.

## Launch migration

Deprecated compatibility invocation (both modes must be explicit):

```bash
relocalization_query_frame_mode:=base_link bbs_query_frame_mode:=base_link
```

Formal default invocation (the explicit values are useful for audit logs):

```bash
relocalization_query_frame_mode:=mapping_body bbs_query_frame_mode:=mapping_body
```

Mixed modes are rejected before backend invocation. This prevents a body-frame
query cloud from being registered against base-frame descriptor candidates, and
prevents a `T_map_body` result from reaching LocalizationManager unconverted.

`LocalizationManager` continues to receive only `T_map_base` on
`/agt/relocalization/pose`; its `map -> odom` formula is unchanged.
