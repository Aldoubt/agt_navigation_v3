# P2.20 Mapping-Body Relocalization Contract

## Decision

Formal PGO map packages use `mapping_body` as their relocalization query-frame
mode. The contract is:

```text
formal poses.txt:        T_map_body
stored localization PCD: map-frame accumulation of body-frame mapping clouds
runtime GICP query:      T_body_livox(raw livox cloud)
native result:           T_map_body
published anchor:        T_map_base = T_map_body * T_body_base
```

`agt_localization_manager` remains the only `map -> odom` publisher. This
change does not alter its formula, GICP options, map-tracker algorithm, or map
assets.

## Compatibility and migration

`relocalization_query_frame_mode=mapping_body` is the default. `base_link`
continues to use its timestamped TF query path but logs a deprecation warning
because it is not frame-consistent with a formal PGO body-cloud map. The old
`body_aligned` spelling is accepted as a deprecated alias for `mapping_body`.

Candidate BBS now implements the same mapping-body candidate-pose contract.
Its default is `bbs_query_frame_mode=mapping_body`; it produces `T_map_body`
and the Python wrapper performs the one required conversion before publication.
The former `base_link` candidate path remains available only as an explicitly
selected deprecated compatibility mode.

## Map-package declaration

New packages created by `agt_map_manager/create_map_package` write:

```yaml
relocalization_contract:
  formal_pose_semantics: T_map_body
  map_cloud_frame: body
  query_frame_mode: mapping_body
  query_frame: body
```

The package validator rejects a declared contract whose pose semantics, cloud
frame, query mode, or query frame disagrees. Legacy immutable schema-v1
packages without this declaration remain readable; they must be bound by
separate provenance evidence before field acceptance.
