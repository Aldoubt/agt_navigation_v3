# P2.21 Global Localization Frame-Contract Acceptance Checklist

- [ ] Active map package declares formal poses as `T_map_body` and mapping
      clouds as `body` (or has equivalent immutable provenance evidence).
- [ ] `relocalization_query_frame_mode:=mapping_body` is in the invoked launch.
- [ ] Candidate BBS uses `bbs_query_frame_mode:=mapping_body`.
- [ ] Manual seed uses a map-frame `T_map_base_link` seed; the wrapper converts
      it to `T_map_body` only for native GICP and converts the result back once.
- [ ] Candidate BBS status records `query_frame=body`, refined
      `T_map_body`, and published `T_map_base_link`.
- [ ] Mixed `relocalization_query_frame_mode` and `bbs_query_frame_mode` are
      rejected before backend invocation.
- [ ] `base_link/base_link` is used only for an explicitly documented legacy
      map package and its deprecation warning is captured in the run log.
- [ ] `/agt/relocalization/pose` is `map` to `base_link`; only
      `agt_localization_manager` broadcasts `map -> odom`.
- [ ] No GICP settings, BBS search settings, map assets, LocalizationManager,
      or MapTracker were changed as part of this contract migration.
