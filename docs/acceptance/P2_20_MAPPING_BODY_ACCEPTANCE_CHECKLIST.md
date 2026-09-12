# P2.20 Mapping-Body Acceptance Checklist

- [ ] Map package validates its `relocalization_contract` as `T_map_body` /
      `mapping_body` /
      `body`.
- [ ] Replay uses `global_query_frame_mode:=mapping_body`.
- [ ] Manual seed is a complete `T_map_base`; conversion to native seed is
      `T_map_body = T_map_base * inverse(T_body_base)`.
- [ ] Accepted native result is converted exactly once:
      `T_map_base = T_map_body * T_body_base`.
- [ ] Global relocalization reports accepted fitness, overlap, and published
      base pitch without an installation-extrinsic pitch jump.
- [ ] Map tracker is first run open-loop
      (`map_tracker_apply_correction:=false`); record state and innovation
      distributions before considering correction authority.
- [ ] `RECOVERY_REQUIRED`, localization `LOST`, and TF/Nav2 lifecycle failures
      remain release blockers.
- [ ] `base_link` is used only for legacy/BBS compatibility and its warning is
      captured in the replay log.
- [ ] Map Gate remains independently governed by P0.7; relocalization runtime
      success does not promote a `MAP REVIEW` to `PASS`.
