# One-workspace field demonstration work order

## Gate P0 — environment and source reproducibility (this PR)
- [x] Three pinned source repositories; avoid main rewrites or new repo.
- [x] Explicit owner for duplicate ROS package names.
- [x] Separate mapping / navigation candidate base-path build plans.
- [x] Validation of pure 5/7/10 m map-frame line geometry.
- [x] Two YAML operator scenarios + one FW-mini geometry/hardware profile.
- [ ] Install source dependencies and perform Humble build on the real PC.
- [ ] Single `field_console.launch.py` orchestration with map ID/hash checks.
- [ ] One-click mapping -> optimize (no loop) -> PCD/PGM/BBS assets.

## Gate P1 — 5/7/10 m field test and obstacle stopping
- [ ] Convert the pure line geometry into a ROS 2 Action server that snapshots
      confirmed `map->base_link` and accepts an explicit operator confirmation.
- [ ] Confirm identity of the selected map (same PCD and OccupancyGrid version).
- [ ] Verify every velocity hop: controller -> smoother -> Collision Monitor ->
      Navigation Guard -> FW-mini ABI-correct CAN -> physical feedback.
- [ ] Simulate low speed and then stage up carefully, using vendor limits;
      1.0 m/s is the **measured** acceptance target, not a safe default.
- [ ] Record actual measured 30 m section time twice and 5/7/10 m end-point
      error five times each with independent measurement of arrival point.
- [ ] Front person/unknown obstacle causes STOP; no leaf filtering until
      semantically and physically validated.
- [ ] Test stale point cloud, TF, odom, localization loss and immediate stop.

## Gate P2 — mapping, GNSS and ground factors without bad loop closures
- [ ] Disable PGO global loop closure, avoid exporting optimized:true as proof.
- [ ] FAST-LIO2 map + PCD->PGM + BBS/GICP index + versioned map package.
- [ ] Implement the ACTUAL ground plane/height normal factor within a fixed-lag
      optimizer with degeneracy and slope-aware confidence checks.
- [ ] Implement the ACTUAL quality-gated GNSS factor (frame transform,
      lever arm, covariance, fix status, innovation and freshness).
- [ ] No competing publishers of map->odom; local relative odom is always
      measured independently of global corrections.
- [ ] Repeated greenhouse row candidates remain AMBIGUOUS until discriminated
      by unique observation or a valid absolute measurement. Consistent
      motion by itself cannot disambiguate perfectly identical rows.

## Gate P3 — greenhouse full-row coverage and HMI
- [ ] Segment PCD ground, wall/poles, and planted row boundaries;
      project a reviewed occupancy grid as visual base map.
- [ ] Auto-suggest row corridors + human move/delete/add/review in Qt.
- [ ] Enforce chassis turn radius / steering modes at row ends; no invented
      off-map connector. Support return to confirmed start.
- [ ] RPP vs DWB oscillation A/B using identical route, TF and latency profiles.
- [ ] Repeatable rosbag replay and simulation before any live motor enable.

## Release discipline
No branch merges to main without unit tests, ROS Humble build, no-CAN
integration test, signed/stable map identity, physical stop-distance
measurements, safe watchdog and an operator-controlled enable.
