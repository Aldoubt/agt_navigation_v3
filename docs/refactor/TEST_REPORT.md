# Refactor test report

## Phase 0 baseline (2026-09-10)

- `colcon list --names-only`: PASS; 26 AGT repository packages discovered.
- `colcon build --symlink-install`: BLOCKED before AGT package completion.
  External workspace package `pantilt_camera_serial` failed because its existing
  `build/pantilt_camera_serial/ament_cmake_python/pantilt_camera_serial/pantilt_camera_serial`
  directory cannot be replaced by the requested symlink. Seven concurrent
  packages were consequently aborted; 42 were not processed.
- Launch parse / rosbag / simulation / real robot: not run in Phase 0.

This is not a claim of field validation. The external build-artifact collision
must be repaired by its owner or an explicitly authorized clean build before a
whole-workspace PASS can be asserted.

## Phase 3 state-estimation relocation

- `colcon list --names-only`: PASS for both relocated packages.
- `colcon build --symlink-install --packages-select agt_batch_lio_adapter agt_fastlio_adapter`: PASS (2/2).
- Launch parse, rosbag, simulation and real robot: not run. The package-name,
  executable-name, config and installed-launch contracts are intentionally unchanged.
