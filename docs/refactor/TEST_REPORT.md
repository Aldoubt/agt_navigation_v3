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

## Phase 4 localization relocation

- `colcon list --names-only`: PASS for all five relocated localization packages.
- Targeted build: `agt_localization_manager`, `agt_map_tracker`, and
  `agt_relocalization_benchmark` PASS. `agt_global_relocalization_native`
  configuration is BLOCKED by missing external `small_gicpConfig.cmake`; its
  dependent Python relocalization package was not processed.
- The five stale CMake build-cache directories were moved recoverably to a
  temporary directory before reconfiguration; no source, map or install data
  was deleted.
- `ros2 launch agt_batch_lio_adapter batch_lio_adapter.launch.py --show-args`:
  PASS. `ros2 launch agt_localization_manager localization_manager.launch.py
  --show-args`: PASS.
- `pytest navigation/localization/agt_localization_manager/test/test_correction_math.py -q`:
  PASS (2 tests). Rosbag, simulation and real-robot checks remain unrun.

## Functional-domain completion (2026-09-10)

- `colcon list --names-only`: PASS; all 27 registered AGT packages are
  discoverable from functional-domain paths.
- `CMAKE_PREFIX_PATH=/home/yangxuan/ros2_ws/.agt_native:\$CMAKE_PREFIX_PATH
  colcon build --symlink-install --cmake-clean-cache --packages-select <all 27 AGT packages>`:
  PASS. This reconfigured each relocated package and retained existing source
  and map data.
- Launch argument parsing: PASS for
  `agt_mapping_bringup/mapping_mode.launch.py`,
  `agt_system_bringup/navigation_debug.launch.py`,
  `agt_system_bringup/simulation_debug.launch.py`, and
  `agt_nav2_bringup/navigation.launch.py`.
- Software regression checks: PASS; `colcon test` reported 39 tests with
  zero errors/failures for `agt_base_control`, `agt_map_converter`,
  `agt_map_manager` and `agt_navigation_runtime`. The two directly
  discoverable pytest modules reported 6 passed.
- `guard_fail_closed_acceptance.py`: PASS. It observed a 5.8 ms
  LOST-to-stop latency and no stale command replay after mode changes.
- This is a build and composition check only. Live LiDAR, fixed-route mapping,
  rosbag replay, real-robot TF, relocalization and Nav2 motion remain
  separately required runtime validation.
