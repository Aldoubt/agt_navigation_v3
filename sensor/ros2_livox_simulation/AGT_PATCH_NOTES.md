# AGT validated Gazebo Livox variant

Upstream lineage: `stm32f303ret6/livox_laser_simulation_RO2`, MIT licensed.

This vendored copy is kept in the AGT repository so the validated Gazebo gate is
reproducible without depending on an unversioned workspace-local source tree.
AGT changes used by the 2026-09-05 Humble/Gazebo Classic acceptance include:

- ROS 2 `livox_ros_driver2/msg/CustomMsg` publication with a non-zero `timebase`;
- deterministic monotonic per-point `offset_time` over a configurable scan period;
- explicit SDF min/max range gating to prevent invalid hundreds-of-metres rays;
- Jammy/Humble link logic using CMake targets instead of distro-specific `.so` names.

Keep the upstream MIT `LICENSE` with this package. Changes should be validated
against the full Gazebo mapping -> map assets -> automatic relocalization ->
Nav2 acceptance flow before replacing this copy.
