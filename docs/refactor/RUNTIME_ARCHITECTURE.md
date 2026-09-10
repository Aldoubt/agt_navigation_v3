# Runtime architecture baseline

This is a process-level observation, separate from functional-domain layout.
Launch files instantiate ordinary `Node` actions; no AGT component container or
custom executor was found. `ros2_livox_simulation` registers a Gazebo component,
but it is simulation-only.

| Runtime area | Processes / high-bandwidth edges | Notes |
|---|---|---|
| Sensor and LIO | Livox driver -> Batch-LIO -> adapter | Preserve Livox `CustomMsg` timing directly to LIO. |
| Secondary cloud | Livox format bridge -> `/agt/livox/points` -> obstacle/relocalization/tracker | PointCloud2 DDS fan-out; candidate for future measured intra-process study only. |
| Localization | global relocalizer/native executable -> localization manager -> map tracker | Manager exclusively publishes global correction TF. |
| Navigation/control | Nav2 servers + smoother -> guard -> Bunker driver | 50 Hz command chain; manual chassis arbitration remains external. |
| Assets/runtime | map manager -> status/active-map paths -> localization and Nav2 launch | Current field demo uses explicit assets; HMI path owns active-map lifecycle. |

Future `proc-lens` work may measure callback latency, cloud bandwidth and memory
before considering composition. This refactor changes neither executor nor DDS
topology.
