# Point-cloud pipeline

## Required order

```text
raw MID360 CustomMsg
  +-> Batch-LIO/FAST-LIO time-preserving input (unchanged)
  `-> secondary PointCloud2 navigation branch
       -> finite/range check
       -> TF-aware robot self-filter
       -> optional narrow rear mask
       -> radial slope ground filter
       -> 0.10 m voxel/downsample
       -> /agt/navigation/points_obstacles
       -> Nav2 local VoxelLayer
```

## Why the navigation branch is separate

The obstacle filter must not alter the per-point timing or samples consumed by LIO. The rear chassis rods are removed only from the Nav2 obstacle branch. The mapping/localization front end keeps its time-preserving input and its own validated filtering policy.

## Why not delete the rear sector by default

A broad rear-sector deletion throws away trees, structures and terrain that can make Scan Context/global registration distinctive. Therefore the optional rear-sector mask is narrow, range-limited and disabled until the real rosbag demonstrates that the geometry filter is insufficient.

## Rolling-terrain handling

Candidate points are transformed to `base_link` for self/rear/ground classification, while the published cloud retains its original source frame and timestamp. The radial filter groups points by azimuth, follows locally continuous ground up to the configured slope, and retains height discontinuities as obstacles. It is not a negative-obstacle or terrain-cost-map implementation.

## Benchmark matrix

Before changing thresholds, record results for:

- self-filter off/on
- rear mask off/on
- voxel 0.10 / 0.20 / 0.30 m
- static scene
- tracked rough ground
- tree shade
- rear-rod-heavy viewpoints

For each case record point count, FAST-LIO2 tracking quality, global localization success/time, and registration fitness.
