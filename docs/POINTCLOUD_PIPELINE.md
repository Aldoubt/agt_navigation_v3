# Point-cloud pipeline

## Required order

```text
raw MID360
  -> message validation
  -> NaN/finite check
  -> range/blind filter
  -> TF-aware robot self-filter
  -> optional narrow rear mask
  -> voxel/downsample
  -> FAST-LIO2 / global localization
```

## Why self-filter comes before registration

The rear chassis rods are static with respect to the robot but are not part of the environment map. If retained, they can produce persistent false correspondences. Removing only the known robot geometry preserves useful environmental structure behind the robot.

## Why not delete the rear sector by default

A broad rear-sector deletion throws away trees, structures and terrain that can make Scan Context/global registration distinctive. Therefore the optional rear-sector mask is narrow, range-limited and disabled until the real rosbag demonstrates that the geometry filter is insufficient.

## Tilt handling

The cloud is not levelled. All filtering and registration operate in `lidar_link`, while robot geometry is transformed from `base_link` through TF. This keeps the measured MID360 mounting tilt consistent between mapping and localization.

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
