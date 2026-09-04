# Global Relocalization V1: 3D-BBS + small_gicp

## Decision

V1 global relocalization is LiDAR-map based and does **not** use RTK/INS as an automatic seed or correction source.

Runtime chain:

```text
/agt/relocalization/request
        |
        v
agt_global_relocalization
  accumulate /agt/livox/points
        |
        v
query_scan.pcd
        |
        v
agt_global_relocalization_native/bbs_gicp_localizer
        |
        +--> 3D-BBS: no-initial-pose coarse global search
        |
        +--> small_gicp: GICP refinement
        |
        v
JSON pose + score + fitness + overlap
        |
        v
quality gates / covariance mapping
        |
        v
/agt/relocalization/pose
        |
        v
agt_localization_manager
        |
        v
map -> odom
```

`agt_localization_manager` remains the only owner of `map -> odom`.

## Why not copy Ikunio/Lidar_nav2_ws wholesale

The Ikunio workspace is valuable reference material and is MIT licensed, but its `global_small_gicp_relocalization` node is not by itself a true no-initial-pose global localizer. The inspected implementation initializes small_gicp from `previous_result_t_` / RViz `initialpose` and directly broadcasts `map -> odom`.

We therefore migrate only the useful registration pattern:

- accumulate several registered PointCloud2 scans;
- preprocess map/query clouds;
- use small_gicp for precise GICP refinement;
- keep configuration tunable.

We do **not** migrate its TF ownership or initial-pose dependency.

## Coarse global search

Use upstream `KOKIAOKI/3d_bbs` rather than a copied workspace fork.

3D-BBS is specifically designed for initial global 3D LiDAR localization from a pre-built point-cloud map. It supports no initial pose. For practical speed it expects the query scan to be approximately gravity aligned; V1 therefore bounds roll/pitch search while searching yaw over the full `[-pi, pi]` range.

Initial CPU policy:

- CPU BBS first; CUDA is optional later;
- translation range = complete map bounds;
- yaw = full 360 degrees;
- roll/pitch = bounded around gravity alignment;
- timeout is mandatory;
- BBS score must pass a minimum threshold before refinement.

The CPU baseline avoids making the field demo depend on a specific NVIDIA/CUDA installation.

## Fine registration

Use upstream `koide3/small_gicp` directly.

small_gicp is used only after 3D-BBS gives a coarse transform. Fine registration returns:

- refined `T_map_scan`;
- convergence state;
- inlier count / overlap proxy;
- optimizer error / fitness proxy.

The AGT wrapper maps those metrics into the existing `score`, `fitness`, and `overlap` gates.

## Why Scan Context is not copied into V1

The canonical Scan Context implementation is technically mature and useful for place retrieval, but its public repository states a CC BY-NC-SA license. V1 therefore does not copy or vendor that code into the product repository.

A future descriptor database can still be added behind the same backend boundary if a commercially compatible implementation is selected or separately licensed.

## RTK/INS policy

`agt_ins_driver` and `agt_rtk_manager` remain useful for:

- geographic metadata while mapping;
- inspection/camera/gimbal record association;
- operator diagnostics and RTK health;
- future map-to-ENU survey workflows.

They do **not** feed `agt_global_relocalization`, do not generate a relocalization initial pose, and never publish `map -> odom` in V1.

## Dependencies

Pinned versions live in `dependencies/field_demo.repos`:

- `KOKIAOKI/3d_bbs`
- `koide3/small_gicp`
- mapping `robotics-laboratory/fast-lio2`
- navigation `Functionhx/Batch-LIO`
- optional calibration `morte2025/LiDAR_IMU_Init_ROS2`

## Build note

3D-BBS currently installs headers and `libcpu_bbs3d` but does not provide a normal CMake package config. Build it first:

```bash
cd external/3d_bbs
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_CUDA=OFF
make -j
sudo make install
```

Then install small_gicp helper library:

```bash
cd external/small_gicp
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_HELPER=ON
make -j
sudo make install
```

Only then build `agt_global_relocalization_native`.

## Acceptance before navigation

A real MID360 bag must prove:

1. BBS can recover from multiple deliberately different starting locations and headings without `/initialpose`.
2. Wrong BBS hypotheses are rejected rather than accepted by GICP.
3. Final GICP overlap and fitness distributions provide separable good/bad thresholds.
4. Published `/agt/relocalization/pose` aligns with the optimized mapping PCD.
5. Repeated relocalization does not create a second `map -> odom` broadcaster.
6. RTK loss or drift has no effect on the relocalization result.
