# Rosbag relocalization benchmark

## Is same-bag map + relocalization testing valid?

Yes, as a **closed-set smoke test**. It is useful for checking that the complete mapping/relocalization geometry, frames, point format, BBS search and GICP refinement are internally consistent.

It is **not** sufficient evidence for field robustness because the query scans also contributed to the map. Repeated vegetation, tree rows and static structures can therefore look easier than a new traversal. The field baseline must be checked on a second rosbag after the same-bag sweep.

## Preferred fast workflow: use PGO patches

`robotics-laboratory/fast-lio2` `/pgo/save_maps` with `save_patches=true` writes:

```text
site_A/
├── map.pcd
├── poses.txt
└── patches/
    ├── 0.pcd
    ├── 1.pcd
    └── ...
```

Each patch is a body-frame keyframe cloud and each `poses.txt` row contains its final optimized global pose. This is already a relocalization benchmark dataset.

### 1. Build map and save patches

```bash
ros2 launch agt_mapping_bringup mapping_mode.launch.py use_sim_time:=true
ros2 bag play /data/bags/site_A --clock

ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/data/site_A', save_patches: true}"
```

If HBA is used, finish HBA/map refinement first and use the final PCD for the sweep. Keep in mind that `poses.txt` belongs to the PGO key poses; after an HBA transformation that changes key poses, regenerate/convert the corresponding reference poses before treating sub-meter errors as absolute truth.

### 2. Build relocalization assets

```bash
ros2 run agt_global_relocalization_native build_relocalization_assets \
  --map /data/site_A/map.pcd \
  --output /data/site_A/relocalization
```

### 3. Convert PGO keyframes into benchmark cases

For a quick first pass, use every fifth keyframe:

```bash
ros2 run agt_relocalization_benchmark import_pgo_cases \
  /data/site_A \
  --stride 5
```

This creates `/data/site_A/cases.csv` referencing `patches/*.pcd`.

### 4. Sweep BBS/GICP candidates

```bash
ros2 run agt_relocalization_benchmark sweep \
  --map /data/site_A/map.pcd \
  --assets /data/site_A/relocalization \
  --cases /data/site_A \
  --config $(ros2 pkg prefix agt_relocalization_benchmark)/share/agt_relocalization_benchmark/config/relocalization_sweep.yaml \
  --output /data/site_A/relocalization_benchmark
```

Outputs:

```text
relocalization_benchmark/
├── trials.csv
├── summary.yaml
└── best_params.yaml
```

The ranking is deliberately conservative:

1. fewer **false-positive** global poses wins first;
2. then higher correct success rate;
3. then lower XY/yaw error;
4. then lower runtime.

A clean FAILED result is preferable to a confident but wrong `map` pose.

The initial candidate set sweeps:

- query voxel size (`scan_leaf`);
- BBS minimum level resolution;
- BBS hierarchy depth;
- BBS score threshold;
- roll/pitch search range;
- GICP max correspondence distance;
- local submap radius.

The sweep also reports suggested `min_score`, `min_overlap` and `max_fitness` gates from the correct/incorrect result distributions. These are recommendations, not automatically written into the production runtime.

## Optional online case capture while replaying the bag

To test the mapping TF chain itself, run this while mapping the rosbag:

```bash
ros2 run agt_relocalization_benchmark capture_cases --ros-args \
  -p use_sim_time:=true \
  -p cloud_topic:=/fastlio2/body_cloud \
  -p reference_frame:=map \
  -p query_frame:=body \
  -p sample_interval_sec:=5.0 \
  -p output_dir:=/data/site_A/replay_cases
```

It stores a body-frame query PCD and the simultaneous `map -> body` TF every few seconds. This is useful for replay/debugging, but PGO can later adjust historical key poses, so PGO `patches + poses.txt` is the preferred reference for parameter selection.

## Second playback: end-to-end integration test

After selecting a candidate, replay the original bag again through the **navigation/relocalization** software chain. For offline testing, use one query frame at a time or sample moments where the platform is stationary; the production V1 global relocalizer intentionally rejects moving queries.

ROS 2 Humble rosbag2 supports pause/resume/seek/play-next/burst control services, so a later runner can automate time-point stepping without restarting the player for every sample.

Verify for several positions:

```text
/livox/lidar + /livox/imu
        ↓
Batch-LIO / local odometry
        ↓
/agt/livox/points
        ↓
manual/offline relocalization request
        ↓
3D-BBS -> local-map small_gicp
        ↓
/agt/relocalization/pose
        ↓
agt_localization_manager
        ↓
map -> odom -> base_link
```

Record at least:

- correct/failed/false-positive counts;
- XY, Z and yaw error against reference;
- BBS score;
- GICP fitness;
- overlap;
- BBS elapsed time and total wall time.

## Acceptance levels

### Level A — same-bag closed-set smoke

Goal: detect broken frames, bad point format, unsuitable search resolution and obvious false matches.

Suggested minimum before moving on:

- 0 false-positive poses in the sampled cases;
- high success rate across several distinct positions/headings;
- errors comfortably inside the navigation initialization tolerance;
- runtime acceptable on the target IPC.

### Level B — second rosbag / different traversal

This is the meaningful pre-field test. Build the map from bag A and query it using bag B from the same site, preferably with changed heading, location, people/vehicles and vegetation motion.

Tune on bag A; **validate, do not retune, on bag B**. If bag B fails, return to the candidate sweep with a training/validation split rather than weakening the production gates until everything passes.
