# agt_terrain_map_generator

Terrain-aware 2.5D navigation-map generation for `agt_navigation_v3`.

## Boundary

Input is a completed 3D localization/mapping asset such as `global_map.pcd`. This package must not sit in the FAST-LIO2 time-preserving sensor path and must not modify the localization PCD.

The pipeline is intentionally split into stable interfaces:

1. `PatchPreprocessor`: gravity-level body-frame patches.
2. `TerrainPreprocessor`: body-geometry self/rear rejection followed by voxel filtering.
3. `GroundSegmenter`: ground vs. non-ground separation. First target adapter: Patchwork++.
4. `ElevationBuilder`: robust local ground elevation, baseline target is median + confidence.
5. `SlopeBuilder`: slope derived from the ground/elevation surface.
6. `ObstacleBuilder`: non-ground obstacle evidence relative to local ground height.
7. `MapExporter`: Nav2 occupancy and terrain-layer export into an AGT Map Package.

## Migration policy

`agt_map_converter` remains the regression/fallback converter until the terrain-aware path passes rosbag/PCD acceptance tests. Do not delete or silently redirect the old converter during v0.x.

## Map-package outputs

Expected final outputs are:

- `localization/global_map.pcd` (copied/reference only; never terrain-filtered)
- `navigation/map.pgm`
- `navigation/map.yaml`
- `terrain/elevation.*`
- `terrain/slope.*`
- `terrain/obstacle.*`
- `metadata.yaml`

## Current v0.3.2 terrain representation status

`TerrainPreprocessor`, `SelfFilter`, `RearFilter`, static `RobotGeometryProvider`,
and voxel filtering are implemented for the offline patch path.  v0.3 adds a
geometry-aligned TraversabilityGrid, local 3x3 roughness scoring, trajectory
free-corridor carving from `poses.txt`, and a concrete terrain-package exporter.
The preprocessor keeps retained points in the gravity-level frame used by
Patchwork++, but converts each point back to body coordinates when evaluating
robot geometry.  None of these stages is part of the FAST-LIO2 or localization
input path.

`pipeline.enabled` remains `false` until PatchSetSource, the preprocessor,
segmentation, builders, trajectory stage, traversability stage, and exporter are
connected as one production map job and pass the offline acceptance suite.

### v0.3.1-a robot collision cleaning

`RobotGeometryLoader` reads rendered URDF collision geometry below `base_link`.
The first implementation supports only box and cylinder collisions and skips
meshes.  `RobotMeshFilter` removes points inside these volumes either in the
static lidar-to-base frame or, for a completed global map, against each
historical optimized `map_from_base` pose.  It is terrain/navigation map
cleaning only: it must never alter FAST-LIO input or `global_map.pcd` used for
localization.  The standalone `robot_mesh_filter_test` executable provides the
offline global-map validation path.

### v0.3.2 point provenance and persistence evidence

`PointProvenanceBuilder` reconstructs map-frame points from mapping patches and
their optimized poses, preserving intensity, patch id, pose id, and a monotonic
relative timestamp. The existing mapping assets have no absolute acquisition
timestamp, so this timestamp is explicitly the `poses.txt` sequence index.
`VoxelPersistenceFilter` groups these records at 0.10 m and labels a voxel
stable when it is observed by at least three different poses or patches. It
does not remove map points or feed Nav2/traversability. The standalone
`voxel_persistence_analyzer` writes stable and unstable voxel-centre PCDs plus
the aggregate YAML evidence for offline inspection.

### v0.3.2-b static confidence layer

`StaticConfidenceEvaluator` preserves every voxel and evaluates a continuous
score from its repeat-observation evidence and occupied 3x3x3 neighborhood
support. `surface_score` and `viewpoint_score` are present as 0.5 placeholders
only and are not used by the first formula. The `static_confidence_exporter`
generates high, medium, low-confidence, and score-intensity PCDs for RViz; low
confidence is evidence only, never a dynamic classification or deletion.

### v0.3.2-b.1 multi-scale confidence and ground prior

The compatible multi-scale exporter additionally maps each 0.10 m fine voxel
to 0.25 m structure and 0.50 m scene indices without copying raw points. Its
score uses configured temporal, fine, structure, scene, and ground weights.
An existing Patchwork++ `ground_map.pcd` may be supplied as a read-only prior:
matching voxels receive its confidence, while unknown/non-ground voxels use the
specified neutral 0.5 ground score. It does not trigger a new segmentation,
modify a point cloud, or classify low confidence as dynamic.
