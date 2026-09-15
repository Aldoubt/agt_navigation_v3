# MQ3-A Patchwork terrain segmentation design

## Scope and boundary

MQ3-A adds a read-only, PCD-only offline experiment.  It reads a frozen
`global_map.pcd` and writes a sibling result directory; it neither consumes
`sensor_msgs/PointCloud2` nor changes the PCD, MQ2-B converter, navigation map
contract, or any active map package.

`agt_map_converter` remains the MQ2-B authority: it projects the final PCD to
`navigation/map.yaml`, `navigation/map.pgm`, and terrain debug layers using its
anchored-ground-confidence and ground-relative-band logic.  MQ3-A does not
call, replace, or redirect that converter.  Its optional benchmark merely
reads the existing MQ2 `map.pgm` for counts.

## Existing terrain generator review

The existing `terrain_offline_generate` is a separate patch-set pipeline.  It
uses `PclPatchSetSource` to load `poses.txt` plus `patches/*.pcd`, gravity-levels
each body-frame patch, applies robot geometry/voxel preprocessing, optionally
calls the native Patchwork++ plugin, then maps the segmented points with the
shared builders.  It exports a full `terrain_package/` with Nav2 `map.pgm` and
`map.yaml`, terrain PGM layers, and `metadata.yaml` containing terrain version,
grid geometry, source, parameters, and free/occupied/unknown counts.

Elevation is the per-cell median of ground Z samples with count/variance
confidence.  Slope is a central-difference gradient over confident elevation
cells.  These builders are reused by MQ3-A only to produce its diagnostic
`elevation.pgm` and `slope.pgm`; MQ3-A does not emit a Nav2 map.

## MQ3-A interface and outputs

`PatchworkInterface` is the only segmentation boundary.  Its public signature
uses PCL XYZ clouds and returns explicit ground and non-ground clouds.  No
third-party Patchwork headers appear in that interface.  The current
`PatchworkAdapter` is a deterministic cell-lowest-surface implementation for
offline contract and test validation, not a claim of native Patchwork++
equivalence.  A future native Patchwork++, alternate Patchwork++, or
Patchwork-LIO implementation can replace it behind the same interface.

The executable is:

```bash
ros2 run agt_terrain_map_generator terrain_patchwork_offline \
  --input-pcd /path/global_map.pcd \
  --output-dir /path/mq3_patchwork_result \
  --config /path/patchwork.yaml \
  --mq2-dir /path/navigation
```

`--mq2-dir` is optional.  If omitted, it looks beside a conventional
`localization/global_map.pcd` for `navigation/map.pgm`; unavailable MQ2 values
are written as `null`, never invented.

The output directory contains `ground.pcd`, `nonground.pcd`, `elevation.pgm`,
`slope.pgm`, `terrain_metadata.yaml`, and `mq3_vs_mq2_report.yaml`.
Metadata uses format `mq3-patchwork-v1` and records the absolute input path,
configured sensor height, counts/ratios, resolution, UTC timestamp, and
`patchwork_status`.  Successful mixed segmentation is `PASS`; successful
all-ground segmentation is `REVIEW`; unreadable/empty/non-finite input exits
with `FAIL` rather than generating an ambiguous artifact.

The comparison report is descriptive only.  MQ3 occupied/free/unknown cells
are its temporary segmentation grid (`nonground` takes occupied precedence,
then `ground` free, otherwise unknown).  MQ3 trajectory-conflict count is not
available because this PCD-only phase accepts no trajectory input; it is
consequently written as `null`.  MQ2's value is copied from its existing metadata when
present.  This is a reported input limitation, not an acceptance judgement.
