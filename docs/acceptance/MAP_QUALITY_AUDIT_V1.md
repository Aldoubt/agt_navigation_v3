# Map Quality Audit V1

Status: active development scope for `fix/map-quality`.

## Objective

Improve the Nav2 navigation-map quality produced from the existing 3D mapping
assets without changing the localization contract that was validated on
`fix/lidar-mount-yaw`.

The branch addresses three concrete failure classes:

1. dynamic/temporary clutter or LIO smear that is already present in the source
   PCD;
2. projection artifacts introduced while converting 3D structure to a 2D Nav2
   occupancy grid, especially vegetation, overhangs, slopes and sparse vertical
   returns;
3. excessive or misplaced unknown/occupied regions that reduce planning
   usability.

The branch does **not** change controller tuning, MPPI, relocalization scoring,
or the MID360 mount calibration.

## Branch policy

Keep only one long-lived baseline and one active development branch:

```text
main
  -> validated runnable baseline

fix/map-quality
  -> active map-quality work
  -> merge back to main when the gates below pass
```

Do not create parallel feature branches for individual experiments.  Record
experiments as reproducible commands/artifacts under this branch and either keep
or revert their commits.

## Current production boundary

The current production/fallback navigation-map path remains:

```text
final localization/global_map.pcd
        |
        v
agt_map_converter
  XY cell count/min_z/max_z
  local fill + slope
  vertical-span/slope obstacle rule
  optional trajectory swept-footprint carve
        |
        v
navigation/map.pgm + map.yaml
        |
        v
agt_map_manager package/validation
        |
        v
Nav2
```

`agt_terrain_map_generator` already contains terrain-aware building blocks,
PatchSet/Patchwork++ integration points, provenance/static-confidence work and
2.5D builders, but it remains an offline candidate.  It must not silently
replace `agt_map_converter` until an A/B acceptance run demonstrates a clear
improvement on the same frozen assets.

Localization must continue to consume the frozen 3D localization map.  A
navigation-only fix must never rewrite `localization/global_map.pcd`.

## Known baseline observations

The existing v003 audit already established:

- the PGM/YAML structure and PCD-to-PGM provenance are readable;
- the map contains a very large unknown background plus internal unknown areas;
- the frozen trajectory-footprint audit reported 13,245 swept cells:
  13,229 free, 11 unknown and 5 occupied;
- those five occupied cells form three review regions;
- visible thin black structures cannot be labelled dynamic solely from the PGM.

These observations are the baseline.  They are not permission to auto-delete
occupied cells.

## Fault isolation rule

Every visible map defect must first be classified into one of these layers:

```text
A. source PCD defect
   ghost vehicle / person
   LIO double edge / motion smear
   self return
   inconsistent accumulated geometry

B. 3D -> 2D projection defect
   canopy/overhang projected as wall
   slope interpreted as obstacle
   min_z/max_z span sensitive to outliers
   sparse vertical return produces black line

C. navigation semantics defect
   unknown policy
   footprint/trajectory carve
   occupancy threshold / resolution

D. Nav2 costmap/runtime defect
   inflation
   obstacle/voxel layer
   footprint
```

Do not fix a D problem by editing the PCD, and do not fix an A problem by
loosening Nav2 inflation.

## MQ0 - Reproduce and freeze the current baseline

Before changing algorithms:

1. regenerate the current map from the exact frozen source PCD and converter
   parameters;
2. save converter metadata, validation output and hashes;
3. record occupied/free/unknown counts and connected-component statistics;
4. run the trajectory-corridor conflict audit;
5. save a visual PGM plus the existing elevation/slope/obstacle debug layers.

Acceptance:

- regeneration is deterministic;
- output provenance points to the exact input PCD;
- no source map package is modified in place.

### MQ0 first reproduction result (2026-09-15)

Using the frozen v003 converter parameters and the recorded v002-edited
localization PCD, the regenerated output validated structurally but did **not**
reproduce the existing v003 navigation map byte-for-byte.

Observed reproduction:

- grid: 947 x 1125 at 0.10 m;
- free: 34,131 cells;
- occupied: 55,762 cells;
- unknown: 975,482 cells;
- trajectory conflicts before carve: 11,846 / ~14,158 swept cells
  (~83.67%);
- those conflicts contain 5,439 occupied and 6,407 unknown cells across
  7 regions;
- `validate_nav_map`: PASS;
- acceptance gate: REVIEW.

Byte comparison against the existing v003 navigation directory:

- `map.yaml`: SAME;
- `elevation.pgm`: SAME;
- `slope.pgm`: SAME;
- `obstacle.pgm`: SAME;
- `map.pgm`: DIFF;
- `converter_metadata.yaml`: DIFF.

The existing v003 report records 36,707 free, 53,186 occupied and 975,482
unknown cells.  Unknown count is therefore unchanged while exactly 2,576 cells
move between occupied/free in the regenerated final navigation map.

The follow-up pixel-level comparison closed this ambiguity:

- existing v003 `map.pgm` -> MQ0 regenerated `map.pgm`: exactly 2,576
  `free -> occupied` cells;
- existing v003 `obstacle.pgm` -> existing v003 `map.pgm`: exactly 2,576
  `occupied -> free` cells;
- MQ0 regenerated `obstacle.pgm` -> MQ0 regenerated `map.pgm`: 0 cells;
- the v003 metadata does not contain a recorded manual patch history; its only
  metadata delta versus the current rerun is the newer trajectory-conflict QA
  fields.

Therefore the source-PCD projection is reproducible and the historical v003
navigation map contains a 2,576-cell free-space modification that is not
represented by the current converter provenance.  The evidence proves a
post-projection/final-map divergence, but it does **not** by itself prove
whether the historical cause was manual editing, an older tool behavior, or
another unrecorded post-process.

Do not change converter thresholds until those 2,576 cells are grouped into
map-frame regions and checked against the source PCD/trajectory evidence.

## MQ1 - Source-vs-projection diagnosis

For each major black-line/ghost review region, collect local evidence from the
source 3D points:

- point count/density;
- min/max/median/percentile height;
- vertical span;
- local ground estimate;
- height above ground;
- trajectory intersection;
- connected component geometry.

The goal is to answer:

```text
Is this structure already wrong in 3D?
or
Is a reasonable 3D structure being projected badly into 2D?
```

No automatic dynamic-object deletion is allowed in MQ1.

Acceptance:

- every selected defect region has a reproducible classification/evidence
  record;
- the branch can point to at least one source-PCD defect and/or one projection
  defect before algorithm changes begin.

### MQ1 first historical-delta analysis (2026-09-15)

The 2,576 historical `occupied -> free` cells were compared against the
current recorded swept footprint and the frozen source PCD.

Spatial relationship to the current swept footprint:

- current swept footprint: 0 / 2,576 cells;
- +0.10 m expansion: 757 / 2,576 (29.4%);
- +0.20 m expansion: 1,365 / 2,576 (53.0%);
- +0.30 m expansion: 1,778 / 2,576 (69.0%);
- +0.40 m expansion: 2,042 / 2,576 (79.3%);
- +0.50 m expansion: 2,196 / 2,576 (85.2%).

The delta is highly fragmented: 488 connected regions for 2,576 cells, with
the largest region only 79 cells (0.79 m^2).  This makes a single large parked
vehicle or one contiguous manual erase unlikely as a complete explanation.
The strong concentration immediately outside the recorded swept footprint is
consistent with a historical clearance halo or path-adjacent cleanup, but that
cause is not yet proven.

Source-PCD statistics for the largest regions show two recurring signatures:

1. tall mixed-height returns: several regions contain ground-level returns near
   z ~= -1.2 m together with strong returns at z ~= 3-5.5 m, consistent with
   canopy/overhang or tall vegetation being projected into the same XY cells;
2. lower mixed-height returns: many regions contain low returns near z ~= -1.0
   m and substantial returns around z ~= 0-0.8 m, producing robust vertical
   spans around 1.6-1.8 m.

These are not sparse single-point outliers: the p05-p95 span remains large and
typical point density is roughly 3-7 points/cell in the largest regions.

Important limitation: these first statistics pool all PCD points across each
connected region.  Region-level min/max/percentiles can mix neighboring ground
and obstacle cells.  Before changing converter rules, MQ1 must compute
**per-cell** vertical profiles and a local-ground-relative height distribution,
then aggregate those per-cell features by region.  This is required to
distinguish canopy/overhang from ground-connected obstacles and slopes.

### MQ1 fixed audit command

The temporary analysis is now fixed as an installed, evidence-only command:

\`\`\`bash
ros2 run agt_map_converter analyze_map_delta \
  --reference "$REF" \
  --candidate "$OUT" \
  --source-pcd "$PCD" \
  --trajectory-poses "$POSES" \
  --output ~/ros2_ws/agt_data/map_quality/mq1_delta_analysis
\`\`\`

Default selected transition is \`candidate occupied -> reference free\`.
The command requires reference/candidate map geometry to match and writes:

- \`map_delta_analysis.yaml\`: hashes, transition matrix, trajectory-expansion
  coverage and top region summaries;
- \`delta_cells.yaml/csv\`: per-cell point count, z profile, local-ground
  estimate, height-above-ground distribution and trajectory proximity;
- \`delta_regions.yaml/csv\`: aggregated region evidence;
- \`delta_mask.pgm\`: selected occupancy delta;
- \`delta_vs_trajectory.pgm\`: selected delta versus the recorded swept
  footprint.

The local-ground estimate is diagnostic, not a classifier: it uses the
configured percentile over a small XY neighborhood.  The tool deliberately
does not label a region as dynamic, canopy, vegetation, slope or a true
obstacle.

## MQ2 - Robust fallback converter

Improve `agt_map_converter` conservatively before replacing it.

Candidate changes, in order:

1. replace pure `min_z` ground representation with robust local
   median/percentile statistics;
2. evaluate obstacles by height above a local ground surface rather than only
   absolute cell vertical span;
3. add support/density confidence so one sparse high return does not create a
   long blocking wall;
4. distinguish overhang/canopy evidence from ground-connected blocking
   structure where the data supports it;
5. preserve unknown for low-confidence cells;
6. keep trajectory carve as explicit evidence-based clearing, with all cleared
   conflicts recorded.

Each change must have a synthetic unit test and an A/B result on the frozen
field map.

Acceptance:

- no regression on deterministic map export/validation;
- trajectory-corridor conflicts do not increase;
- occupied artifacts decrease only where 3D evidence supports the change;
- true static obstacles used in the review set remain occupied.

## MQ3 - Terrain-generator A/B candidate

Run `agt_terrain_map_generator` on the same frozen mapping assets only after
MQ2 has a stable baseline.

Preferred input when available:

```text
patches/*.pcd + poses.txt
  -> patch-local ground segmentation
  -> T_map_body aggregation
  -> robust elevation/slope/obstacle layers
```

Compare against the fallback converter with the same:

- grid bounds/resolution where practical;
- trajectory review corridor;
- selected static-obstacle regions;
- selected canopy/vegetation regions;
- selected dynamic/ghost regions.

The terrain generator remains disabled in production unless it clearly wins the
A/B and completes a deterministic package-generation job.

## MQ4 - Navigation acceptance

A candidate navigation map passes only when:

1. `validate_nav_map` passes;
2. Map Manager can package/validate/reseal it without weakening integrity
   checks;
3. offline global relocalization continues to use the matching localization
   assets;
4. `map -> odom -> base_link` remains valid;
5. Nav2 global planning succeeds through known traversed corridors;
6. the robot footprint does not intersect newly introduced false obstacles;
7. review images show no obvious deletion of true walls/poles/terrain hazards.

Controller tuning is explicitly outside this gate.  RPP/MPPI A/B starts only
after this branch produces a trustworthy map.

## Deliverables

The branch should finish with:

- one frozen baseline report;
- region-level source-vs-projection evidence;
- converter A/B metrics;
- synthetic regression tests for each accepted converter change;
- one candidate navigation map package;
- an updated map-quality acceptance report;
- no change to the validated localization PCD unless a separate, explicitly
  versioned source-PCD cleanup is justified.

## Stop conditions

Stop and reclassify instead of continuing to tune thresholds when:

- raw PCD visibly contains double walls/smear caused by mapping/LIO;
- a proposed converter threshold fixes one region but deletes known static
  obstacles elsewhere;
- the candidate depends on manual undocumented PGM edits;
- the terrain-generator path cannot reproduce a deterministic package;
- Nav2 runtime costmap artifacts are being mistaken for offline map defects.
