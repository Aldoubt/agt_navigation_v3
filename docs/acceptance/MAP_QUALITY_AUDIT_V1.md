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

### MQ1.5 - Exact current-converter trigger attribution

Before changing thresholds or terrain algorithms, the fixed delta audit now
recomputes the current converter's **pre-carve** obstacle decision from the same
source PCD and frozen converter metadata.

For every selected delta cell it records:

- converter point count, min-z, max-z and vertical span;
- filled min-z elevation used by the current slope path;
- exact computed slope in degrees;
- `span_trigger`, `slope_trigger`, and one of
  `span_only / slope_only / both / neither`.

The report also emits global and per-region trigger counts plus
`delta_trigger_classes.pgm`.  A non-zero `neither` count is a consistency
REVIEW signal: the selected candidate occupied cell cannot be explained by the
current converter's recorded span/slope rule and must not be used to tune MQ2.

Stop MQ1 diagnosis after one frozen-field run confirms the trigger attribution.
Use that measured `span_only / slope_only / both` split to choose the first MQ2
algorithm change instead of tuning `max_step` or `max_slope_deg` blindly.

### MQ1.5 frozen-field result

The frozen v003 delta is now fully attributable to the current converter rule:

- span-only: 681 / 2,576 (26.4%);
- slope-only: 797 / 2,576 (30.9%);
- both span+slope: 1,098 / 2,576 (42.6%);
- neither: 0;
- exact attribution coverage: 100%.

All 797 slope-only cells have robust per-cell p05-p95 span <= the existing
0.22 m step threshold.  Therefore they are pure slope-path decisions rather
than hidden vertical-span decisions.  Slope participates in 1,895 / 2,576
selected cells (73.6%), while span participates in 1,779 / 2,576 (69.1%).

Several of the largest regions have median legacy computed slopes around
80-88 degrees while their source-PCD evidence contains either high-only returns
or mixed ground/high structure.  This closes MQ1 diagnosis: do not tune the
20-degree threshold upward to hide the symptom.

### MQ2-A - Conservative ground-confidence slope surface

The first MQ2 experiment changes only the **slope surface**, not the vertical
span rule and not the default production behavior.

New opt-in mode:

```text
--slope-surface-mode ground_confidence
--ground-radius-cells 2
--ground-height-tolerance-m 0.25
```

Algorithm:

```text
valid cell min-z
  -> local square-neighborhood low surface
  -> trust cell only when min-z is within tolerance of local low
  -> build/fill slope surface from trusted cells only
  -> apply slope threshold only to trusted cells
```

A valid but low-confidence cell is left **unknown** unless the unchanged
vertical-span rule already marks it occupied.  The experiment deliberately
does not infer free space beneath high-only/canopy returns.

The default remains `legacy_min_z`; field/mainline behavior cannot change until
the frozen-map A/B and static-obstacle review pass.  The converter also exports
`ground_confidence.pgm` plus metadata counters for trusted cells, low-confidence
valid cells, span triggers and slope triggers.

### MQ2-A frozen-map A/B result

The first `ground_confidence` run passed structural map validation and produced
the following legacy -> MQ2-A transitions:

- occupied -> unknown: 8,063;
- occupied -> free: 1,294;
- free -> occupied: 19;
- free -> unknown: 1,197;
- total changed cells: 10,573.

Final map totals changed from 34,131 free / 55,762 occupied / 975,482 unknown
to 34,209 free / 46,424 occupied / 984,742 unknown.  Occupied cells therefore
fell by 9,338 (16.7%) while free cells changed by only +78; most of the removed
occupancy was conservatively demoted to unknown instead of being released as
free.

Trajectory evidence moved in the same conservative direction:

- pre-carve occupied conflicts: 5,439 -> 4,188 (-1,251, -23.0%);
- pre-carve unknown conflicts: 6,407 -> 7,171 (+764);
- total trajectory conflicts: 11,846 -> 11,359 (-487, -4.1%);
- conflict ratio: 83.67% -> 80.23%.

Interpretation: MQ2-A successfully removes a substantial part of the false
slope occupancy without broadly increasing free space.  It is a useful
candidate, not yet a production map: the 1,294 occupied->free cells and 19
free->occupied cells still require focused A/B review, and the remaining
49,437 span-trigger cells show that the vertical-span path is now the dominant
next problem.

Metadata semantics note: the historical `valid_cells` field is actually the
final non-unknown cell count **after** trajectory carve.  It is retained for
compatibility.  New outputs also record `raw_valid_cells` (count>=min_points)
and `final_known_cells` explicitly.

### MQ2-A overlap review result

Comparison against the historical v003 free-space cleanup does **not** justify
promoting MQ2-A yet.

For the 2,576 historical `legacy occupied -> v003 free` cells, MQ2-A produces:

- occupied: 1,857 (72.1%);
- unknown: 263 (10.2%);
- free: 456 (17.7%).

For all 1,294 `legacy occupied -> MQ2-A free` cells:

- 456 overlap the historical v003 free-space cleanup;
- 838 (64.8%) are newly released beyond that historical cleanup.

Additional review transitions are:

- legacy free -> MQ2-A occupied: 19;
- legacy free -> MQ2-A unknown: 1,197.

Interpretation: MQ2-A is directionally useful for suppressing false slope
occupancy, but its newly-free set is not sufficiently corroborated by the
historical field cleanup.  The historical cleanup is not ground truth, so lack
of overlap is not an automatic failure; however, 838 new free cells are too
many to accept without source-PCD and static-obstacle review.

Gate decision: **MQ2-A remains REVIEW / experimental. Do not start MQ2-B by
stacking another obstacle-rule change on top of this candidate.** First audit
the 1,294 occupied->free cells, with priority on the 838 newly released cells,
and inspect the 19 free->occupied regressions.  The 1,197 free->unknown cells
are primarily a planning-usability regression and should be quantified by
region/corridor before promotion.

### MQ2-A released-cell review: flat-ground wins and floating-surface risk

The first focused review of the 1,294 `legacy occupied -> MQ2-A free` cells
shows that the largest release regions are dominated by **slope-only** legacy
decisions.  Typical cells have very small vertical span (millimeters to a few
centimeters), `near_ground_fraction=1.0`, zero low/tall/overhang fractions,
and legacy computed slopes around 80-88 degrees.  This strongly supports the
original diagnosis that the legacy min-z gradient path created false cliff
boundaries on otherwise locally flat returns.

However, the same review exposes a second MQ2-A failure mode: some small
released regions have similarly small vertical span and
`near_ground_fraction=1.0`, but their local ground reference itself sits at
high map z (examples around z ~= 2.38 m and z ~= 3.29 m).  A 0.2 m local
neighborhood can therefore self-consistently label a flat high-only surface as
"ground".  This may represent canopy/roof/vehicle-top structure and must not be
promoted to free solely from local consistency.

Gate consequence:

- keep the current MQ2-A result as evidence that slope-surface gating is useful;
- do **not** promote `ground_confidence` to production yet;
- before MQ2-B, add a multi-scale / anchored ground-support check so a locally
  flat but vertically floating surface becomes unknown rather than free;
- preserve the rule that insufficient ground evidence is unknown, not free.

### MQ2-A released-cell distribution result

The complete 1,294-cell release audit confirms that every
`legacy occupied -> MQ2-A free` cell is **slope-only** under the legacy rule.

Robust per-cell p05-p95 span:

- p50: 0.0146 m;
- p75: 0.0428 m;
- p90: 0.1132 m;
- p95: 0.1523 m;
- p99: 0.1928 m;
- all 1,294 cells are <= 0.22 m.

Legacy slope on the same cells is extreme:

- p50: 80.63 deg;
- p75: 86.47 deg;
- p90: 87.98 deg;
- p95: 88.36 deg;
- p99: 88.99 deg.

988 / 1,294 cells meet the strict very-flat-like evidence rule and
1,241 / 1,294 meet the broader ground-like evidence rule.  Only 438 / 1,294
are within the recorded trajectory expanded by +0.5 m.

The high-tail review proves the local-only self-consistency failure is real:
some released cells have local-ground estimates around z=9-14 m while still
showing small vertical span.  These cells are not evidence of navigable ground;
they are exactly the floating-surface case that MQ2-A.1 must reject.

### MQ2-A.1 - Trajectory-anchored ground connectivity

Add a second opt-in mode:

```text
--slope-surface-mode anchored_ground_confidence
--ground-radius-cells 2
--ground-height-tolerance-m 0.25
--ground-connect-max-slope-deg 45
```

The local MQ2-A ground candidates are unchanged.  MQ2-A.1 then keeps only
candidates that are connected to candidate cells intersecting the recorded
trajectory footprint.  Connectivity uses 8-neighbor propagation and requires
adjacent candidate elevations to remain continuous under the configured
permissive connection-slope bound.

This is an evidence anchor, not a navigation-slope threshold.  The final
20-degree obstacle slope rule is unchanged.  Candidate local ground that is not
trajectory-connected is demoted to unknown unless the unchanged vertical-span
rule marks it occupied.

New debug/metadata outputs:

- `ground_local_candidate.pgm`;
- `ground_anchor_seed.pgm`;
- `ground_confidence.pgm` (final anchored support);
- local candidate, anchor seed, anchored ground and floating-candidate counts.

The production default remains `legacy_min_z`.  MQ2-B remains blocked until
the frozen-map MQ2-A.1 A/B confirms that high floating surfaces are suppressed
without collapsing known traversable ground.

### MQ2-A.1 first frozen-map result: hard connectivity is too strict

The first trajectory-anchored run used only 8-neighbor cell-to-cell
connectivity.  It rejected floating surfaces, but it also collapsed too much
known ground:

- local ground candidates: 45,804;
- trajectory anchor seeds: 6,232;
- anchored ground: 16,986;
- disconnected/floating candidates: 28,818;
- low-confidence valid cells: 66,500.

MQ2-A -> MQ2-A.1 map transitions:

- occupied -> unknown: 994;
- occupied -> free: 4;
- free -> unknown: 15,070.

Final free space fell from 34,209 to 19,143 cells (-44.0%).  This is not an
acceptable planning baseline.  Importantly, trajectory pre-carve conflict
statistics were unchanged from MQ2-A (4,188 occupied + 7,171 unknown =
11,359), so the hard connectivity gate mostly removed off-corridor map
usability rather than improving corridor evidence.

Decision: **reject the radius-1 hard-connectivity result as too strict**.  Keep
the anchored idea, but model candidate ground as a sparse graph instead of
requiring contiguous 10 cm cells.

### MQ2-A.1b - Sparse candidate-graph connectivity

Refine the anchored mode with:

```text
--ground-connect-radius-cells 3
```

At 0.10 m resolution this permits a ground candidate to connect across up to
~0.3 m of sparse sampling/unknown cells, while the existing
`ground-connect-max-slope-deg` still limits allowed vertical change by metric
distance.  A disconnected high surface therefore cannot bridge a large
vertical jump merely because the XY graph radius is wider.

The goal is to recover sparse but continuous traversable ground without
re-admitting locally-flat floating surfaces.  This remains experimental; the
production default is unchanged and MQ2-B stays blocked until the frozen-map
A/B demonstrates materially better free-space retention than the radius-1
result.

### MQ2-A.1b frozen-map result

The sparse candidate-graph refinement materially recovers map usability while
retaining an anchored-ground filter.

Ground-support counters:

- local ground candidates: 45,804;
- anchor seeds: 6,232;
- anchored ground: 33,349;
- disconnected/floating candidates: 12,455;
- low-confidence valid cells: 50,137.

Compared with MQ2-A, the A.1b map transitions are:

- occupied -> unknown: 743;
- occupied -> free: 1;
- free -> unknown: 3,966.

Final free space is 30,244 cells versus 34,209 in MQ2-A (-11.6%), a large
improvement over the radius-1 hard-connectivity result (19,143 free).  The
trajectory conflict evidence is unchanged at 11,359 cells (4,188 occupied +
7,171 unknown), so A.1b is best interpreted as an **off-corridor floating
surface safety filter**, not a trajectory-quality improvement.

Decision: keep A.1b as the preferred experimental slope baseline.  It remains
more conservative than MQ2-A and is not yet production-default, but it retains
88.4% of MQ2-A free space while rejecting 12,455 locally-plausible candidates
that lack trajectory-connected ground support.

### MQ2-B - Ground-relative collision-band obstacle experiment

With the slope path isolated, add an opt-in obstacle rule that no longer marks
an entire cell occupied merely because `max_z-min_z > 0.22 m`.

New mode:

```text
--obstacle-mode ground_relative_band
--collision-band-min-height-m 0.15
--collision-band-max-height-m 1.50
--collision-band-min-points 2
--collision-band-min-fraction 0.20
```

This experiment requires a ground-confidence slope mode.  It uses the filled
trusted/anchored ground surface as a reference and counts source-PCD returns
inside a configurable height-above-ground collision band.

Decision logic:

- supported in-band returns -> occupied;
- one/sparse in-band return without enough support -> unknown;
- high-only returns above the collision band do not become occupied merely
  because vertical span is large;
- a cell can become free only when it has trusted/anchored ground support and
  no slope obstacle or ambiguous collision-band evidence.

The legacy vertical-span rule remains the production/default obstacle mode.
MQ2-B adds `collision_band.pgm` plus metadata counts for supported obstacles,
ambiguous cells and high-only/overhang evidence.  The 0.15-1.50 m band is an
engineering starting point for A/B, not yet a frozen vehicle collision envelope.

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

### MQ2-B frozen-map result and stop decision

The first ground-relative collision-band experiment is **too aggressive to
promote** as the fallback converter.

MQ2-A.1b -> MQ2-B map transitions:

- occupied -> unknown: 33,115;
- occupied -> free: 5,871;
- unknown -> occupied: 885;
- free -> occupied: 5;
- free -> unknown: 35.

Final map totals changed from:

- MQ2-A.1b: 30,244 free / 45,680 occupied / 989,451 unknown;
- MQ2-B: 36,075 free / 7,584 occupied / 1,021,716 unknown.

Occupied cells therefore fell by 38,096 (83.4%) while free cells rose by
5,831 (19.3%).  The trajectory conflict count improved only from 11,359 to
10,663 (-6.1%): occupied conflicts fell 4,188 -> 3,087, but unknown conflicts
rose 7,171 -> 7,576.

MQ2-B evidence counters:

- collision-band supported obstacle cells: 9,326;
- ambiguous collision-band cells: 2,514;
- high-only/overhang evidence cells: 17,018;
- legacy span-trigger cells on the same source remain 49,437.

Interpretation: collision-band evidence is useful, but the current direct
replacement of the legacy span rule removes far more occupancy than the
trajectory evidence justifies.  A large fraction of former occupied space is
demoted to unknown and 5,871 cells are newly released as free.  Without a
frozen labelled static-obstacle review set, this is too large a semantic change
for the stable fallback converter.

Decision:

- keep `legacy_span` as the default/fallback obstacle rule;
- keep MQ2-A.1b as the preferred experimental slope correction baseline;
- keep MQ2-B as evidence/prototype code only;
- **stop adding converter heuristics here** instead of tuning collision-band
  thresholds to force a desired map;
- move to MQ3 and compare against the terrain-aware pipeline, which already has
  Patchwork++, robust elevation, ground-relative obstacle, traversability and
  confidence interfaces.

### MQ3 implementation readiness note

The existing `agt_terrain_map_generator` has the component implementations
needed for the next A/B (patch source, preprocessing, Patchwork++ adapter,
median elevation, slope, height obstacle, trajectory carver, traversability,
exporter), but its ROS node is intentionally not yet a runnable production
pipeline.  `pipeline.enabled=true` currently throws because those components
are not fully wired as one map-generation job.

Therefore MQ3 must start by wiring a deterministic **offline one-shot terrain
generation job** on the frozen patch-set assets.  Do not enable the runtime node
or replace `agt_map_converter` yet.

Required first MQ3 inputs/gates:

1. frozen `patches/*.pcd + poses.txt` from the same mapping run;
2. measured Patchwork++ body-frame patch-origin ground height (the current
   config value 0.0 is intentionally invalid for activation);
3. the same 0.10 m comparison grid where practical;
4. the same trajectory evidence and selected review regions;
5. deterministic output package plus counts/hashes suitable for the existing
   map-quality A/B report.

### MQ3-P0 implementation: offline one-shot patch pipeline

A standalone `terrain_offline_generate` executable now wires the existing terrain components without enabling the ROS runtime node:

```text
PclPatchSetSource
  -> GravityLevelPatchPreprocessor
  -> TerrainPreprocessor
  -> Patchwork++ per patch
  -> transform ground/non-ground back to map
  -> MedianElevationBuilder
  -> CentralDifferenceSlopeBuilder
  -> HeightObstacleBuilder
  -> trajectory evidence
  -> TraversabilityBuilder
  -> TerrainPackageExporter
```

The exporter also writes Nav2 `map.pgm + map.yaml` from traversability state (blocked=occupied, free=free, otherwise unknown) and records free/occupied/unknown counts in metadata. Existing terrain debug layers are preserved.

Activation safeguards:

- the ROS `pipeline.enabled` default remains false;
- the localization/global PCD is never modified;
- the offline command requires an explicit measured `--sensor-height-m > 0`;
- if native Patchwork++ is unavailable at build time, the executable fails explicitly rather than substituting another algorithm;
- MQ3-P0 uses elevation-cell confidence only; static-confidence and persistence evidence remain separate until a later A/B proves they should participate.

MQ3-P0 is not accepted until the frozen patch-set job builds/runs and its output is compared against the converter baselines.

### MQ3-P0 build and asset gate result

The isolated `agt_terrain_map_generator` test run passed: 32 tests, 0 errors, 0 failures. The frozen field asset set contains 292 pose records and 292 patch PCDs, with no missing patch names reported by the poses-to-patches contract check.

Selected frozen MQ3 input:

`/home/yangxuan/agt_data/maps/bunker_mid360_mapping_20260901_205036_formal`

The remaining activation gate is Patchwork++ `sensor_height_m`. A reproducible evidence-only helper, `terrain_estimate_sensor_height`, now estimates the gravity-level body/patch-origin height from sampled patches. It does not modify assets and its result must still be checked against physical sensor/body geometry before the first accepted Patchwork++ run.

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
