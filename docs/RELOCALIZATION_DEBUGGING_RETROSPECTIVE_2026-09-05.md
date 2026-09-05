# Relocalization Debugging Retrospective — 2026-09-05

## Purpose

This document records the engineering lessons from the first successful
end-to-end offline automatic relocalization gate in `ros2_ws`.

It is written as a reusable debugging guide, not just a history of one bug. The
main lesson is to isolate **data quality, local odometry, frame semantics,
candidate retrieval, geometric registration, state-machine logic and TF
ownership** instead of tuning all of them at once.

## Starting symptom

The observed symptom was simple:

> automatic relocalization repeatedly failed and it was unclear whether the
> cause was Batch-LIO drift, rosbag quality, the global map, BBS convergence,
> TF, or the localization state machine.

A dangerous early assumption would have been:

```text
relocalization failed
    -> Batch-LIO must be drifting
    -> tune IMU/LIO parameters
```

That assumption was not supported by the final evidence.

## Debugging method used

The useful order was:

```text
1. Verify data exists and timestamps/units are plausible.
2. Verify local LIO publishes independently of relocalization.
3. Verify automatic request/retry/stationary state logic.
4. Bypass ROS and test the native registration backend directly.
5. Test registration on known-truth map keyframes.
6. Match map provenance to the correct rosbag.
7. Make frame semantics explicit.
8. Only then tune search space / timeout / thresholds.
9. Return to full ROS replay and verify final map->odom ownership.
```

This sequence prevented one subsystem from hiding another subsystem's failure.

## What was ruled out first

### MID360 IMU unit mismatch

The Batch-LIO config uses `acc_norm: 1.0`, which looked suspicious until the bag
was sampled. Static acceleration norm was approximately `0.992 g`, so the
MID360 IMU stream in this bag is in g-style units. `acc_norm: 1.0` is therefore
appropriate for this data.

**Lesson:** do not change an IMU scale parameter because the number looks
unusual. Measure the actual message first.

### “Batch-LIO drift makes BBS impossible”

The global query is constructed from LiDAR data and the fixed sensor/base
transform. It is not transformed through the accumulated Batch-LIO trajectory
before global search.

Batch-LIO still matters for stationary detection, local odometry availability
and the `map -> odom` handoff, but drift cannot directly move the raw global
query out of the map.

**Lesson:** draw the actual data-flow graph before blaming an upstream estimator.

## State-machine bugs found

### Zero twist caused false stationary detection

Batch-LIO `/aft_mapped_to_init` publishes zero twist. The original stationary
gate could therefore classify a moving robot as stationary. Motion is now also
estimated from consecutive local poses.

**Lesson:** a populated ROS field is not necessarily a meaningful measurement.

### Automatic request did not retry cleanly

After one failed automatic request, the old request/query state could remain
latched. Failure handling now permits a fresh stationary retry.

**Lesson:** every autonomous state needs an explicit recovery path.

### Local odometry buffer was shorter than backend latency

The manager originally buffered about 5 s of local odometry while global search
could take around 10 s. A valid global result could arrive after its aligned
local sample had already been discarded. The buffer is now 30 s.

**Lesson:** timestamp-alignment buffers must be sized from worst-case pipeline
latency, not nominal callback rate.

## Map provenance mattered

The older `site_test` mapping log contained out-of-order LiDAR messages,
repeated `NO Effective Points!`, and an LIO process crash. It was not accepted as
the benchmark map.

`bag_mapping_current` had a clean mapping run. Its build duration/history matched
the `211105` bag replay at 0.5x, so `211105` was used for the final matched-map
acceptance replay.

**Lesson:** a `.pcd` file existing on disk is not proof that it is a valid map.

Future map packages should store source bag, commit ids, parameters, calibration
hash and asset-generation version so provenance never has to be inferred from
shell history again.

## Why whole-map CPU BBS failed

The first implementation used CPU 3D-BBS over the complete outdoor map with a
large translation and angular search space. Full-map search repeatedly timed
out, while a small known search window succeeded quickly.

Trees, ground and repeated structures keep many coarse hypotheses plausible, so
branch-and-bound pruning is weak.

**Lesson:** a global registration algorithm can be correct but still be the
wrong place-recognition architecture for the scale and repetition of a scene.

## Restoring the missing candidate-retrieval layer

The mapping pipeline already saved `patches/*.pcd` and `poses.txt`; the first
relocalization implementation did not use them.

An AGT-owned Polar Context database now performs:

```text
query
  -> ring-key place prefilter
  -> sector-key yaw alignment
  -> Top-K keyframe candidates
  -> candidate-local geometry
```

This turns BBS from a whole-map global searcher into a local coarse-registration
stage.

**Lesson:** mapping keyframes are a natural relocalization index, not merely a
debug artifact.

## Known-truth tests were essential

Before trusting live rosbag results, a saved keyframe patch was converted into a
known-truth query. Tests at 0°, 90° and 93° verified candidate retrieval, yaw
interpretation, BBS behavior, GICP refinement and descriptor angle quantization.

**Lesson:** when an end-to-end perception system fails, create an input with a
known answer before tuning the live system.

## Frame semantics: `body` was not `base_link`

FAST-LIO/Batch-LIO keyframe patches are in an IMU/LIO `body` frame. The live
query is in robot `base_link`. Treating them as identical is wrong because the
MID360 installation is tilted and the internal LiDAR/IMU extrinsic is nonzero.

The relation was composed from the measured `base_link -> lidar_link` mounting
pose and the pinned Batch-LIO LiDAR/IMU extrinsic. Current values are recorded in
`docs/TF_CONVENTION.md`.

**Lesson:** frame names are semantic contracts. Never insert an identity
transform simply because two frames belong to one sensor assembly.

## Why the adapter stopped depending on TF lookup

During `/clock` replay, per-message TF lookup introduced failures even though
`body -> base_link` is a fixed calibration. The adapter now stores and uses the
calibrated transform directly. A static TF is still published for graph
consumers.

**Lesson:** static calibration should be a configuration dependency, not
necessarily a runtime communication dependency.

## BBS threshold and timeout lesson

A higher BBS score threshold often made BnB search slower and caused timeout
before a useful coarse pose was returned. The final division of responsibility
is:

```text
Polar Context     place/yaw hypothesis
BBS               coarse geometric seed
GICP              precise 6-DoF registration
ROS quality gates final acceptance/rejection
```

The candidate BBS threshold is intentionally low (`0.05`). This does **not**
mean final acceptance is weak; GICP and ROS quality gates remain strict.

**Lesson:** tune a threshold according to the responsibility of that stage.

## Final V1 CPU baseline

```text
candidate descriptor      AGT Polar Context
descriptor prefilter      40
candidate Top-K           2
BBS minimum level         0.5 m
BBS levels                5
candidate XY search       +/- 4 m
candidate Z search        +/- 2 m
BBS angular residual      0 deg
per-candidate timeout     8 s
BBS threshold             0.05
backend timeout           18 s
odom alignment buffer     30 s
```

## Final offline evidence

```text
map     agt_data/maps/bag_mapping_current
bag     bunker_mid360_mapping_20260901_211105
```

Observed sequence:

```text
QUERY_READY
-> BBS_SEARCHING
-> BBS_COARSE_FOUND
-> GICP_REFINING
-> SUCCEEDED
-> global_pose_accepted
-> LOCALIZED
```

Accepted metrics:

```text
score                   0.874344
fitness                 0.128999
overlap                 0.973186
position_std_m          0.231676
yaw_std_deg             4.507872
candidate_patch         35.pcd
local_odom_fresh        true
global_correction_valid true
```

`map -> odom` was observed continuously after acceptance.

## Gazebo navigation exposed a second class of bugs

The relocalization chain could be correct while Nav2 still failed to move the
robot reliably. The first symptom was a valid global path followed by an RPP
command that stayed near one angular-acceleration increment.

### Batch-LIO zero twist also affected the controller

Batch-LIO publishes zero twist on `/aft_mapped_to_init`. RPP uses odometry twist
for rotate-to-heading acceleration limiting. With a 50 Hz controller and an
internal `0.8 rad/s^2` limit, a zero measured angular velocity repeatedly
produced only:

```text
0.8 rad/s^2 * 0.02 s = 0.016 rad/s
```

The adapter now derives child-frame linear and angular velocity from consecutive
adapted poses, using quaternion delta for angular velocity plus dt/deadband/outlier
guards.

**Lesson:** odometry pose can be excellent enough for localization while an
untrustworthy twist field independently breaks a downstream controller.

### Separate algorithmic bootstrap limits from physical safety limits

Even after pose-derived twist was available, the 10 Hz LIO update rate meant a
very low initial RPP turn could remain poorly observable. The validated Gazebo
baseline therefore uses:

```text
RPP internal max_angular_accel     10.0 rad/s^2
velocity smoother physical limit   0.8 rad/s^2
cmd_vel_guard physical limit       0.8 rad/s^2
```

The large RPP value does not authorize the chassis to accelerate at 10 rad/s^2;
it prevents the high-level controller from repeatedly re-applying only its first
acceleration step. Downstream layers retain the physical limit.

**Lesson:** do not duplicate the same low slew limit at several layers when the
upstream layer depends on lower-rate measured velocity.

### Final rotation must count as navigation progress

`SimpleProgressChecker` only considers translation. Near a goal, a tracked robot
may correctly spend several seconds rotating in place, which the simple checker
can misclassify as “no progress”. Humble already provides
`nav2_controller::PoseProgressChecker`; the baseline now uses it with a
`0.10 rad` movement-angle threshold.

**Lesson:** progress semantics must match the robot's valid motion primitives.

## Test infrastructure can perturb the system

Two test-harness problems materially distorted early Gazebo results.

First, many previous background launches had been terminated by killing only
their parent process. Hundreds of child ROS processes from old test domains were
still alive. Before the clean acceptance run, 230 explicitly identified old
test processes were removed and the new stack was isolated in ROS domain 148.

Second, `ros2 action send_goal --feedback` printed the full navigation feedback
at high rate and consumed about one quarter of a CPU core in this test. The
formal acceptance switched to a minimal `rclpy` ActionClient with no feedback
callback and only a final result.

**Lesson:** benchmark the system, not the benchmark tool. Use a unique ROS domain,
verify there is exactly one stack, and keep instrumentation lower-rate than the
control loop unless high-rate tracing is the thing being measured.

The clean result was:

```text
cold-start relocalization    SUCCEEDED
candidate BBS elapsed        453.279 ms
NavigateToPose               SUCCEEDED
navigation elapsed           45.322 s
```

The simulator still reported occasional missed 50 Hz control cycles under the
desktop/remote-display load. That remains a scheduling-performance observation,
not a reason to reduce the real-vehicle 50 Hz controller target.

## Fail-closed motion gate was tested as a state transition

Reading the guard implementation was not considered sufficient. An isolated ROS
pub/sub acceptance sequence continuously injected non-zero velocity while
forcing `LocalizationStatus.STATE_LOST`, then restored localization without a
new command.

Measured evidence:

```text
LOST -> first zero output     1.4 ms
peak after 80 ms LOST        0.000000
peak after recovery/no cmd   0.000000
fresh post-recovery command  accepted
```

This proves the important semantic distinction:

```text
reopen localization gate != replay old motion intent
```

**Lesson:** safety-state recovery must require fresh intent, not merely a return
to a healthy state.

## What this acceptance does not prove

This replay proves the software chain can work end to end. It does not yet prove
field robustness. Still required:

1. at least five materially different start positions;
2. multiple headings at the same positions;
3. repetitive tree rows and low-feature areas;
4. false-positive measurement, not only success rate;
5. recovery after meaningful accumulated local-LIO drift;
6. rough tracked-chassis vibration;
7. software restart and complete power cycle;
8. controlled real-vehicle validation of the `cmd_vel_guard` localization gate
   and the pose-delta-based mission stop gate. Their software/bench fixes were
   completed later on 2026-09-05; see `docs/RVIZ_FIELD_ACCEPTANCE.md`.

## Reusable checklist

```text
[ ] Is this the correct map for this bag/site/version?
[ ] Is the map-generation log clean?
[ ] Are IMU units and timestamps measured, not assumed?
[ ] Does local LIO publish valid pose independently?
[ ] Are frame names and transform directions explicitly written down?
[ ] Is the robot truly stationary according to pose delta?
[ ] Can a known-truth patch localize offline?
[ ] Does descriptor Top-K contain the correct place?
[ ] Can the candidate localizer solve a fixed captured query?
[ ] Are timeout/buffer durations compatible?
[ ] Is the final pose checked by independent GICP quality gates?
[ ] Is there exactly one map->odom owner?
[ ] Does the ROS state actually become LOCALIZED?
```

The order matters. It keeps debugging evidence-driven and prevents parameter
tuning from hiding data, frame or state-machine defects.
