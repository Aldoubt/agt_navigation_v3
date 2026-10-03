# Local row perception (research opt-in)

The same C++17 core runs online and offline. It publishes observations and an odom
centerline; it never publishes a velocity command or navigation TF correction.
The default V1 launch is unchanged. All supplied thresholds are development
values, not accepted vehicle limits or calibrated probabilities.

## Data and frames

Input is the separate `/agt/livox/points` branch. `/livox/lidar -> LIO` is never
filtered or changed. Raw input must contain `timestamp` (absolute seconds) or
`offset_time` (nanoseconds from header.stamp). `input_deskewed=true` is only for
a frontend output already compensated to header time; in that mode old offset
fields are ignored to avoid a second deskew.

Odometry poses refer to `geometry.frame_id` through the actual child-to-body TF.
Each point is transformed at its own acquisition time using interpolated odom
translation and SLERP rotation. The carrier is gravity horizontal with the
body origin and its yaw at the reference timestamp. The node publishes
`odom -> local_row_carrier` for diagnostics; it does not own either navigation
TF edge. Point timestamps after the final available pose wait briefly for
odometry; missing poses, old stamps, future stamps, and expired quality deny
row authorization. Epoch changes clear poses, BEV history and prior fits.

Online fixed sensor extrinsics come from TF. The offline CSV pose frame must
explicitly match the geometry frame and needs explicit sensor extrinsics.
Handheld recordings are a scanner carrier, not a calibrated vehicle; configure
their ground height envelope separately and report no robot closed-loop claim.

## Core pipeline

1. Classify box/cylinder self returns and body-blocked rays at acquisition time.
2. Transform valid returns into the reference horizontal carrier with per-point
   motion compensation; retain a finite, epoch-bound frame window.
3. Fit a gravity-constrained ground plane from low representatives in distinct
   XY cells, checking spatial span, residual and configured height envelope.
4. Build multi-height BEV support with distinct frame IDs and elapsed-time decay.
5. Fit both row sides with a shared direction using RANSAC and Huber IRLS.
   Missing sides, short span, rank deficiency and poor normalized Hessian
   conditioning invalidate the fit.
6. Propagate approximate fitted parameter covariance to lateral/heading error;
   combine uncertainty, spatial support, independent-frame persistence and
   transported temporal consistency into a quality score.
7. Publish conservative visibility: free requires recent ground and a real ray
   through the configured low obstacle band; high rays and absent returns do
   not certify low free space. Occupied, body shadow and stale evidence remain
   explicit. Check the union of current-forward and centerline swept envelopes
   for a contiguous stopping corridor.

For `x` forward and `y` left, `y_L=a*x+b_L`, `y_R=a*x+b_R`, `c=(b_L+b_R)/2`:

```text
robot lateral error = -c/sqrt(1+a*a)
robot heading error = -atan(a)
width = (b_L-b_R)/sqrt(1+a*a)
```

Positive lateral error means the carrier is left of the row. Fitted covariance
is an approximation from spatial cells, not a validated posterior. Correlated
vegetation, ground errors and calibration errors require empirical coverage
checks. The quality score is not a probability.

## Online

```bash
ros2 launch agt_local_row_perception local_row_perception.launch.py
```

Subscriptions: cloud, `/agt/odometry/local`, `/agt/odometry/quality`, TF.

| Topic | Meaning |
| --- | --- |
| `/agt/local_row/state` | `LocalRowState` with valid bits, covariance, quality factors, support and reasons |
| `/agt/local_row/path` | centerline in odom at its acquisition timestamp; empty when invalid |
| `/agt/local_row/visibility` | occupancy view: free=0, occupied/body=100, everything unproven=-1 |
| `/agt/local_row/markers` | body, occupied, occluded, unknown and stale classification colors |
| `/agt/livox/points_self_filtered` | secondary cloud retaining every original field and point timing |
| `/diagnostics` | support, age/epoch failure reasons, self counts and observed corridor |

The filtered cloud retains its sensor frame and source header, so its sensor
origin is preserved. It is suitable for research consumers; it must not replace
the raw LIO input. The current self model is static; dynamic gimbals or payloads
require an actual joint-state model or a conservative driving envelope.
It is published after timestamp/TF validation and a complete odometry bracket
for all retained acquisition times; configure `filtered_cloud_topic` if needed.
Missing TF or pose brackets publish an invalid row diagnostic and no replacement
cloud. A steady clock watchdog also invalidates the observer if cloud delivery
stops while the ROS clock is paused. Epoch values are identities, not counters.

| Node parameter / launch argument | Default |
| --- | --- |
| `config_file` | installed `config/row_perception.yaml` in the launch |
| `geometry_file` | installed diagnostic collision proxy in the launch |
| `field_verified`, `input_deskewed`, `use_sim_time` | `false` |
| `cloud_topic` | `/agt/livox/points` |
| `odom_topic` | `/agt/odometry/local` |
| `odom_quality_topic` | `/agt/odometry/quality` |
| `filtered_cloud_topic` | `/agt/livox/points_self_filtered` |
| `odom_frame`, `carrier_frame` | `odom`, `local_row_carrier` |
| `max_input_age_sec`, `max_future_sec`, `pending_wait_sec` | `0.5`, `0.05`, `0.20` |

The C++ node accepts empty config paths as development defaults; use explicit
files for recorded experiments. Algorithm settings are under `row_perception`
in `config_file`, while body shapes and measured vehicle properties are under
`geometry` in `geometry_file`. Input and filtered topics must be distinct.

`clearance_valid` requires **both** a field-verified geometry file and runtime
`field_verified:=true`, fresh odometry quality, a valid two-sided row, current
low-band coverage, measured braking/latency/margin values and forward motion.
Unknown physical values are zero in the diagnostic template and deny clearance.
Reverse motion and rotational sweeps are deliberately not authorized by this
first straight-row core. This gate needs obstacle-height/detection coverage
acceptance in the actual installation; field_verified must record that evidence.

Geometry YAML schema:

```yaml
geometry:
  frame_id: base_link
  field_verified: false
  footprint: [[0.5, 0.4], [0.5, -0.4], [-0.5, -0.4], [-0.5, 0.4]] # Example only
  vehicle_height_m: 0.0
  braking_deceleration_mps2: 0.0
  command_latency_sec: 0.0
  safety_margin_m: 0.0
  self_shapes:
    - {id: body, type: box, center_xyz: [0, 0, 0], size_xyz: [1, 0.8, 0.4], rpy: [0, 0, 0], padding_m: 0}
    - {id: fixture, type: cylinder, center_xyz: [0, 0, 0.5], radius_m: 0.04, height_m: 0.4, rpy: [0, 0, 0], padding_m: 0}
```

Do not include the transmitting aperture in an opaque shape. A shape enclosing
the sensor origin legitimately blocks all modeled rays. Physical footprint and
self geometry have different meanings: safety padding is not an excuse to
delete outside obstacles as self.

## Offline

```bash
ros2 run agt_local_row_perception local_row_offline \
  --config row_perception.yaml --geometry robot_geometry.yaml \
  --extrinsics sensor_extrinsics.yaml \
  --scans exported/scans.csv --poses exported/poses.csv --output row_results
```

`scans.csv`: `stamp,cloud_path,sensor_frame,odom_epoch`; cloud paths are relative
to this CSV. `stamp` is the original header/timebase, not bag receipt time.
Extra columns are allowed.

`poses.csv`: `stamp,x,y,z,qx,qy,qz,qw,odom_epoch`; pose is odom<-geometry.frame_id,
strictly increasing in each epoch. Point CSV or ASCII PCD fields are `x,y,z`
plus `timestamp` or `offset_time` and optional intensity. Binary PCD should be
exported to CSV first. No timing is inferred from point array order.

`sensor_extrinsics.yaml`:

```yaml
geometry:
  sensor_transform:
    translation_xyz: [0.0, 0.0, 0.0] # Replace with explicit calibrated transform
    rpy: [0.0, 0.0, 0.0]
```

`--allow-identity-extrinsics` explicitly declares cloud and pose body coordinates
already match and requires `sensor_frame == geometry.frame_id`; it is recorded
in the manifest. A run cannot mix sensor frames under one extrinsic transform.
`--input-deskewed` makes the same
explicit declaration as online. No geometry file or any unverified geometry
keeps clearance invalid. Field-certified offline scoring additionally requires
`--field-verified`; offline output is still no closed-loop validation.

Output: `row_states.csv`, `centerlines.csv`, per-frame SVG overlays, copied input
configs, and `manifest.yaml` with frame counts and acceptance status. Missing
pose brackets remain invalid frames instead of extrapolated synthetic poses.
Use a new output directory per run; existing outputs are never overwritten.
Offline stopping distance uses a finite difference of the body poses. When that
speed is unavailable, clearance remains invalid. Online stopping distance uses
the greater speed from the body twist and recent pose difference, accounting for
the actual odometry-child to body lever arm.

For a machine without ROS:

```bash
cmake -S . -B /tmp/agt_local_row_build -DAGT_LOCAL_ROW_STANDALONE=ON -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/agt_local_row_build --parallel 2
```

Eigen3 and yaml-cpp are the only standalone dependencies. This package adds no
tests; algorithm thresholds and detection coverage must be established using
the actual annotated data and robot recordings.
