# Global relocalization design

## Goal

The robot starts without a valid initial pose and recovers its 6-DoF pose from the 3D map using the MID360.

## Runtime sequence

```text
START
  |
  v
WAIT_SENSOR
  |
  v
ACCUMULATE_SCAN
  |
  v
SCAN_CONTEXT_RETRIEVAL
  |
  +-- no candidates --> retry
  |
  v
COARSE_REGISTRATION
  |
  v
GICP_FINE_REGISTRATION
  |
  v
VALIDATE
  |
  +-- reject --> next candidate / retry
  |
  v
LOCALIZED
  |
  v
HANDOFF_TO_FAST_LIO2
```

## Candidate database

A map database should contain a downsampled global map, local submaps/keyframes, and global descriptors. Descriptor retrieval must return candidate map poses, not merely a similarity score.

## Registration

Do not run ICP/GICP against the entire map for every startup. Retrieve candidates first, then register against local submaps. A robust global-registration backend can be added between retrieval and GICP if benchmarks show that descriptor yaw/translation seeds are insufficient.

## Validation

A pose is accepted only when multiple checks pass:

- registration fitness/residual
- sufficient point overlap
- plausible height relative to map
- candidate consistency across accumulated scans
- no impossible transform jump relative to the short local motion estimate

The exact thresholds remain benchmark parameters until the real MID360 rosbag is evaluated.

## FAST-LIO2 handoff

The recovered `T_map_base` is converted into the global correction required by the continuous odometry chain. The localization manager publishes/owns `map -> odom`; FAST-LIO2 continues to provide the local `odom -> base_footprint` trajectory.

The handoff must be atomic from the consumer's point of view: do not expose an intermediate false pose as a valid localization result.

## Startup modes

### Global

No pose seed. Full candidate retrieval is required.

### Seeded

A persisted `last_pose.yaml` may restrict the search area for faster startup. Failure falls back to global mode.

### Manual

A developer/operator may supply an initial pose for diagnostics. This is not the acceptance path for the core requirement.
