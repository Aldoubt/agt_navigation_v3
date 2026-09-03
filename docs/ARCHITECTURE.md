# Architecture

## Objective

Build a ROS 2 Humble localization system that can start with no initial pose, globally localize a tilted-mounted Livox MID360 against a prebuilt 3D map, then hand the recovered global pose to FAST-LIO2 for continuous tracking.

## Non-goals for V1

- RTK/INS fusion
- camera/LIVO fusion
- Bunker CAN control
- Nav2 mission behavior
- power-cycle recovery as an independent localization algorithm

Power-cycle recovery is a V2 system-level acceptance case built on the same no-initial-pose global localization path.

## Data flow

```text
                    map database
              global map + submaps
                + Scan Context
                       |
                       v
MID360 -> preprocessing -> global relocalization
  |            |              |
  |            |              +-> coarse registration
  |            |              +-> GICP fine registration
  |            |              +-> validation
  |            |
  |            +----------------------+
  |                                   |
  v                                   v
FAST-LIO2 continuous odometry     recovered map pose
  |                                   |
  +-------------------+---------------+
                      v
             Localization Manager
                      |
                      v
                  map -> odom
                      |
                      v
                   base_link
```

## Localization states

```text
BOOT
  -> WAIT_SENSOR
  -> BUILD_INITIAL_SCAN
  -> GLOBAL_SEARCH
  -> COARSE_MATCH
  -> FINE_MATCH
  -> VALIDATE
  -> LOCALIZED
  -> FAST_LIO_TRACKING
```

Any failed global search returns to `GLOBAL_SEARCH` with bounded retry/backoff.

## Global localization strategy

V1 is intentionally a two/three-stage architecture rather than whole-map ICP:

1. **Global descriptor retrieval**: Scan Context or equivalent descriptor retrieves Top-K map candidates.
2. **Global/coarse registration**: a robust global registration backend produces a metric pose candidate.
3. **Fine registration**: GICP/Nano-GICP refines the candidate against a local submap.
4. **Validation**: fitness, overlap, residual, height and motion sanity checks reject false matches.

The implementation must keep these stages behind interfaces so backends can be benchmarked without changing ROS interfaces.

## FAST-LIO2 handoff

Global localization owns `map -> odom` correction. FAST-LIO2 owns continuous local odometry. The handoff must not duplicate TF publishers. A localization manager is the single owner of the global correction relationship.

`last_pose` persistence is optional acceleration only. A missing or stale seed must fall back to true global search.

## Tilted MID360 policy

The MID360 installation tilt is physical and must remain represented by TF. Do not rotate the incoming cloud to make the sensor appear level. Mapping and localization use the same `lidar_link` convention and the same sensor-to-body transform.

## Self filtering

The point-cloud preprocessor removes known robot geometry in `base_link` using collision primitives transformed into `lidar_link`. This is preferred to deleting the entire rear sector because environmental geometry behind the robot is useful to global localization.

A narrow rear angular/range mask exists only as an optional fallback for known rods/structures that extend outside the collision proxy.

## Failure handling

A failed global localization must never silently publish a confident pose. Status and confidence are explicit. FAST-LIO2 handoff occurs only after validation succeeds.
