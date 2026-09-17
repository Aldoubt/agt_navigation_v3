# FAST-LIO2 anchor navigation mode

`fastlio_anchor_nav.launch.py` is a navigation-only field mode. It does not
start Batch-LIO, PGO, map saving, or `agt_map_tracker`.

```text
/livox/lidar + /livox/imu
        |
        v
FAST-LIO2: odom -> body, /fastlio2/lio_odom
        |
        v
agt_fastlio_adapter: calibrated odom -> base_link,
                      /agt/odometry/local
        |
        +--------------------> agt_localization_manager

/livox/lidar -> PointCloud2 bridge -> 3D-BBS + GICP (one-shot auto request)
                                            |
                                            v
                             /agt/relocalization/pose (T_map_base_link)
                                            |
                                            v
                              map -> odom anchor, then fixed Nav2 RPP
```

The global relocalizer automatically retries only until it obtains one accepted
startup result. `agt_map_tracker` is deliberately absent and the localization
manager tracker topics are isolated, so no low-rate tracker measurement changes
the active `map -> odom` correction during navigation. Explicit manual recovery
remains available through `/agt/localization/relocalize`.

`body_to_base_calibration_file` is read only to reuse the frozen body-to-LiDAR
calibration used by the formal map package and global relocalization. It does
not launch or consume Batch-LIO odometry.

## Field preflight

Before enabling motion, verify exactly one publisher for each dynamic edge:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 topic hz /agt/odometry/local
ros2 topic echo --once /agt/localization/metrics
```

The expected local chain is `FAST-LIO2: odom -> body` plus
`agt_fastlio_adapter: odom -> base_link`. Do not start the Batch-LIO navigation
launch in the same ROS domain.
