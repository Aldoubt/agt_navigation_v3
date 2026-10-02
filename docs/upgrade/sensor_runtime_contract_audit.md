# P6 Sensor Runtime Contract Audit

Date: 2026-10-02

Branch: `refactor/navigation-runtime-v4`

P6 audited the existing sensor ownership and topic contracts. No driver,
calibration, topic remap, filter, point timing, or TF behavior was changed.
The sensor-only aggregator remains a launch composition and does not include
Nav2 or the base. A physical sensor launch was not started in this audit.

## LiDAR and IMU

| Boundary | Owner/input | Canonical output and contract |
|---|---|---|
| MID360 driver | `agt_robot_bringup/lidar.launch.py` selects one driver path from the validated robot config: vendor `livox_ros_driver2/msg_MID360_launch.py`, or the explicit mapping `mapping_custom` Node | `/livox/lidar`, `livox_ros_driver2/msg/CustomMsg`, with `header`, `timebase`, and each point's `offset_time`; `/livox/imu`, `sensor_msgs/msg/Imu`. Current vendor launch selects customized format, one shared topic, `livox_frame`, and 10.0 Hz publish frequency. The mapping mode selects the same CustomMsg output type. |
| FAST-LIO2 | Selected only by `agt_navigation_runtime/lio.launch.py` through `fastlio_navigation_lio.launch.py` | Subscribes directly to `/livox/lidar` and `/livox/imu`. The frozen runtime YAML remains the frontend calibration authority. |
| Batch-LIO | Mutually exclusive alternative selected by the same `lio.launch.py` | Its temporary selected runtime YAML rewrites old `/agt/sensors/lidar/custom` and `/agt/sensors/imu/data` defaults to the passed canonical `/livox/lidar` and `/livox/imu`. It consumes the raw Livox CustomMsg path. |
| Secondary conversion | `agt_livox_tools/livox_format_bridge.launch.py`, included once by system localization | `/livox/lidar` CustomMsg -> `/agt/livox/points` `sensor_msgs/msg/PointCloud2`. Header frame and `timebase` stamp are retained; `x/y/z`, intensity, tag, line, `offset_time`, and per-point `timestamp` are emitted. |
| Local obstacle processing | `agt_pointcloud_preprocessor/local_perception.launch.py` in the navigation layer | `/agt/livox/points` -> `/agt/navigation/points_obstacles`. Its range/self/ground/voxel operations are after the Livox conversion and are never inserted before either LIO frontend. Costmap clearing continues to use `/agt/livox/points`. |
| Relocalization/tracking | Global relocalization and optional map tracker | Both consume `/agt/livox/points` as PointCloud2; neither publishes sensor data or owns sensor TF. |

The raw and secondary branches are:

```text
MID360
  ├─ /livox/lidar CustomMsg + /livox/imu Imu ─> exactly one selected LIO frontend
  └─ /livox/lidar CustomMsg ─> Livox format bridge ─> /agt/livox/points PointCloud2
                                                       ├─ relocalization/tracking
                                                       ├─ local perception -> /agt/navigation/points_obstacles
                                                       └─ costmap clearing uses /agt/livox/points
```

No generic voxel, height, self, or obstacle preprocessor is present on the raw
CustomMsg-to-LIO edge. `lidar_filter_num` in the FAST-LIO runtime YAML remains
an internal frontend setting and was not changed.

## RTK/INS

| Item | Contract |
|---|---|
| Hardware owner | `agt_robot_bringup/rtk.launch.py` includes `agt_asensing_driver/asensing.launch.py` once when the validated config enables RTK. `bunker_inspection` marks the INS installed but disabled by default. |
| Vendor input | ASENSING serial stream on configured `/dev/ttyUSB0`, 460800 baud. The driver parser produces the ROS messages; this audit did not read from the serial device. |
| Driver outputs | `/ins/navsatfix` (`sensor_msgs/NavSatFix`), `/ins/imu` (`sensor_msgs/Imu`), `/ins/pose` (`PoseStamped`), `/ins/velocity` (`TwistStamped`), `/ins/odom` (`nav_msgs/Odometry`), `/ins/status` (ASENSING `INSStatus`), and `/ins/raw_frame` (`UInt8MultiArray`). |
| Navigation consumer | Optional `agt_rtk_manager` consumes `/ins/navsatfix` and `/ins/status`, publishing record/quality topics `/agt/rtk/status` and `/agt/rtk/map_pose`. The manager does not publish TF. V1 does not feed RTK into LIO or global relocalization. |

## C1 camera and gimbal

| Item | Contract |
|---|---|
| Hardware owner | `agt_robot_bringup/camera.launch.py` includes the C1 atomic launch once when the selected robot config enables the camera/gimbal. The device path, dimensions, frame rate, pixel format, and serial path come from the robot device config. |
| Camera input/output | USB camera device -> `/cv_camera0/image_raw` (`sensor_msgs/Image`), frame `camera_link`. The reference inspection config is 1920x1080 at 10 FPS with `mjpeg2rgb`. |
| Gimbal feedback/control | Serial gimbal driver under `/pantilt_camera_serial0`; publishes `pantilt_status` and `pantilt_angle_info`, and provides `/pantilt_camera_serial0/move_pantilt` (`MovePantilt` action). The driver does not create a competing robot TF tree. |
| Capture capability | `camera_gimbal_capability` consumes image and gimbal status, calls the move action, and exposes `/camera_gimbal/acquire_view` plus `/camera_gimbal/health`. The V1 mission invokes capture after its measured-stop gate. |

## Launch and singleton ownership

- `agt_robot_bringup/lidar.launch.py`, `rtk.launch.py`, and `camera.launch.py`
  are the physical sensor atomics. The profile-resolved `sensors.launch.py`
  composes only the enabled sensor atomics; its helper gates out the description
  and base in sensor-only mode and has no Nav2 include.
- `agt_system_bringup/hardware.launch.py` includes the platform hardware
  aggregator; `system.launch.py` selects that hardware aggregator once. It does
  not also include `sensors.launch.py`. These are alternative entry points and
  must not be run together for the same robot.
- `agt_system_bringup/localization.launch.py` selects exactly one LIO backend,
  one Livox bridge, one relocalization manager, and one `map -> odom` owner.
- Device driver instances are not shared between launch entry points. Runtime
  graph inspection of physical drivers remains unrun; P6 source tests assert
  launch composition and unique source ownership only.

## P6 tests, evidence scope, and rollback

The source-level sensor tests cover vendor and mapping driver format settings,
the raw LIO topic/type, Batch-LIO topic normalization, conversion fields and
timestamp semantics, secondary-cloud consumers, sensor-only aggregator
composition, and absence of Nav2 from that aggregator. A mock ROS graph sent a
synthetic `CustomMsg` through the built C++ bridge and verified the output
PointCloud2 geometry, frame, timebase stamp, `offset_time`, and per-point
timestamp. It ran in an isolated domain with no physical driver, LIO, Nav2,
base, camera, or RTK process.

| Evidence class | P6 result |
|---|---|
| Unit/mock | **PASS** — four selected packages built; 58 colcon test results passed; static owner/topic checks passed; isolated fake Livox-to-PointCloud2 graph passed. |
| Rosbag replay | **NOT_RUN** — no rosbag was replayed in P6. |
| Real hardware | **NOT_RUN** — no physical sensor or robot launch was started. |

No runtime sensor configuration changed, so rollback is to revert the P6
source-test and documentation commit. The raw Livox input and default
`bunker_v1` driver selection remain byte-for-byte unchanged. Keep the previous
atomic launch entry points if rolling back the P4 launch split; never add a
filter before LIO as a rollback shortcut.
