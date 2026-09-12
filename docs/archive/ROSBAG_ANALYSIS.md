# ROS bag analysis

Generated with `ros2 bag info`. The bags use `/agt/sensors/...` names rather than live `/livox/lidar` and `/livox/imu` names.

| bag | LiDAR format | Fast-LIO2 / Batch-LIO | BBS/GICP |
|---|---|---|---|
| bunker_mid360_mapping_20260901_205036 | CustomMsg (`/agt/sensors/lidar/custom`) | YES | YES after CustomMsg→PointCloud2 |
| bunker_mid360_mapping_20260901_211105 | CustomMsg plus PointCloud2 (`/agt/commissioning/mapping/registered_points`) | YES, use CustomMsg | YES, use recorded PointCloud2 |

Both bags contain `sensor_msgs/msg/Imu` on `/agt/sensors/imu/data`. Neither contains exact `/livox/lidar` or `/livox/imu`; remap/configure the consumer. Reverse conversion is prohibited for LIO unless `offset_time` or per-point `timestamp` is present; otherwise it is geometry-only.

## Raw output

### bunker_mid360_mapping_20260901_205036

Files:             bunker_mid360_mapping_20260901_205036_0.db3
Bag size:          3.1 GiB
Storage id:        sqlite3
Duration:          482.297427344s
Start:             Sep  1 2026 20:50:36.853540316 (1788267036.853540316)
End:               Sep  1 2026 20:58:39.150967660 (1788267519.150967660)
Messages:          268427
Topic information: Topic: /agt/sensors/lidar/custom | Type: livox_ros_driver2/msg/CustomMsg | Count: 9578 | Serialization Format: cdr
                   Topic: /agt/chassis/connected | Type: std_msgs/msg/Bool | Count: 4778 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr
                   Topic: /agt/sensors/lidar/custom_filtered | Type: livox_ros_driver2/msg/CustomMsg | Count: 4736 | Serialization Format: cdr
                   Topic: /agt/sensors/imu/data | Type: sensor_msgs/msg/Imu | Count: 96018 | Serialization Format: cdr
                   Topic: /agt/chassis/odometry | Type: nav_msgs/msg/Odometry | Count: 47934 | Serialization Format: cdr
                   Topic: /diagnostics | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 4736 | Serialization Format: cdr
                   Topic: /agt/chassis/rc_state | Type: bunker_msgs/msg/BunkerRCState | Count: 47934 | Serialization Format: cdr
                   Topic: /agt/chassis/status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 4778 | Serialization Format: cdr
                   Topic: /agt/chassis/status/raw | Type: bunker_msgs/msg/BunkerStatus | Count: 47934 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 0 | Serialization Format: cdr


### bunker_mid360_mapping_20260901_211105

Files:             bunker_mid360_mapping_20260901_211105_0.db3
Bag size:          4.2 GiB
Storage id:        sqlite3
Duration:          390.102454763s
Start:             Sep  1 2026 21:11:06.127665770 (1788268266.127665770)
End:               Sep  1 2026 21:17:36.230120533 (1788268656.230120533)
Messages:          218336
Topic information: Topic: /agt/commissioning/mapping/registered_points | Type: sensor_msgs/msg/PointCloud2 | Count: 3891 | Serialization Format: cdr
                   Topic: /agt/sensors/lidar/custom | Type: livox_ros_driver2/msg/CustomMsg | Count: 3896 | Serialization Format: cdr
                   Topic: /agt/chassis/connected | Type: std_msgs/msg/Bool | Count: 3896 | Serialization Format: cdr
                   Topic: /agt/sensors/lidar/custom_filtered | Type: livox_ros_driver2/msg/CustomMsg | Count: 3895 | Serialization Format: cdr
                   Topic: /agt/sensors/imu/data | Type: sensor_msgs/msg/Imu | Count: 77909 | Serialization Format: cdr
                   Topic: /agt/chassis/odometry | Type: nav_msgs/msg/Odometry | Count: 39019 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr
                   Topic: /diagnostics | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 3895 | Serialization Format: cdr
                   Topic: /agt/chassis/rc_state | Type: bunker_msgs/msg/BunkerRCState | Count: 39019 | Serialization Format: cdr
                   Topic: /agt/chassis/status | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 3896 | Serialization Format: cdr
                   Topic: /agt/chassis/status/raw | Type: bunker_msgs/msg/BunkerStatus | Count: 39019 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 0 | Serialization Format: cdr
