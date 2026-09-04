# Bunker vibration diagnostics and LI-Init procedure

## 1. Vibration diagnostics by rosbag

Yes: vibration suitability should be measured from a recorded bag, not judged only online.

Record at minimum:

- `/livox/lidar`
- `/livox/imu`
- `/wheel/odom`
- `/agt/odometry/local`
- `/ins/navsatfix`
- `/ins/pose`
- `/ins/velocity`
- `/ins/odom`
- `/ins/status`
- `/tf`
- `/tf_static`

Helper:

```bash
bash $(ros2 pkg prefix agt_mapping_bringup)/share/agt_mapping_bringup/scripts/record_vibration_bag.sh
```

Recommended single-session sequence:

1. 20 s completely stationary, engine/electronics on;
2. 20 s tracks enabled but robot stationary if the platform permits it;
3. slow straight motion;
4. medium straight motion;
5. in-place left rotation;
6. in-place right rotation;
7. traverse grass/gravel/rough ground;
8. stop and remain stationary again.

Offline metrics to calculate:

- accel/gyro min, max, mean, RMS and standard deviation per axis;
- clipping/saturation event count near the sensor measurement limit;
- PSD / dominant vibration frequencies;
- stationary bias drift before vs after driving;
- odometry pose jumps and velocity spikes;
- correlation between track operation, wheel odometry and IMU peaks.

Keep the raw bag. Re-run the same bag through Batch-LIO after parameter changes so tuning stays A/B comparable.

## 2. LI-Init purpose

LI-Init is an installation calibration/diagnostic tool, not a normal navigation node. It estimates:

- LiDAR-to-IMU rotation;
- LiDAR-to-IMU translation;
- LiDAR/IMU temporal offset;
- gravity vector;
- IMU bias.

For ROS 2 Humble, evaluate the community ROS2 port `morte2025/LiDAR_IMU_Init_ROS2` while cross-checking parameter conventions and output with the upstream HKU-MARS `LiDAR_IMU_Init` MID360 configuration.

## 3. Data collection for LI-Init

Do not rely on a normal Bunker patrol trajectory. Tracked-base motion gives poor excitation for several calibration axes.

Preferred procedure:

1. rigidly mount LiDAR and the IMU that will actually be used by LIO;
2. record `/livox/lidar` and `/livox/imu` at native timing;
3. begin with a short stationary period;
4. rotate the complete LiDAR+IMU rigid body around roll, pitch and yaw axes;
5. include moderate translation in multiple directions;
6. avoid impacts and cable motion;
7. collect enough data for refinement to converge.

If the MID360 built-in IMU is used, verify the actual acceleration unit from the live message before setting `mean_acc_norm`; do not copy `1` or `9.805` blindly from another sensor configuration.

## 4. ROS2 test workflow

Example workspace:

```bash
mkdir -p ~/li_init_ws/src
cd ~/li_init_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
git clone https://github.com/morte2025/LiDAR_IMU_Init_ROS2.git
cd ~/li_init_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Create/modify a MID360 config so the inputs match the recorded bag:

```text
lidar: /livox/lidar
imu:   /livox/imu
```

Then play the dedicated calibration bag and run the LI-Init launch/config supplied by that port.

## 5. Accepting a calibration

Do at least 3 independent calibration runs using separate excitation recordings.

Accept only when:

- rotation results are mutually close;
- translation results are mutually close;
- time offset is repeatable;
- the refined result is stable rather than still drifting at shutdown;
- replay through Batch-LIO/FAST-LIO2 reduces deskew artifacts and stationary drift.

Store the accepted result as a versioned calibration asset with date, sensor serials, mount revision and source bag name.

After calibration, normal mapping/navigation should use the fixed measured extrinsics/time offset. Do not continuously estimate extrinsics during the first field demo unless a selected LIO explicitly requires it.
