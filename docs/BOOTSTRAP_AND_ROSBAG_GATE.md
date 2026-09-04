# Bootstrap and MID360 rosbag -> field gate

This document defines the minimum reproducible environment setup and the gate from
offline MID360 rosbag tuning to the first Bunker field test.

## 1. One-command dependency bootstrap

Base prerequisite:

```text
Ubuntu 22.04
ROS 2 Humble installed at /opt/ros/humble
```

The bootstrap intentionally does not rewrite the machine's ROS apt repository.
Everything above the base Humble installation is automated.

First clone only this repository:

```bash
mkdir -p ~/agt_ws/src
git clone https://github.com/Aldoubt/agt_navigation_v3.git \
  ~/agt_ws/src/agt_navigation_v3
cd ~/agt_ws
```

Then run:

```bash
bash src/agt_navigation_v3/scripts/bootstrap_humble.sh --smoke
```

The script is repeatable. Existing repositories are skipped rather than overwritten.

It installs/fetches:

```text
APT / ROS packages
  build tools / vcstool / rosdep / colcon
  Eigen / PCL / yaml-cpp / Boost / TBB
  ROS Humble GTSAM
  Navigation2 / nav2_bringup / pcl_conversions / tf2_eigen

AGT hardware repositories
  Aldoubt/agt_ins_driver             master
  Aldoubt/agt_bunker_base            main
  Aldoubt/agt_chassis_description    main
  Aldoubt/Autolabor-C1-ROS2          main

Pinned field-demo third parties
  Livox-SDK/Livox-SDK2
  Livox-SDK/livox_ros_driver2
  robotics-laboratory/fast-lio2
  Functionhx/Batch-LIO
  KOKIAOKI/3d_bbs
  koide3/small_gicp
  morte2025/LiDAR_IMU_Init_ROS2
```

The external algorithms and Livox stack use exact commits from
`dependencies/field_demo.repos`. AGT-owned hardware repositories intentionally follow
their current `main/master` branches until field validation is stable enough to freeze.

Native libraries installed to `/usr/local`:

```text
Livox-SDK2
3D-BBS CPU library
small_gicp
```

The bootstrap does **not** execute Livox ROS Driver 2's upstream `build.sh`, because
that script deletes the workspace-level `build/` and `install/` directories. Instead,
it creates the ROS2 `package.xml` / `launch` links and lets the normal workspace
`colcon build` own the build.

Useful modes:

```bash
# fetch/install only
bash src/agt_navigation_v3/scripts/bootstrap_humble.sh --no-build

# do not touch apt packages
bash src/agt_navigation_v3/scripts/bootstrap_humble.sh --no-apt

# rebuild /usr/local native dependencies
bash src/agt_navigation_v3/scripts/bootstrap_humble.sh --force-native

# explicitly select workspace
bash src/agt_navigation_v3/scripts/bootstrap_humble.sh \
  --workspace ~/agt_ws --smoke
```

Expected end states:

```text
BOOTSTRAP PASS
FIELD BUILD SMOKE PASS       # when --smoke is used
```

## 2. Is MID360 rosbag parameter tuning the last step before the robot?

Almost, but the acceptance gate is **not** simply "the LIO parameters look good".
The bag must prove the complete localization-side chain that can be proven offline.

### Gate A — raw sensor contract

The bag must contain the same production interfaces used on the robot:

```text
/livox/lidar   livox_ros_driver2/msg/CustomMsg
/livox/imu     sensor_msgs/msg/Imu
```

Verify:

- timestamps are monotonic;
- point timing is preserved;
- no generic voxel filter is inserted before Batch-LIO;
- IMU acceleration unit is known;
- static gyro level is acceptable;
- LiDAR/IMU time offset is not visibly wrong.

Use `mid360_imu_preflight` on a stationary live segment or equivalent bag statistics.

### Gate B — Batch-LIO local odometry

After parameter correction, the bag must produce:

```text
/aft_mapped_to_init
        -> agt_batch_lio_adapter
        -> /agt/odometry/local
```

Required observations:

- no reset during ordinary motion;
- no repeated timestamp disorder warnings;
- stationary end segments converge to a small measured twist;
- trajectory and local cloud are visually consistent in RViz;
- aggressive tracked-base vibration does not cause obvious attitude/height divergence.

Do not tune only for a visually smooth trajectory. The stop gate later depends on
real odometry twist, so static noise must also be measured.

### Gate C — secondary point-cloud branch

The same bag must prove:

```text
Livox CustomMsg
  -> agt_livox_tools
  -> /agt/livox/points
  -> agt_pointcloud_preprocessor
  -> /agt/navigation/points_obstacles
```

Required observations:

- timestamps/frame IDs remain correct;
- tilted MID360 mount is handled by TF, not by rewriting raw LIO points;
- self/rear-pole filtering does not remove useful forward environment structure;
- Nav2 obstacle cloud remains dense enough for local collision marking.

### Gate D — relocalization against the actual final map

Use the **same final optimized `global_map.pcd`** that will generate the Nav2 map.
Prebuild the BBS assets, then replay one or more stationary bag segments from different
known positions.

The following chain must run:

```text
stationary query cloud
 -> 3D-BBS global coarse search
 -> local-map crop
 -> small_gicp GICP
 -> score / fitness / overlap gate
 -> /agt/relocalization/pose
 -> Localization Manager
 -> map -> odom
```

Required observations before first field navigation:

- at least several distinct test positions relocalize without `/initialpose`;
- the winning pose is visually correct in RViz;
- false matches are rejected instead of publishing a plausible-looking wrong pose;
- `map -> odom -> base_link` is continuous after handoff;
- repeated relocalization does not restart Batch-LIO.

If the available bag has no useful stationary segments, record a short dedicated bag
before navigation testing: startup static + several static stops in different places.

### Gate E — what rosbag cannot prove

The bag cannot validate these hardware-runtime items:

```text
Bunker CAN command path
/cmd_vel -> 50 Hz guard -> /mux/cmd_vel -> chassis

real URDF/static TF
base_link -> tilted MID360
base_link -> camera/gimbal frames

C1 AcquireView action
/camera_gimbal/acquire_view

real robot stop dynamics
Nav2 SUCCESS -> measured stop -> camera capture
```

These are the remaining first-day-on-robot checks.

## 3. First field test after the bag gates pass

If Gates A-D pass, then yes: the project is ready to move to the robot for the first
**relocalization + RViz fixed inspection task** test.

Do not start with a multi-point mission. Use this order:

```text
1. power robot, keep tracks disabled / lifted if practical
2. start MID360 + URDF + Bunker + C1
3. verify live Batch-LIO local odom
4. keep robot stationary
5. manually trigger /agt/localization/relocalize
6. verify map -> odom -> base_link in RViz
7. run demo_preflight
8. enable base motion
9. send one nearby Nav2 goal
10. verify measured stop
11. execute fixed 3-view C1 capture
12. RETURN_HOME
13. inspect record directory
14. only then run 3-point patrol
```

The current field acceptance remains:

```text
P001 -> stop -> 3 images -> HOME
then
P001 -> P002 -> P003 -> 9 images -> HOME
```

HMI dispatch, power-cycle resume and per-point camera optionality remain out of scope
until this sequence is repeatable.

## 4. Practical go/no-go rule

**GO to first robot relocalization test** when all are true:

```text
bootstrap / field build smoke PASS
MID360 IMU units known
Batch-LIO bag trajectory usable
stationary twist usable for stop gate
secondary PointCloud2 branch usable
final global_map.pcd fixed
BBS assets generated
multiple offline no-initial-pose relocalizations visually correct
Localization Manager produces the only map->odom
```

**GO to first moving task test** only after additionally verifying live:

```text
Bunker /mux/cmd_vel path
URDF/static TF
C1 AcquireView
Nav2 costmaps
manual demo_preflight PASS
```

If those conditions hold, further desktop feature work is lower priority than getting
the first one-point field run and collecting a real vibration/navigation bag.
