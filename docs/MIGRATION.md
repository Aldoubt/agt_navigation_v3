# AGT Navigation V3 migration guide

This document defines the reproducible migration boundary for the software
checkpoint that passed the 2026-09-05 offline relocalization and Gazebo
navigation gates.

## Validated base system

```text
Ubuntu 22.04
ROS 2 Humble at /opt/ros/humble
Gazebo Classic 11 for the simulation gate
```

The repository intentionally does not commit rosbags, generated maps, colcon
build products or runtime logs. Those are site/test data, not source code.

## Fresh-machine restore

```bash
mkdir -p ~/agt_ws/src
git clone https://github.com/Aldoubt/agt_navigation_v3.git \
  ~/agt_ws/src/agt_navigation_v3

cd ~/agt_ws
bash src/agt_navigation_v3/scripts/bootstrap_humble.sh --smoke
```

The bootstrap will:

1. install Ubuntu/ROS dependencies, including Nav2 and Gazebo Classic support;
2. import the exact source revisions in `dependencies/agt_navigation.repos`;
3. fail if an existing exact-pinned checkout is at a different HEAD;
4. build Livox-SDK2, CPU 3D-BBS, Sophus and small_gicp into
   `<workspace>/.agt_native`;
5. build the field software chain and `agt_gazebo_sim`;
6. run the hardware-independent smoke checks when `--smoke` is supplied.

For an already provisioned Ubuntu/ROS host or an offline rebuild, add `--no-apt`.
That mode skips apt and rosdep network installation but still verifies exact source revisions, rebuilds workspace-local native libraries, runs colcon, and executes the smoke gate.

The validated Gazebo Livox package is vendored at
`src/ros2_livox_simulation` with its upstream MIT license. This is deliberate:
the accepted version contains AGT timing/range/Jammy fixes and must not depend
on an unversioned directory copied from one development machine.

## After bootstrap

```bash
source /opt/ros/humble/setup.bash
source ~/agt_ws/install/setup.bash

ros2 pkg prefix agt_system_bringup
ros2 pkg prefix agt_gazebo_sim
ros2 pkg prefix ros2_livox_simulation
ros2 pkg prefix camera_gimbal_interfaces
```

Run the repeatable safety gate directly if needed:

```bash
ROS_DOMAIN_ID=149 python3 \
  ~/agt_ws/src/agt_navigation_v3/src/agt_base_control/test/guard_fail_closed_acceptance.py \
  --ros-args --params-file \
  ~/agt_ws/src/agt_navigation_v3/src/agt_base_control/config/cmd_vel_guard.yaml
```

Expected final line begins with:

```text
GUARD_ACCEPTANCE PASS
```

## Hardware-only repositories

`dependencies/field_demo.repos` lists the Bunker/URDF/INS/C1 hardware-side
repositories used by the field integration. The software checkpoint pins the
INS and C1 revisions already exercised by this workspace. Bunker/description
branches are intentionally not called frozen until the upcoming vehicle test.

To import the full hardware-side set into a fresh workspace:

```bash
cd ~/agt_ws
vcs import --skip-existing src \
  < src/agt_navigation_v3/dependencies/field_demo.repos
```

## Runtime data that must be copied separately

Do not expect Git to contain site data. Move these through your normal data
backup/transfer channel:

```text
rosbags
3D PCD maps
poses.txt / patches/
generated relocalization assets
generated Nav2 map.yaml/map.pgm and terrain layers
inspection image/result directories
```

Recommended destination outside the repository:

```text
~/agt_ws/agt_data/
~/agt_ws/rosbag/
```

This keeps a source checkout portable and prevents large binary data from being
accidentally pushed to GitHub.

## Recreate the Gazebo acceptance map

The generated `gazebo_mapping_v1` map is test output and is not committed. A
new machine should recreate it using the committed simulation harness rather
than copying a hidden local workspace dependency:

```bash
ros2 launch agt_gazebo_sim mapping_demo.launch.py
```

Then use the production map-save/conversion/relocalization-asset tools documented
in `docs/ACCEPTANCE.md` and `docs/GLOBAL_RELOCALIZATION_BBS_GICP.md` before
running `navigation_demo.launch.py`.

## Migration acceptance rule

A migration is considered successful only when all of the following hold:

```text
bootstrap_humble.sh --smoke        PASS
dependency exact-HEAD verification PASS
core + Gazebo packages build       PASS
launch --show-args checks          PASS
guard fail-closed acceptance       PASS
```

The Gazebo and rosbag gates prove software-chain reproducibility. They do not
replace the upcoming Bunker/MID360 field tuning and hardware acceptance.
