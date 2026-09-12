# R1.1 build baseline

Baseline: `4a12559`; recorded on 2026-09-10.

## Recovered generated artifacts

- Moved only the stale generated Python directory for `pantilt_camera_serial`
  to a `/tmp/pantilt-cache.*` directory; rebuilt successfully.
- Built workspace-local `small_gicp`, restoring its CMake config export.
- Used the existing workspace `.agt_native` prefix for CPU 3D-BBS; native
  relocalization and its Python wrapper build successfully without source edits.
- Moved only `build/camera_gimbal_msgs/ament_cmake_python/camera_gimbal_msgs/camera_gimbal_msgs`
  to `/tmp/camera-gimbal-msgs-cache.*`; `camera_gimbal_msgs` then rebuilt.

No source, map, rosbag, or install tree was deleted.

## Result

The original symlink-install conflicts are resolved. A subsequent whole-workspace
build advanced beyond them but is not a PASS: it fails while configuring the
unrelated ROS 1 catkin package `ins` under `src/drivers/agt_ins_driver/...`.
The R1.1 navigation packages and `agt_system_bringup` targeted build pass.

For native relocalization builds, preserve the existing prefix:

```bash
source install/setup.bash
CMAKE_PREFIX_PATH=/home/yangxuan/ros2_ws/.agt_native:$CMAKE_PREFIX_PATH \
  colcon build --symlink-install --packages-select \
  agt_global_relocalization_native agt_global_relocalization
```
