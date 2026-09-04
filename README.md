# agt_navigation_v3

AGT 户外履带式巡检机器人 ROS 2 Humble 导航、重定位与巡检集成仓库。

目标平台：**Bunker v1 + 倾斜 MID360 + Fast-LIO2 建图 + Batch-LIO 导航里程计 + 3D-BBS/small_gicp 无初值全局重定位 + Nav2 + RTK/INS 记录 + Autolabor C1 云台相机**。

当前开发主线是 `main`。现阶段只验收：

```text
RViz 定点
 -> Nav2 到点
 -> 实测底盘停稳
 -> C1 固定三视角拍照
 -> 记录 pose / image_stamp / RTK / 云台实际角度
 -> 下一点
 -> RETURN_HOME
```

**HMI、断电续巡、每点拍照可选策略暂不进入当前验收。**

---

## 1. 设计原则

1. **建图和导航里程计解耦**：Fast-LIO2 + PGO/HBA 负责全局一致地图；Batch-LIO 负责运行时连续局部里程计。
2. **LiDAR map matching 是全局定位主链**：V1 不依赖 RTK 自动重定位。
3. **一个 TF edge 只有一个 owner**：`agt_localization_manager` 唯一发布 `map -> odom`。
4. FAST-LIO/Batch-LIO 原始 MID360 时序链不经过 generic voxel/self-filter；导航障碍和重定位使用独立 PointCloud2 支路。
5. MID360 实际安装倾角保留在 URDF/TF，不通过“把点云拉平”破坏传感器几何关系。
6. Nav2 SUCCESS 不等于适合拍照；必须再通过 `/agt/odometry/local` measured-stop gate。
7. 每张照片使用 `image_stamp` 作为同步锚点，关联 `map -> base_link`、RTK 和云台实际角度。
8. 50 Hz 是 controller、velocity smoother、cmd guard、CAN command refresh 的端到端要求。
9. 当前优先保证 RViz 单点/三点巡检可重复，再增加 HMI、断电续巡和自动 readiness。

---

## 2. 当前主链

### Mapping Mode

```text
MID360 CustomMsg + built-in IMU
          ↓
robotics-laboratory/fast-lio2
          ↓
GTSAM PGO
          ↓
可选 HBA refinement
          ↓
final global_map.pcd
          ├────────────────┐
          ↓                ↓
agt_map_converter      build_relocalization_assets
          ↓                ↓
Nav2 / terrain map     3D-BBS assets + downsampled PCD
          └────────┬───────┘
                   ↓
                Map Package
```

### Navigation Mode

```text
MID360 CustomMsg + built-in IMU
          ↓
Functionhx/Batch-LIO
          ↓
/aft_mapped_to_init
          ↓
agt_batch_lio_adapter
          ↓
/agt/odometry/local   odom -> base_link
```

### Global Relocalization

```text
/agt/localization/relocalize
          ↓
静止 query cloud
          ↓
3D-BBS global coarse search
          ↓
局部地图裁剪
          ↓
small_gicp GICP refinement
          ↓
score / fitness / overlap gate
          ↓
/agt/relocalization/pose = T_map_base
          ↓
agt_localization_manager
          ↓
T_map_odom = T_map_base * inverse(T_odom_base)
```

Batch-LIO 从开机持续运行；重定位成功只锚定 `map -> odom`，不重启 LIO。

### RTK V1

RTK/INS 当前只用于：

```text
建图地理信息
巡检照片元数据
health / quality
未来 map <-> ENU 对齐资产
```

当前禁止：

```text
RTK -> 自动重定位 seed
RTK -> Batch-LIO correction
RTK -> map->odom 直接 correction
```

---

## 3. 首次部署：clone 后一条脚本补齐依赖

基础前提：

```text
Ubuntu 22.04
ROS 2 Humble 已安装到 /opt/ros/humble
```

只需要先 clone 本仓库：

```bash
mkdir -p ~/agt_ws/src

git clone https://github.com/Aldoubt/agt_navigation_v3.git \
  ~/agt_ws/src/agt_navigation_v3

cd ~/agt_ws
bash src/agt_navigation_v3/scripts/bootstrap_humble.sh --smoke
```

脚本会自动处理三层依赖。

### Ubuntu / ROS 包

```text
vcstool / rosdep / colcon / build tools
Eigen / PCL / yaml-cpp / Boost / TBB
ROS Humble GTSAM
Navigation2 / nav2_bringup / pcl_conversions / tf2_eigen
```

### Workspace 源码仓库

`dependencies/field_demo.repos` 现在包含：

```text
AGT hardware
  Aldoubt/agt_ins_driver            master
  Aldoubt/agt_bunker_base           main
  Aldoubt/agt_chassis_description   main
  Aldoubt/Autolabor-C1-ROS2         main

Pinned third party
  Livox-SDK/Livox-SDK2
  Livox-SDK/livox_ros_driver2
  robotics-laboratory/fast-lio2
  Functionhx/Batch-LIO
  KOKIAOKI/3d_bbs
  koide3/small_gicp
  morte2025/LiDAR_IMU_Init_ROS2
```

第三方算法/Livox 使用当前已验证 commit；我们自己的硬件仓库在实车验证完成前继续跟 `main/master`，暂不冻结。

### Native `/usr/local` 库

bootstrap 自动编译安装：

```text
Livox-SDK2
3D-BBS CPU
small_gicp
```

注意：不会调用 Livox ROS Driver 2 官方 `build.sh`，因为其脚本会删除 workspace 级 `build/`/`install/`。AGT bootstrap 只准备 ROS2 `package.xml/launch`，然后统一由 `colcon` 构建。

### bootstrap 常用模式

```bash
# 只拉取/安装，不构建
bash src/agt_navigation_v3/scripts/bootstrap_humble.sh --no-build

# 系统 apt 已准备好
bash src/agt_navigation_v3/scripts/bootstrap_humble.sh --no-apt

# 强制重装 native third-party
bash src/agt_navigation_v3/scripts/bootstrap_humble.sh --force-native

# 指定 workspace
bash src/agt_navigation_v3/scripts/bootstrap_humble.sh \
  --workspace ~/agt_ws --smoke
```

目标输出：

```text
BOOTSTRAP PASS
FIELD BUILD SMOKE PASS
```

详细说明：`docs/BOOTSTRAP_AND_ROSBAG_GATE.md`。

---

## 4. Humble 编译状态

GitHub Actions 当前分层检查：

```text
python / package.xml / yaml / shell / .repos syntax
ROS 2 Humble hardware-independent AGT core build
Map Converter / Map Manager tests
C1 public interface + inspection runtime build/import
3D-BBS CPU + small_gicp + AGT native relocalization build
```

已验证过的 Humble CI 范围包括：

```text
agt_robot_interfaces
agt_pointcloud_preprocessor
agt_map_converter
agt_map_manager
agt_localization_manager
agt_batch_lio_adapter
agt_base_control
agt_nav2_bringup
agt_navigation_runtime
agt_rviz_patrol
agt_global_relocalization_native
```

目标机完整检查：

```bash
cd ~/agt_ws
bash src/agt_navigation_v3/scripts/field_build_smoke.sh
```

CI PASS 不等于 hardware PASS；MID360、Bunker、URDF、C1 实机仍需现场验证。

---

## 5. MID360 P0：先确认 IMU 单位和外参

Batch-LIO baseline：

```yaml
extrinsic_est_en: false
extrinsic_T: [0.011, 0.02329, -0.04412]
extrinsic_R: [1,0,0, 0,1,0, 0,0,1]
time_diff_lidar_to_imu: 0.0
```

配置：

```text
src/agt_mapping_bringup/config/batch_lio_mid360.yaml
```

静止测试：

```bash
ros2 run agt_mapping_bringup mid360_imu_preflight.py --ros-args \
  -p duration_sec:=10.0
```

判断：

```text
mean_acc_norm ~ 1.0   -> Batch-LIO acc_norm = 1.0
mean_acc_norm ~ 9.81  -> Batch-LIO acc_norm = 9.81
其它                  -> FAIL
静止 gyro 过大        -> FAIL
```

当前固定的 `robotics-laboratory/fast-lio2` 源码会对 Livox IMU acceleration 乘 `10.0`，所以未修改的 mapping baseline 只接受原始静止 norm 约 `1.0`。若实际 driver 输出约 `9.81 m/s²`，先修正 mapping 前端单位处理，不能靠 covariance 掩盖。

LI-Init 当前是按证据启用的验证工具，不是开机依赖。

---

## 6. 从 MID360 rosbag 到上车的真正门槛

**不是“参数调顺了就直接跑车”**。在上车前，rosbag 至少应把 A-D 四层跑通。

### A. Raw sensor contract

```text
/livox/lidar   livox_ros_driver2/msg/CustomMsg
/livox/imu     sensor_msgs/msg/Imu
```

确认时间戳、点时间字段、IMU 单位、静止 gyro、LiDAR/IMU 时间偏差。

### B. Batch-LIO

```text
/aft_mapped_to_init
 -> agt_batch_lio_adapter
 -> /agt/odometry/local
```

要求：普通运动不 reset、轨迹/局部点云合理、静止 twist 足够稳定，能够支撑 measured-stop gate。

### C. Navigation point-cloud branch

```text
CustomMsg
 -> agt_livox_tools
 -> /agt/livox/points
 -> agt_pointcloud_preprocessor
 -> /agt/navigation/points_obstacles
```

检查 frame/timestamp、自车过滤、后方立柱过滤和障碍点密度。

### D. Offline no-initial-pose relocalization

使用最终 `global_map.pcd` + BBS assets，在多个不同静止位置验证：

```text
3D-BBS
 -> local-map small_gicp
 -> /agt/relocalization/pose
 -> Localization Manager
 -> map -> odom
```

至少确认：

```text
多个不同位置无需 /initialpose 能找到正确 pose
错误匹配会被 gate 拒绝
map -> odom -> base_link 视觉正确且稳定
重复重定位不会重启 Batch-LIO
```

这四层通过以后，就应该优先上车，而不是继续桌面扩功能。

rosbag 无法替代的现场项只有：

```text
Bunker CAN /mux/cmd_vel 实际执行
真实 URDF/static TF
C1 /camera_gimbal/acquire_view
真实履带停车动态
真实 Nav2 costmap / controller 表现
```

完整 go/no-go：`docs/BOOTSTRAP_AND_ROSBAG_GATE.md`。

---

## 7. 建图和地图资产

建图：

```bash
ros2 launch agt_mapping_bringup mapping_mode.launch.py
```

最终必须固定同一份优化后 PCD 给 Nav2 与 relocalization。

生成导航/terrain 地图：

```bash
ros2 run agt_map_converter pcd_to_nav_map \
  /data/site_A/global_map.pcd \
  --output /data/site_A/navigation \
  --resolution 0.10 \
  --max-step 0.22 \
  --max-slope-deg 20.0

ros2 run agt_map_converter validate_nav_map /data/site_A/navigation
```

预构建重定位资产：

```bash
ros2 run agt_global_relocalization_native build_relocalization_assets \
  --map /data/site_A/global_map.pcd \
  --output /data/site_A/relocalization
```

创建 Map Package：

```bash
ros2 run agt_map_manager create_map_package \
  --map-root ~/.ros/agt_maps \
  --map-id site_A \
  --map-version v1 \
  --source-pcd /data/site_A/global_map.pcd \
  --navigation-dir /data/site_A/navigation \
  --relocalization-assets-dir /data/site_A/relocalization
```

当前 Nav2 在线 hot-reload/rollback 仍未完成，所以 RViz demo 启动时仍显式传同一版本的 `navigation/map.yaml`。

---

## 8. 当前 RViz 实车验收

外部先启动并确认：

```text
MID360 driver
Bunker CAN driver
robot_state_publisher / URDF
Autolabor C1 capability
可选 agt_ins_driver
```

然后：

```bash
ros2 launch agt_system_bringup rviz_field_demo.launch.py \
  map:=/data/site_A/navigation/map.yaml \
  global_map:=/data/site_A/global_map.pcd \
  relocalization_assets:=/data/site_A/relocalization \
  map_id:=site_A_v1
```

机器人保持静止，人工触发：

```bash
ros2 service call /agt/localization/relocalize \
  std_srvs/srv/Trigger "{}"
```

RViz 确认：

```text
map -> odom -> base_link
```

再运行：

```bash
ros2 run agt_navigation_runtime demo_preflight
```

当前验收顺序固定：

```text
第一轮：
P001 -> measured stop -> 3 images -> HOME

第二轮：
P001 -> 3 images
P002 -> 3 images
P003 -> 3 images
RETURN_HOME -> standby
```

记录验证：

```bash
ros2 run agt_navigation_runtime validate_records \
  /path/to/mission_dir --expected-points 3

ros2 run agt_navigation_runtime generate_demo_report \
  /path/to/mission_dir
```

现场操作文档：`docs/RVIZ_FIELD_ACCEPTANCE.md`。

---

## 9. 当前代码结构

```text
src/
├── agt_robot_interfaces
├── agt_livox_tools
├── agt_pointcloud_preprocessor
├── agt_mapping_bringup
├── agt_fastlio_adapter
├── agt_batch_lio_adapter
├── agt_global_relocalization
├── agt_global_relocalization_native
├── agt_localization_manager
├── agt_rtk_manager
├── agt_map_manager
├── agt_map_converter
├── agt_nav2_bringup
├── agt_base_control
├── agt_navigation_runtime
├── agt_rviz_patrol
└── agt_system_bringup
```

重要文档：

```text
docs/BOOTSTRAP_AND_ROSBAG_GATE.md
docs/RVIZ_FIELD_ACCEPTANCE.md
docs/GLOBAL_RELOCALIZATION_BBS_GICP.md
docs/FIELD_SENSOR_BASELINE.md
docs/MAPPING_AND_LIO_POLICY.md
docs/VIBRATION_AND_LI_INIT.md
docs/CURRENT_NAVIGATION_CAPABILITIES.md
docs/MAINLINE_POLICY.md
```

---

## 10. 改造进度

| 模块 | 状态 | 当前情况 |
| --- | --- | --- |
| Bootstrap / dependency fetch | 🟡 | apt + repos + Livox-SDK2 + BBS + small_gicp 自动化已落地，待目标机首次完整执行 |
| Humble CI | 🟢/🟡 | core/runtime/native 分层编译已验证；不能替代硬件验收 |
| Mapping Fast-LIO2 + PGO/HBA | 🟡 | wrapper/config 已收口，待真实 MID360 建图 |
| Batch-LIO navigation odom | 🟡 | explicit MID360 config + adapter 已落地，待 rosbag/实车参数冻结 |
| MID360 IMU baseline | 🟡 | acc_norm / gyro preflight 已落地 |
| 3D-BBS global coarse | 🟡 | native CPU backend 已落地并在 Humble CI 编译 |
| local-submap small_gicp | 🟡 | GICP refinement 已落地并在 Humble CI 编译 |
| Map Package / Manager | 🟡 | PCD/Nav2/BBS assets/version/hash 已落地 |
| Nav2 active-map hot apply | 🔴 | 两阶段 reload/rollback 后续 |
| Localization Manager | 🟡 | 唯一 map->odom + handoff/gate 已落地 |
| Local obstacle branch | 🟡 | CustomMsg secondary branch + filters 已落地 |
| Nav2 | 🟡 | SmacPlanner2D + RPP + 50 Hz baseline，待履带实车调参 |
| Bunker guard | 🟡 | `/cmd_vel -> /mux/cmd_vel` 50 Hz，待实车 watchdog/限幅确认 |
| Inspection runtime | 🟡 | Nav2 -> measured stop -> C1 -> synchronized record 已落地 |
| RViz patrol | 🟡 | queue + fixed 3 views + RETURN_HOME 已落地 |
| HMI | ⏸️ | RViz 三点稳定后再接 |
| Auto readiness | ⏸️ | 后续 |
| Power-cycle resume | ⏸️ | 后续 |
| Full hardware acceptance | 🔴 | 等 MID360 rosbag gate + Bunker/C1 实车 |

---

## 11. Git / 依赖冻结策略

- `main` 是当前快速验证主线；
- 第三方算法/Livox 栈使用 exact commit，保证环境可复现；
- AGT 自有 Bunker/C1/INS/URDF 在实车稳定前继续跟 `main/master`；
- 不为每个小功能长期维护分支；
- RViz 三点巡检稳定后，再做 release/tag 和自有驱动版本冻结。
