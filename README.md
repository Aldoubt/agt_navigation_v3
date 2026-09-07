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

## 0. 实车传感器启动固定流程

本节是 Bunker v1 实车的固定**传感器/底盘状态验收**流程。每个长期运行的
命令使用独立终端；先启动并验证传感器，确认静止数据正常后才允许进入建图或
导航流程。

当前已现场验证的物理基线：MID360 `192.168.1.117`，Bunker `can0@500000`，
ASENSING 使用 Prolific 的稳定 `/dev/serial/by-id` 路径，C1 相机使用
`/dev/video0`、`1920x1080@30`。

### 0.1 前检（不启动任何运动控制）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ip -br address show enp2s0
ping -c 2 192.168.1.117
ls -l /dev/serial/by-id
v4l2-ctl --device=/dev/video0 --all | head -30
```

预期 MID360 ping 成功，且至少存在下列稳定设备链接：

```text
usb-1a86_USB_Serial-if00-port0                    # C1 云台
usb-Prolific_Technology_Inc._USB-Serial_Controller_* # ASENSING INS
```

### 0.2 启动 MID360（终端 1）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=1 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=livox_frame \
  -p user_config_path:=/home/yangxuan/ros2_ws/src/external/livox_ros_driver2/config/MID360_config.json
```

### 0.3 启动 ASENSING RTK/INS（终端 2）

不要使用配置文件中的 `/dev/ttyUSB0` 默认值；它与 C1 云台端口冲突。始终显式
覆盖为 Prolific 的 by-id 路径：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run agt_asensing_driver asensing_node --ros-args \
  --params-file ~/ros2_ws/install/agt_asensing_driver/share/agt_asensing_driver/config/asensing.yaml \
  -p port:=/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_DTAZj137C01-if00-port0
```

RTK/INS 在 V1 只记录质量和照片元数据；不得用它发布或修正 `map -> odom`。

### 0.4 启动 C1 相机与云台（终端 3）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch autolabor_c1_bringup autolabor_c1.launch.py \
  gui:=false \
  device_path:=/dev/video0 \
  port_name:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

此步骤只打开图像流与云台反馈；不要调用 `/camera_gimbal/acquire_view`，以免云台
发生主动转动。

### 0.5 启动 CAN 与 Bunker 状态驱动（终端 4）

先启用 SocketCAN 并确认底盘持续有报文：

```bash
sudo ~/ros2_ws/src/agt_navigation_v3/scripts/agt_bunker_can up
ip -details link show can0
candump can0
```

预期 `can0` 为 `UP / ERROR-ACTIVE` 且 `candump` 持续输出。没有 RX 报文、出现
`ERROR-PASSIVE` 或 `BUS-OFF` 时，停止在这里，检查底盘电源、急停、CAN 线束、
终端电阻及 `500000` 位速率；不要启动 ROS 底盘驱动。

CAN 正常后，在另一个终端启动驱动：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch bunker_base bunker_base.launch.py \
  port_name:=can0 \
  odom_topic_name:=/wheel/odom \
  publish_odom_tf:=false
```

该启动入口已将第三方驱动内部的 `/cmd_vel` 重映射为 `/mux/cmd_vel`。驱动连接时会
请求 commanded mode；保持遥控器和急停可用。传感器验收阶段严禁发布
`/cmd_vel` 或 `/mux/cmd_vel`。

### 0.6 只读验收与停止顺序

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 topic hz /livox/lidar       # 约 10 Hz
ros2 topic hz /livox/imu         # 约 200 Hz
ros2 topic hz /cv_camera0/image_raw  # 1080p 下接近 30 Hz
ros2 topic echo --once /camera_gimbal/health
ros2 topic echo --once /pantilt_camera_serial0/pantilt_status
ros2 topic echo --once /ins/status
ros2 topic echo --once /bunker_status
ros2 topic echo --once /bunker_rc_state
ros2 topic hz /wheel/odom        # 约 50 Hz；静止时速度为 0
ros2 topic info /mux/cmd_vel -v  # 传感器验收时 Publisher count 必须为 0
```

停止时按反向顺序在各终端 `Ctrl+C`：先 Bunker，再 C1、RTK/INS、MID360；最后执行：

```bash
sudo ~/ros2_ws/src/agt_navigation_v3/scripts/agt_bunker_can down
```

完成本节并且全车保持静止后，才可启动 URDF、LIO、建图或导航链路。

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
          ├─────────────────────┐
          ↓                     ↓
build_relocalization_assets   FAST-LIO2 body_cloud
          ↓                     ↓
3D-BBS assets + PCD     rear dynamic filter → OctoMap O1-H2
          └─────────────────────┬───────────────────────┘
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

### 默认导航地图生成

当前默认导航地图生成能力是 **OctoMap O1-H2 + 后方动态扇区过滤**：0.1 m
OctoMap、`0.0 < z < 2.0 m`，并在 FAST-LIO2 已发布的 `body_cloud` 上过滤
0.8–5.0 m 后方、60° 扇区内的点。该分支只生成 Nav2 `map.pgm/map.yaml`；
它不修改 MID360 原始输入、FAST-LIO2、全局定位 PCD 或 Batch-LIO。

启动与导出命令见 `agt_mapping_bringup/README.md`。`agt_terrain_map_generator`
及旧 `agt_map_converter` 保留为显式的后续升级/回归路径，不再作为默认地图生成器。

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

脚本会自动处理三层依赖，并校验核心依赖的 exact commit。

### Ubuntu / ROS 包

```text
vcstool / rosdep / colcon / build tools
Eigen / PCL / yaml-cpp / Boost / TBB
ROS Humble GTSAM
Navigation2 / nav2_bringup / pcl_conversions / tf2_eigen
Gazebo Classic / gazebo_ros / xacro / robot_state_publisher
```

### Workspace 源码仓库

默认迁移使用 `dependencies/agt_navigation.repos`。当前软件 gate 所需的
AGT interface/INS 与第三方算法都使用已验证 exact commit：

```text
AGT runtime dependencies
  Aldoubt/agt_ins_driver            cfa7b94...
  Aldoubt/Autolabor-C1-ROS2         10d8217...

Pinned third party
  Livox-SDK/Livox-SDK2
  Livox-SDK/livox_ros_driver2
  robotics-laboratory/fast-lio2
  Functionhx/Batch-LIO
  KOKIAOKI/3d_bbs
  koide3/small_gicp
  strasdat/Sophus
```

`dependencies/field_demo.repos` 额外列出 Bunker/URDF 等硬件仓库；其中尚未经过
本轮实车冻结的仓库仍保留 `main`，不会被误称为当前软件 gate 的可复现依赖。

Gazebo 使用的 `ros2_livox_simulation` 已作为 MIT vendored package 放在：

```text
src/ros2_livox_simulation
```

因此新系统不需要手工复制工控机上的仿真插件目录。

### Workspace-local native 库

bootstrap 自动编译安装：

```text
Livox-SDK2
3D-BBS CPU
small_gicp
```

默认安装到 workspace：

```text
<workspace>/.agt_native
```

不会要求把验证版本写入系统 `/usr/local`，迁移和清理更可控。

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

迁移步骤：`docs/MIGRATION.md`。
详细 bootstrap 说明：`docs/BOOTSTRAP_AND_ROSBAG_GATE.md`。

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
  --map-root /home/yangxuan/ros2_ws/agt_data/maps \
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
docs/MIGRATION.md
docs/BOOTSTRAP_AND_ROSBAG_GATE.md
docs/RVIZ_FIELD_ACCEPTANCE.md
docs/GLOBAL_RELOCALIZATION_BBS_GICP.md
docs/RELOCALIZATION_DEBUGGING_RETROSPECTIVE_2026-09-05.md
docs/ACCEPTANCE.md
docs/TF_CONVENTION.md
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
| Mapping Fast-LIO2 + PGO/HBA | 🟡 | 已有真实 MID360 优化地图用于离线重定位；仍待实车重新建图/版本化验收 |
| Batch-LIO navigation odom | 🟢/🟡 | 211105 rosbag 已通过 adapter/local-odom 闭环；固定 body->base_link 外参已收口；针对上游 zero twist 已增加 pose-delta 线/角速度估计，待实车参数冻结 |
| MID360 IMU baseline | 🟡 | acc_norm / gyro preflight 已落地 |
| Global candidate + 3D-BBS coarse | 🟢/🟡 | Polar Context Top-K + candidate-local CPU BBS 已通过匹配 rosbag 离线闭环，待多点位实车成功率验收 |
| local-submap small_gicp | 🟢/🟡 | GICP 6DoF refinement 已在真实 rosbag 自动重定位中通过，待实车分布统计 |
| Map Package / Manager | 🟡 | PCD/Nav2/BBS assets/version/hash 已落地 |
| Nav2 active-map hot apply | 🔴 | 两阶段 reload/rollback 后续 |
| Localization Manager | 🟢/🟡 | 唯一 map->odom + handoff/gate 已在 211105 离线闭环进入 LOCALIZED 并实际发布 map->odom，待实车验收 |
| Local obstacle branch | 🟡 | CustomMsg secondary branch + filters 已落地 |
| Nav2 | 🟢/🟡 | SmacPlanner2D + RPP 50 Hz baseline；Gazebo 无 initialpose 冷启动后已取得真实 `NavigateToPose=SUCCEEDED`；PoseProgressChecker 与 RPP bootstrap 已收口，待履带实车调参 |
| Bunker guard | 🟢/🟡 | 50 Hz + LocalizationStatus fail-closed gate 已通过 ROS pub/sub 故障注入：LOST 后约 1.4 ms 捕获到 0，恢复但无新 cmd 时 stale replay=0；待实车 watchdog/限幅确认 |
| Inspection runtime | 🟢/🟡 | measured stop 已改为 pose delta + twist + hold time，零 twist/运动回归测试通过；C1 interface 已并入 ros2_ws，待实车拍照链验收 |
| RViz patrol | 🟡 | queue + fixed 3 views + RETURN_HOME 已落地 |
| HMI | ⏸️ | RViz 三点稳定后再接 |
| Auto readiness | ⏸️ | 后续 |
| Power-cycle resume | ⏸️ | 后续 |
| Offline relocalization gate | 🟢 | 2026-09-05：bag_mapping_current + 211105 无 initialpose 自动重定位闭环通过 |
| Gazebo navigation gate | 🟢 | 2026-09-05：清理旧测试进程后单栈冷启动；自动重定位 `score=0.927`，随后 PGO 已知自由区目标 `NavigateToPose=SUCCEEDED`（45.3 s） |
| Full hardware acceptance | 🔴 | 离线重定位 + Gazebo 闭环 gate 已通过；仍待 Bunker 多点位/多朝向、树荫、振动、C1 实车 |

---

## 11. Git / 依赖冻结策略

- `main` 是当前快速验证主线；
- 第三方算法/Livox 栈使用 exact commit，保证环境可复现；
- AGT 自有 Bunker/C1/INS/URDF 在实车稳定前继续跟 `main/master`；
- 不为每个小功能长期维护分支；
- RViz 三点巡检稳定后，再做 release/tag 和自有驱动版本冻结。
