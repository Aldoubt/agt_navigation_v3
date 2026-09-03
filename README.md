# agt_navigation_v3

ROS 2 Humble navigation and inspection integration stack for the AGT tracked robot.

目标平台：**Bunker v1 履带底盘 + 倾斜安装 MID360 + FAST-LIO2 + 无初值 3D 全局重定位 + Nav2 + RTK/INS 辅助 + Qt HMI + Autolabor C1 云台相机**。

> 当前集成分支：`runtime-v1`
>
> 代码持续落地，但只有通过 Ubuntu 22.04 / ROS 2 Humble `colcon build`、目标 rosbag 回归和真实 Bunker 验收的内容，才算 hardware-ready。

---

## 1. Design principles / 设计原则

### 1.1 LiDAR localization is primary

树荫环境下 RTK 可能从 FIX 退化到 FLOAT/无解，因此导航不能依赖 RTK 连续可用。

```text
Global Relocalization / Localization Manager
                  |
                  | owns map -> odom
                  v
map ------------ odom ------------ base_link
                                  ^
                                  |
                            FAST-LIO2
                       continuous local odometry
```

原则：

- FAST-LIO2 是高频局部运动估计主源；
- Localization Manager 是唯一 `map -> odom` 修正所有者；
- Bunker `/wheel/odom` 仅用于控制/诊断/后续谨慎设计的 fallback，不作为正常模式 Nav2 主里程计；
- RTK/INS 是低频全局观测、启动校验和健康信号；
- 不再通过第二套 EKF 重复融合 FAST-LIO2 已经使用过的同一 IMU，避免重复计权和 TF/odom 反馈环。

Nav2 的标准局部里程计入口冻结为：

```text
/agt/odometry/local
```

后续由 FAST-LIO2 adapter / Localization Manager 输出。

### 1.2 Exactly one owner per TF edge

目标 TF：

```text
map
 |
 +-- odom                  Localization Manager
      |
      +-- base_link        FAST-LIO2 / localization adapter
           |
           +-- physical static frames from URDF
```

`agt_bunker_base` 必须保持 `publish_odom_tf=false`，不能与 FAST-LIO2 竞争 `odom -> base_*`。

MID360 实际倾角保留在 TF 中，不在点云输入阶段人为拉平。

### 1.3 Preserve MID360 timing for FAST-LIO2

FAST-LIO2 的输入链必须保留点时间/IMU 时序语义。通用 voxel/downsample 不能无验证地放在 FAST-LIO2 前面。

```text
MID360 raw/custom msg + IMU
        |
        +----------------------> FAST-LIO2 time-preserving path
        |
        +--> self-filter / obstacle processing
                    |
                    +--> /agt/navigation/points_obstacles
                    +--> global relocalization scan
                    +--> debug
```

`/agt/navigation/points_obstacles` 是 Nav2 3D local costmap 的独立分支，不能反向替换 FAST-LIO2 原始时序输入。

### 1.4 RTK is an observation, not a TF owner

`agt_rtk_manager` 输入：

```text
/ins/navsatfix
/ins/status
```

输出：

```text
/agt/rtk/status
/agt/rtk/map_pose
```

当 active Map Package 提供地理锚点时，已经实现：

```text
WGS84 -> ECEF -> ENU -> map
```

`/agt/rtk/map_pose` 是带 covariance 的 map-frame **位置观测**。RTK manager 不发布 `map -> odom`，也不直接让 FAST-LIO2 发生跳变；后续只允许 Localization Manager / global optimizer 在质量门控后消费这个约束。

地理锚点必须随地图版本管理。

### 1.5 Map Package is the product asset

地图不是单独一个 `map.pgm`。目标资产：

```text
maps/<map_id>/
├── metadata.yaml
├── localization/
│   ├── global_map.pcd
│   ├── submaps/
│   └── scan_context.db
├── navigation/
│   ├── map.yaml
│   ├── map.pgm
│   ├── elevation.pgm
│   ├── slope.pgm
│   └── obstacle.pgm
├── rtk/
│   └── origin.yaml
└── preview.png
```

巡检任务、RTK anchor、定位地图、Nav2 地图和地形层最终都必须绑定同一个 `map_id + map_version`。

### 1.6 HMI is a client, not the robot brain

保留现有 `agt_robot_hmi`。Qt HMI 负责地图显示、巡检点编辑、任务控制和结果展示，不直接控制 Nav2、CAN、云台串口或定位内部状态。

兼容接口继续保留：

```text
/agt/task/request
/agt/task/start
/agt/task/pause
/agt/task/cancel
/agt/task/status
```

`agt_navigation_runtime` 在迁移到正式 typed interfaces 的过程中继续兼容这套 HMI 边界。

### 1.7 Camera-gimbal is a capability

任务层只消费 C1 已冻结的公开 Action：

```text
/camera_gimbal/acquire_view
```

成功代表：云台真实稳定到位，并且获得稳定之后产生的新图像。

巡检数据以 `image_stamp` 为同步锚点：

```text
image_stamp
   +-> lookup(map -> base_link)
   +-> associate RTK
   +-> actual gimbal encoder angles
   +-> image path / metadata
```

### 1.8 50 Hz is an end-to-end control requirement

现有 `agt_bunker_base` 已经以 `/mux/cmd_vel` 为默认控制输入，底层循环为 50 Hz，本仓库不重写 CAN 驱动。

```text
Nav2 controller @ 50 Hz
        |
      /cmd_vel
        |
velocity_smoother @ 50 Hz
        |
 /cmd_vel_smoothed
        |
agt_cmd_vel_guard @ 50 Hz
 clamp + slew + stale watchdog
        |
  /mux/cmd_vel
        |
agt_bunker_base -> CAN
```

上游命令超时后 guard 立即转为显式零速度，并继续按固定频率刷新。

当前 Regulated Pure Pursuit 只是第一轮履带实车 baseline，不是最终冻结选型；RPP / MPPI / DWB 后续使用同一场地和 rosbag 做 benchmark 再冻结。

---

## 2. Repository layout

```text
agt_navigation_v3/
├── config/                         common design-stage configuration
├── docs/                           architecture / TF / map / acceptance
└── src/
    ├── agt_robot_interfaces/       shared typed ROS interfaces
    ├── agt_navigation_runtime/     HMI -> Nav2 -> C1 -> recorder
    ├── agt_rtk_manager/            RTK quality gate + WGS84/ENU/map
    ├── agt_nav2_bringup/           Nav2 Humble, no AMCL ownership conflict
    ├── agt_base_control/           50 Hz command guard/watchdog
    └── agt_system_bringup/         staged top-level integration launch
```

---

## 3. External repositories

推荐放在同一个 colcon workspace；HMI 也可以在操作端单独运行。

```text
https://github.com/Aldoubt/agt_navigation_v3.git
https://github.com/Aldoubt/agt_ins_driver.git
https://github.com/Aldoubt/agt_bunker_base.git
https://github.com/Aldoubt/agt_chassis_description.git
https://github.com/Aldoubt/Autolabor-C1-ROS2.git
https://github.com/Aldoubt/agt_robot_hmi.git
```

MID360 driver、FAST-LIO2 ROS 2 port 和 3D Map Localization backend 等到目标 rosbag benchmark 后再冻结具体 commit/branch。

---

## 4. Build

目标环境：Ubuntu 22.04 + ROS 2 Humble。

```bash
mkdir -p ~/agt_ws/src
cd ~/agt_ws/src

git clone https://github.com/Aldoubt/agt_navigation_v3.git
git -C agt_navigation_v3 checkout runtime-v1

git clone https://github.com/Aldoubt/agt_ins_driver.git
git clone https://github.com/Aldoubt/agt_bunker_base.git
git clone https://github.com/Aldoubt/agt_chassis_description.git
git clone https://github.com/Aldoubt/Autolabor-C1-ROS2.git
# HMI 可选同机或操作端单独构建
git clone https://github.com/Aldoubt/agt_robot_hmi.git

cd ~/agt_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

当前 ChatGPT 执行环境没有真实 ROS 2 Humble 目标工作区，因此这里的“implemented”不能替代目标机 `colcon build` 和实机验收。

---

## 5. Usage / 使用方法

### 5.1 INS + RTK manager

先启动 ASENSING：

```bash
ros2 launch agt_asensing_driver asensing.launch.py
```

再启动 RTK manager：

```bash
ros2 launch agt_rtk_manager rtk_manager.launch.py
ros2 topic echo /agt/rtk/status
```

地理锚点示例：

```text
src/agt_rtk_manager/config/map_origin.example.yaml
```

实际运行时把 active Map Package 对应的 origin 文件配置到 `map_origin_file`，即可输出 `/agt/rtk/map_pose`。

注意：FLOAT 可以按参数作为辅助监控观测发布，但不等于允许它直接修正导航 TF。

### 5.2 Bunker base

```bash
ros2 launch agt_bunker_base bunker_base.launch.py
ros2 launch agt_base_control cmd_vel_guard.launch.py
```

重要约束：

```text
agt_bunker_base publish_odom_tf = false
Nav2 / FAST-LIO2 does not use /wheel/odom as normal-mode global/local pose owner
/mux/cmd_vel only receives guarded motion commands
```

### 5.3 C1 camera-gimbal

按照 `Autolabor-C1-ROS2` 仓库自己的 Phase-1 bringup 启动。

本仓库只依赖公开 capability：

```text
/camera_gimbal/acquire_view
/camera_gimbal/health
```

### 5.4 Nav2 baseline

Nav2 不启动 AMCL，因为 `map -> odom` 由 3D LiDAR localization 管理。

```bash
ros2 launch agt_nav2_bringup navigation.launch.py \
  map:=/absolute/path/to/maps/site_A/navigation/map.yaml
```

发送导航 goal 前必须具备：

```text
valid map -> odom
valid odom -> base_link
/agt/odometry/local
/agt/navigation/points_obstacles
```

其中 `/agt/odometry/local` 计划由 FAST-LIO2 adapter / Localization Manager 输出，**不能默认改回 `/wheel/odom`**。

当前：

```text
controller_frequency = 50 Hz
controller baseline  = Regulated Pure Pursuit
local costmap        = 3D VoxelLayer from /agt/navigation/points_obstacles
```

### 5.5 Inspection runtime

```bash
ros2 launch agt_navigation_runtime runtime.launch.py
```

示例任务：

```text
src/agt_navigation_runtime/config/mission_example.yaml
```

执行主链：

```text
NAVIGATE
 -> ARRIVAL
 -> BASE SETTLE
 -> GIMBAL MOVE / STABLE
 -> NEW IMAGE CAPTURE
 -> image_stamp TF lookup
 -> associate RTK
 -> write image + CSV + JSONL
 -> next view / next point
```

默认记录目录：

```text
~/.ros/agt_inspection_records/
```

### 5.6 Staged system bringup

当前顶层入口采用安全的分阶段启用策略：

```bash
ros2 launch agt_system_bringup system.launch.py
```

默认只启用 RTK manager 和 base guard。Localization / obstacle cloud 尚未 ready 时不自动启动 Nav2 和 mission runtime。

定位与地图 ready 后：

```bash
ros2 launch agt_system_bringup system.launch.py \
  enable_nav2:=true \
  map:=/absolute/path/to/maps/site_A/navigation/map.yaml
```

C1 + Nav2 都 ready 后再启用巡检 runtime：

```bash
ros2 launch agt_system_bringup system.launch.py \
  enable_nav2:=true \
  enable_runtime:=true \
  map:=/absolute/path/to/maps/site_A/navigation/map.yaml
```

MID360 driver、FAST-LIO2、3D global localization、INS driver、Bunker driver 和 C1 driver 当前仍保持独立硬件验收边界。待目标机稳定后再收口到真正的一键整机 bringup。

---

## 6. Frozen ROS integration contracts

```text
Localization
  /agt/odometry/local                nav_msgs/Odometry       planned adapter output
  map -> odom                        TF                      Localization Manager only

RTK
  /ins/navsatfix                     sensor_msgs/NavSatFix
  /ins/status                        agt_asensing_driver/INSStatus
  /agt/rtk/status                    agt_robot_interfaces/RTKStatus
  /agt/rtk/map_pose                  PoseWithCovarianceStamped

Obstacle perception
  /agt/navigation/points_obstacles   sensor_msgs/PointCloud2

Control
  /cmd_vel                           Nav2 controller output
  /cmd_vel_smoothed                  velocity_smoother output
  /mux/cmd_vel                       guarded Bunker command
  /wheel/odom                        Bunker wheel odometry / diagnostics

Inspection
  /agt/mission/execute               ExecuteInspectionMission action
  /camera_gimbal/acquire_view        C1 AcquireView action

HMI compatibility
  /agt/task/request
  /agt/task/start
  /agt/task/pause
  /agt/task/cancel
  /agt/task/status
```

---

## 7. Integration progress / 改造进度

| Area | Status | Notes |
| --- | --- | --- |
| Architecture / TF / map design | ✅ | 文档已建立，参数仍随实车更新 |
| `agt_robot_interfaces` | 🟡 implemented, build pending | mission / inspection / RTK interfaces |
| HMI compatibility bridge | 🟡 implemented | 保留 `/agt/task/*` |
| Inspection runtime | 🟡 implemented | Nav2 + C1 + timestamped records |
| C1 capability integration | 🟡 integrated | 硬件验收仍在 C1 仓库执行 |
| RTK quality/freshness manager | 🟡 implemented | 不拥有导航 TF |
| WGS84 -> ECEF -> ENU -> map | 🟡 implemented | 需要真实 map anchor 标定验证 |
| Nav2 bringup | 🟡 baseline implemented | no AMCL; RPP baseline; real Humble test pending |
| Bunker 50 Hz command guard | 🟡 implemented | `/cmd_vel_smoothed -> /mux/cmd_vel`; 频率需实机验收 |
| Bunker CAN driver | ✅ external repo | 直接复用 `agt_bunker_base` |
| Qt HMI | ✅ external base available | 不重写，逐步迁移正式 interfaces |
| Staged system bringup | 🟡 implemented | 自动 readiness state machine 尚未实现 |
| FAST-LIO2 adapter | 🔴 next | 输出 `/agt/odometry/local` |
| Localization Manager | 🔴 next | single TF owner + runtime LOST/relocalization |
| Timing-safe MID360 split | 🔴 next | LIO 原始时序链与 obstacle branch 分离 |
| 3D global relocalization backend | 🔴 next | Scan Context -> coarse -> GICP + validation |
| Map Manager V1 | 🔴 next | discovery/version/atomic switch/rollback |
| Terrain converter | 🔴 next | elevation/slope/roughness/traversability |
| Power-cycle mission resume | 🔴 later | checkpoint + relocalize + map validation + continue |
| Full hardware acceptance | 🔴 not passed | 需要目标机器人和 Humble 环境 |

状态说明：

- ✅：已有明确可复用资产；
- 🟡：代码已落地，但不能宣称实机 PASS；
- 🔴：接口/方向已冻结，仍待代码落地。

---

## 8. Immediate landing order

接下来不重新拆架构，按下面顺序继续：

1. **FAST-LIO2 adapter + Localization Manager**：建立 `/agt/odometry/local`、唯一 TF owner、`LOCALIZED -> DEGRADED -> LOST -> RELOCALIZING`；
2. **Timing-safe MID360 split**：保住 FAST-LIO2 点时间，同时生成自车过滤后的 Nav2 obstacle cloud；
3. **Global relocalization adapters**：Scan Context / coarse registration / GICP / validation + MID360 rosbag benchmark；
4. **Map Manager V1**：Map Package discovery、schema/hash/version 校验、原子切换和 rollback；
5. **Terrain converter V1**：PCD -> elevation + slope + roughness + traversability -> Nav2；
6. **System readiness manager**：sensor -> map -> localization -> Nav2 -> mission 门禁；
7. **Power-cycle resume**：checkpoint + 重新定位 + map version 校验 + 从未完成巡检点继续；
8. **Controller benchmark**：RPP / MPPI / DWB 实车对比并冻结参数。

---

## 9. Dependency freeze policy

当前阶段不过早冻结所有外部仓库。端到端 Demo 和 acceptance dataset 稳定后，再记录：

```text
repository URL
commit SHA
driver / firmware version
sensor calibration version
map schema version
```

最终可复现版本由 workspace dependency manifest + Map Package metadata 共同确定。
