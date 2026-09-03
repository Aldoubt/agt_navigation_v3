# agt_navigation_v3

AGT 户外履带式巡检机器人 ROS 2 Humble 导航与任务集成仓库。

目标平台：**Bunker v1 + 倾斜 MID360 + FAST-LIO2 + 无初值 3D 全局重定位 + Nav2 + RTK/INS 辅助 + Qt HMI + Autolabor C1 云台相机**。

> 当前集成分支：`runtime-v1`
>
> 本仓库持续落地代码，但“已实现”不等于“已实机通过”。只有完成 Ubuntu 22.04 / ROS 2 Humble `colcon build`、目标 rosbag 回归和真实 Bunker 验收后才可进入稳定主线。

---

## 1. 设计原则

### 1.1 LiDAR 定位是主链，RTK 是辅助

树荫环境下 RTK 可能从 FIX 退化到 FLOAT 或 LOST，因此导航不能依赖 RTK 连续可用。

```text
3D Global Relocalization
        |
        | T_map_base
        v
Localization Manager -------- owns map -> odom
        ^
        |
FAST-LIO2 -> agt_fastlio_adapter -> /agt/odometry/local
                                     |
                                     +---- odom -> base_link
```

约束：

- FAST-LIO2 是正常模式下唯一高频局部运动估计主源；
- `agt_localization_manager` 是唯一 `map -> odom` 修正所有者；
- Bunker `/wheel/odom` 只用于控制、诊断和未来谨慎设计的 fallback，不作为正常模式 Nav2 主 odometry；
- 不默认再用 `robot_localization` 把 FAST-LIO2 已使用过的同一 IMU 二次融合，避免重复计权和 TF/odom 反馈环；
- RTK/INS 只作为地理锚点、启动校验、运行时健康观测或后续优化约束。

Nav2 的标准局部 odometry 接口固定为：

```text
/agt/odometry/local
```

### 1.2 每条 TF 只能有一个 owner

目标 TF：

```text
map
 |
 +-- odom                  agt_localization_manager
      |
      +-- base_link        selected FAST-LIO2 / verified adapter path
           |
           +-- static physical frames from agt_chassis_description
```

`agt_bunker_base` 必须保持 `publish_odom_tf=false`。

MID360 的真实倾角保留在 URDF/TF 中，不在点云输入前人为拉平。

### 1.3 FAST-LIO2 时序链和导航障碍点云必须分开

通用 voxel/downsample 不能无验证地放在 FAST-LIO2 前面，否则可能破坏点时间和 deskew 所需信息。

```text
MID360 raw/custom message + IMU
        |
        +---------------------> FAST-LIO2 time-preserving path
        |
        +-> self-filter / obstacle processing
                    |
                    +-> /agt/navigation/points_obstacles
                    +-> global relocalization scan
                    +-> debug
```

`/agt/navigation/points_obstacles` 是 Nav2 local costmap 的独立分支，不能反向替代 FAST-LIO2 输入。

### 1.4 Global pose 必须和 local odom 同时间匹配

`agt_localization_manager` 接收：

```text
/agt/odometry/local
/agt/relocalization/pose
```

全局重定位给出时刻 `t` 的 `T_map_base`，Manager 从 local odom buffer 中寻找时间最接近的 `T_odom_base(t)`，然后计算：

```text
T_map_odom = T_map_base * inverse(T_odom_base)
```

全局结果必须通过 frame、timestamp、covariance 和最大时间偏差门限才会被接受。

运行状态：

```text
BOOT
 -> WAIT_LOCAL_ODOM
 -> WAIT_GLOBAL
 -> LOCALIZED
 -> DEGRADED
 -> LOST
 -> RELOCALIZING
 -> LOCALIZED
```

进入 `LOST` 后停止刷新动态 `map -> odom`，不让旧 TF 长期伪装成有效定位。

### 1.5 RTK 不拥有导航 TF

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

已实现：

```text
WGS84 -> ECEF -> ENU -> map
```

`/agt/rtk/map_pose` 是带 covariance 的 map-frame **位置观测**。RTK manager 不发布 `map -> odom`，也不会直接跳变修正 FAST-LIO2。

Map Package 中的 `rtk/origin.yaml` 必须和 `map_id/map_version` 一起版本管理。

### 1.6 Map Package 是产品资产，不只是 PGM

目标结构：

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
│   ├── roughness.pgm
│   └── obstacle.pgm
├── rtk/
│   └── origin.yaml
└── preview.png
```

巡检点、定位地图、Nav2 地图、地形层和 RTK anchor 最终必须绑定同一个地图版本。

### 1.7 HMI 是客户端，不是机器人业务逻辑

继续使用 `agt_robot_hmi`，不重新开发 GUI。HMI 负责显示、任务编辑和操作，下层业务由 runtime 完成。

现有兼容接口继续保留：

```text
/agt/task/request
/agt/task/start
/agt/task/pause
/agt/task/cancel
/agt/task/status
```

后续逐步迁移到 `agt_robot_interfaces` typed contracts，而不是重写 Qt 页面。

### 1.8 C1 云台相机通过 capability 接入

任务层只调用：

```text
/camera_gimbal/acquire_view
```

成功必须代表云台真实稳定并获得稳定之后产生的新图像。

巡检记录以 `image_stamp` 为同步锚点：

```text
image_stamp
   +-> lookup(map -> base_link)
   +-> associate RTK
   +-> actual gimbal angles
   +-> image path / metadata
```

### 1.9 50 Hz 是端到端控制要求

复用现有 `agt_bunker_base` CAN 驱动，不重写底盘协议。

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

上游速度命令超过 watchdog 门限后，guard 立即转为显式零速度并继续刷新。

当前 Regulated Pure Pursuit 只是第一轮 Bunker baseline；RPP / MPPI / DWB 后续用同一场地和数据集做实车 benchmark 再冻结。

---

## 2. 当前代码结构

```text
agt_navigation_v3/
├── config/
├── docs/
└── src/
    ├── agt_robot_interfaces/       typed mission / RTK / localization contracts
    ├── agt_fastlio_adapter/        FAST-LIO2 odom frame/timestamp gate
    ├── agt_localization_manager/   single map->odom owner + runtime health state
    ├── agt_rtk_manager/            RTK quality gate + WGS84/ENU/map observation
    ├── agt_nav2_bringup/           Nav2 Humble, no AMCL ownership conflict
    ├── agt_base_control/           50 Hz cmd_vel guard/watchdog
    ├── agt_navigation_runtime/     HMI -> Nav2 -> C1 -> recorder
    └── agt_system_bringup/         staged top-level launch
```

---

## 3. 外部仓库

建议放入同一个 colcon workspace；HMI 也可在操作端单独运行。

```text
https://github.com/Aldoubt/agt_navigation_v3.git
https://github.com/Aldoubt/agt_ins_driver.git
https://github.com/Aldoubt/agt_bunker_base.git
https://github.com/Aldoubt/agt_chassis_description.git
https://github.com/Aldoubt/Autolabor-C1-ROS2.git
https://github.com/Aldoubt/agt_robot_hmi.git
```

MID360 driver、FAST-LIO2 ROS 2 fork 和 3D Map Localization backend 在目标 rosbag benchmark 后再冻结具体 commit。

---

## 4. 构建

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
git clone https://github.com/Aldoubt/agt_robot_hmi.git

cd ~/agt_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

本轮代码尚未在真实 Humble 目标机执行完整 `colcon build`，所以 README 中的 🟡 都表示“代码已落地，但尚未宣称 hardware PASS”。

---

## 5. 使用方法

### 5.1 INS / RTK

```bash
ros2 launch agt_asensing_driver asensing.launch.py
ros2 launch agt_rtk_manager rtk_manager.launch.py
ros2 topic echo /agt/rtk/status
```

地理锚点示例：

```text
src/agt_rtk_manager/config/map_origin.example.yaml
```

配置 active map 的 `map_origin_file` 后可输出 `/agt/rtk/map_pose`。

### 5.2 FAST-LIO adapter

先确保选定 FAST-LIO2 已经运行，再根据实际 fork 修改：

```text
src/agt_fastlio_adapter/config/adapter.yaml
```

默认契约：

```text
input_topic:         /Odometry
expected_odom_frame: odom
expected_base_frame: base_link
output_topic:        /agt/odometry/local
```

启动：

```bash
ros2 launch agt_fastlio_adapter adapter.launch.py
```

adapter 不会偷偷重命名错误 frame；frame 或时间戳不符合契约时会拒绝转发。`odom -> base_link` TF 必须由选定并验证过的 FAST-LIO2 路径提供，且只能有一个发布者。

### 5.3 Localization Manager

```bash
ros2 launch agt_localization_manager localization_manager.launch.py
ros2 topic echo /agt/localization/status
```

全局重定位 backend 需要向：

```text
/agt/relocalization/pose
```

发布 `geometry_msgs/PoseWithCovarianceStamped`，frame 必须是 `map`，timestamp 必须对应实际匹配 scan 时刻，covariance 不能伪造为零。

手动触发重新定位：

```bash
ros2 service call /agt/localization/relocalize std_srvs/srv/Trigger {}
```

Manager 会清除当前 global correction、进入 `RELOCALIZING`，并发布：

```text
/agt/relocalization/request
```

供后续 global localization backend 消费。

### 5.4 Bunker

```bash
ros2 launch agt_bunker_base bunker_base.launch.py
ros2 launch agt_base_control cmd_vel_guard.launch.py
```

关键约束：

```text
agt_bunker_base publish_odom_tf = false
/wheel/odom != normal-mode Nav2 localization source
/mux/cmd_vel only receives guarded commands
```

### 5.5 C1 云台相机

按照 `Autolabor-C1-ROS2` 自己的 Phase-1 bringup 启动。本仓库只依赖：

```text
/camera_gimbal/acquire_view
/camera_gimbal/health
```

### 5.6 Nav2

不启动 AMCL，因为 `map -> odom` 由 3D LiDAR localization 管理。

```bash
ros2 launch agt_nav2_bringup navigation.launch.py \
  map:=/absolute/path/to/maps/site_A/navigation/map.yaml
```

发送 goal 前必须已有：

```text
valid map -> odom
valid odom -> base_link
/agt/odometry/local
/agt/navigation/points_obstacles
```

当前 baseline：

```text
controller_frequency = 50 Hz
controller            = Regulated Pure Pursuit
local costmap          = 3D VoxelLayer
planner                = SmacPlanner2D
```

### 5.7 巡检 Runtime

```bash
ros2 launch agt_navigation_runtime runtime.launch.py
```

主链：

```text
HMI task
 -> NavigateToPose
 -> base settle
 -> AcquireView
 -> image_stamp
 -> map pose + RTK + gimbal association
 -> image + CSV + JSONL
 -> next view / next point
```

默认记录：

```text
~/.ros/agt_inspection_records/
```

### 5.8 分阶段整机启动

安全默认：

```bash
ros2 launch agt_system_bringup system.launch.py
```

默认只启用 RTK manager 和 base guard。FAST-LIO adapter、Localization Manager、Nav2、mission runtime 默认关闭，避免传感器/定位尚未 ready 时底盘进入导航状态。

FAST-LIO2 frame/topic 已验证且 global backend 已启动后：

```bash
ros2 launch agt_system_bringup system.launch.py \
  enable_fastlio_adapter:=true \
  enable_localization_manager:=true
```

定位和障碍点云都 ready 后再启用 Nav2：

```bash
ros2 launch agt_system_bringup system.launch.py \
  enable_fastlio_adapter:=true \
  enable_localization_manager:=true \
  enable_nav2:=true \
  map:=/absolute/path/to/maps/site_A/navigation/map.yaml
```

C1 也 ready 后再增加：

```text
enable_runtime:=true
```

真正的自动 readiness gate 仍待后续 `agt_system_manager` 落地。

---

## 6. ROS 接口约定

```text
Localization
  FAST-LIO source                    configurable nav_msgs/Odometry
  /agt/odometry/local                nav_msgs/Odometry
  /agt/relocalization/pose           geometry_msgs/PoseWithCovarianceStamped
  /agt/relocalization/request        std_msgs/Empty
  /agt/localization/status           agt_robot_interfaces/LocalizationStatus
  /agt/localization/relocalize       std_srvs/Trigger
  map -> odom                        TF, agt_localization_manager only

RTK
  /ins/navsatfix                     sensor_msgs/NavSatFix
  /ins/status                        agt_asensing_driver/INSStatus
  /agt/rtk/status                    agt_robot_interfaces/RTKStatus
  /agt/rtk/map_pose                  geometry_msgs/PoseWithCovarianceStamped

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

## 7. 改造进度

| 模块 | 状态 | 当前情况 |
| --- | --- | --- |
| 架构 / TF / Map policy | ✅ | 设计原则已固定，参数仍随 benchmark 更新 |
| `agt_robot_interfaces` | 🟡 | mission / inspection / RTK / localization typed interfaces 已落地 |
| `agt_fastlio_adapter` | 🟡 | frame + timestamp gate 已落地，待选定 FAST-LIO2 fork 实测 |
| `agt_localization_manager` | 🟡 | map->odom 唯一 owner、时间匹配和 LOST/relocalization 状态机已落地 |
| HMI compatibility bridge | 🟡 | `/agt/task/*` 已接 runtime |
| Inspection runtime | 🟡 | Nav2 + C1 + image_stamp 数据闭环已落地 |
| RTK quality/freshness | 🟡 | FIX/FLOAT/失效质量门控已落地 |
| WGS84 -> ECEF -> ENU -> map | 🟡 | 已实现，待真实 map anchor 标定 |
| Nav2 bringup | 🟡 | no AMCL、RPP/Smac baseline、50 Hz 配置已落地 |
| Bunker 50 Hz guard | 🟡 | `/cmd_vel_smoothed -> /mux/cmd_vel` 已落地 |
| Bunker CAN driver | ✅ external | 直接复用 `agt_bunker_base` |
| C1 camera-gimbal | ✅ external capability | runtime 已按公开 Action 对接 |
| Qt HMI | ✅ external base | 不重写，继续做正式 interface 迁移 |
| Staged system bringup | 🟡 | 手动安全门已落地，自动 readiness manager 待实现 |
| Timing-safe MID360 split | 🔴 | 下一批：LIO 时序链与 obstacle branch 分离 |
| 3D global relocalization backend adapter | 🔴 | 下一批：Scan Context/coarse/GICP/validation 接口化 |
| Map Manager V1 | 🔴 | discovery/schema/hash/version/atomic switch/rollback |
| Terrain converter V1 | 🔴 | PCD -> elevation/slope/roughness/traversability |
| Power-cycle mission resume | 🔴 | checkpoint + relocalize + map validation + continue |
| Full target-machine acceptance | 🔴 | 尚未执行 Humble + rosbag + Bunker 实机验收 |

状态：✅ 可复用资产；🟡 代码已落地但未宣称实机 PASS；🔴 下一阶段代码。

---

## 8. 接下来持续落地顺序

1. **Timing-safe MID360 split / pointcloud preprocessing node**；
2. **3D global relocalization backend adapter**，接 `/agt/relocalization/request` 和 `/agt/relocalization/pose`；
3. **Map Manager V1**：Map Package schema/hash/version/atomic switch；
4. **Terrain converter V1**：elevation + slope + roughness + traversability；
5. **System readiness manager**：sensor -> map -> local odom -> global localization -> Nav2 -> mission；
6. **Power-cycle resume**：重新定位后继续未完成巡检点；
7. **Bunker controller benchmark**：RPP / MPPI / DWB 实车对比并冻结参数。

---

## 9. 版本冻结策略

当前不过早冻结所有外部仓库。端到端 Demo 和 acceptance dataset 稳定后再记录：

```text
repository URL
commit SHA
driver / firmware version
sensor calibration version
map schema version
```

最终可复现版本由 workspace dependency manifest + Map Package metadata 共同确定。
