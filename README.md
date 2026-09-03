# agt_navigation_v3

AGT 户外履带式巡检机器人 ROS 2 Humble 导航与巡检集成仓库。

目标平台：**Bunker v1 + 倾斜 MID360 + FAST-LIO2 + 3D 全局重定位 + Nav2 + RTK/INS 辅助 + Autolabor C1 云台相机**。

> 当前开发分支：`runtime-v1`
>
> 当前阶段只追求一条可重复、可在 RViz 中人工操作的最小闭环。Qt HMI 和纯自动 readiness 状态机暂缓，等 RViz 闭环稳定后再接入。

---

## 1. 当前阶段目标：RViz 最小巡检闭环

第一轮必须跑通：

```text
FAST-LIO2 一次建图
    ↓
导出 global_map.pcd
    ↓
agt_map_converter
    ↓
map.yaml + map.pgm
+ elevation/slope/obstacle layers
    ↓
3D LiDAR localization + Nav2
    ↓
RViz 依次点选 N 个巡检点
    ↓
/agt/rviz_patrol/start
    ↓
P001 -> 停稳 -> 三个前向/仰视模板拍照
    ↓
记录 image_stamp 对应的 map pose + RTK + 云台实际角度
    ↓
P002 -> ... -> PN
    ↓
自动返回启动时记录的 HOME
    ↓
待机
```

本阶段明确**不做**：

- HMI 正式任务编辑与下发；
- 自动 sensor/map/localization/Nav2 readiness 编排；
- 自动断电续巡；
- RTK 直接接管导航；
- controller 最终选型冻结；
- terrain costmap 自定义插件。

这些都在 RViz Demo 稳定后继续。

---

## 2. 核心设计原则

### 2.1 LiDAR 定位是主链

树荫环境下 RTK 可能退化或丢失，所以 RTK 不能成为导航连续运行的前提。

```text
3D Global Relocalization
        |
        v
agt_localization_manager ---- owns map -> odom
        ^
        |
FAST-LIO2 ------------------- owns high-rate local motion
```

约束：

- FAST-LIO2 是高频局部运动主源；
- `agt_localization_manager` 是唯一 `map -> odom` owner；
- Bunker `/wheel/odom` 只用于控制/诊断，不发布竞争性的 odom TF；
- 不默认再用 `robot_localization` 重复融合 FAST-LIO2 已经使用过的同一 IMU；
- RTK 只做质量门控、地理锚点、记录和未来全局优化约束。

### 2.2 MID360 时序链和导航点云分开

不要把普通 voxel/downsample 节点无验证地放在 FAST-LIO2 前面。

```text
MID360 raw/custom message + IMU
        |
        +------------------> FAST-LIO2 timing-preserving path
        |
        +--> self-filter/downsample
                  |
                  +--> global relocalization
                  +--> Nav2 obstacle cloud
                  +--> debug
```

倾斜安装保留在 URDF/TF 中，不人为把点云拉平。

### 2.3 图片时间戳是巡检记录主锚点

C1 只通过已经冻结的：

```text
/camera_gimbal/acquire_view
```

调用。每次成功拍照后使用 C1 返回的 `image_stamp`：

```text
image_stamp
   +-> lookup(map -> base_link)
   +-> nearest RTK sample
   +-> actual gimbal angles
   +-> saved image path
```

因此记录的是“真正拍照时刻”的机器人状态，而不是到点时随便取一次最新状态。

### 2.4 50 Hz 是端到端控制约束

```text
Nav2 controller @ 50 Hz
        |
velocity_smoother @ 50 Hz
        |
     /cmd_vel
        |
agt_cmd_vel_guard @ 50 Hz
 clamp + slew + stale watchdog
        |
   /mux/cmd_vel
        |
agt_bunker_base -> CAN
```

`agt_bunker_base` 必须保持 `publish_odom_tf=false`。

RPP 只是当前 baseline，RPP / MPPI / DWB 等最终用实车 benchmark 决定。

---

## 3. 当前代码结构

```text
agt_navigation_v3/
├── config/
├── docs/
└── src/
    ├── agt_robot_interfaces/       公共 typed ROS interfaces
    ├── agt_fastlio_adapter/        FAST-LIO2 odom frame/timestamp gate
    ├── agt_localization_manager/   唯一 map->odom owner
    ├── agt_rtk_manager/            RTK quality gate + map observation
    ├── agt_map_converter/          PCD -> Nav2 + elevation/slope/obstacle
    ├── agt_nav2_bringup/           Nav2 Humble baseline, no AMCL
    ├── agt_base_control/           Bunker 50 Hz cmd_vel guard
    ├── agt_navigation_runtime/     NavigateToPose -> C1 -> recorder
    ├── agt_rviz_patrol/            RViz 点选队列 -> mission -> RETURN_HOME
    └── agt_system_bringup/         手动分阶段 bringup；暂不做自动 readiness
```

---

## 4. 外部仓库

建议放入同一个 colcon workspace：

```text
https://github.com/Aldoubt/agt_navigation_v3.git
https://github.com/Aldoubt/agt_ins_driver.git
https://github.com/Aldoubt/agt_bunker_base.git
https://github.com/Aldoubt/agt_chassis_description.git
https://github.com/Aldoubt/Autolabor-C1-ROS2.git
```

`agt_robot_hmi` 当前不参加 RViz Demo；RViz 链稳定后再接回：

```text
https://github.com/Aldoubt/agt_robot_hmi.git
```

---

## 5. 构建

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

cd ~/agt_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

当前分支还没有在你的真实 Humble 目标机完成整仓 `colcon build` + rosbag + Bunker 实机验收，因此“代码已实现”不等于 hardware PASS。

---

## 6. 第一步：一次建图

使用选定的 FAST-LIO2 + MID360 完成一次人工建图，最终导出：

```text
global_map.pcd
```

建图时保持 MID360 实际倾角 TF，不做软件 level。

建图工具的启动命令取决于最终选定的 FAST-LIO2 ROS 2 fork，因此本仓库当前不硬编码一个假的 launch 命令。第一轮验收只要求最终 PCD 在 RViz/PCL 中几何正确、没有明显重影和错误 TF。

---

## 7. 第二步：PCD 转 Nav2 地图

第一版转换器：

```bash
ros2 run agt_map_converter pcd_to_nav_map \
  /absolute/path/to/global_map.pcd \
  --output /absolute/path/to/maps/site_A/navigation \
  --resolution 0.10 \
  --max-step 0.22 \
  --max-slope-deg 20.0
```

输出：

```text
navigation/
├── map.yaml
├── map.pgm
├── elevation.pgm
├── slope.pgm
├── obstacle.pgm
└── converter_metadata.yaml
```

Demo V1 做法：

- 每个 XY grid cell 统计最低/最高 Z 和点数；
- 最低 Z 作为第一版 elevation 近似；
- 根据 elevation gradient 计算 slope；
- 单格高度跨度超过 `max-step`，或 slope 超过 `max-slope-deg`，直接烘焙为占据栅格；
- elevation/slope/obstacle PGM 同时保留给调试和后续 terrain plugin。

注意：

- `0.22 m`、`20°`、`0.10 m` 都只是 Demo 起始值，必须用真实 Bunker/场地 PCD 调；
- 第一版不是完整地面分割器，树冠、路沿、悬空物体复杂场景要用 rosbag/PCD 验证；
- 当前支持 ASCII 和 binary uncompressed PCD；`binary_compressed` 请先转换；
- 后续再升级 roughness、ground confidence、chassis-aware traversability 和独立 Nav2 terrain costmap layer。

先检查：

```bash
ls /absolute/path/to/maps/site_A/navigation
```

然后用 RViz / image viewer 检查 `map.pgm`、`elevation.pgm`、`slope.pgm`、`obstacle.pgm` 是否符合场地直觉。

---

## 8. 第三步：启动定位 / 底盘 / C1 / Nav2

### 8.1 INS / RTK

```bash
ros2 launch agt_asensing_driver asensing.launch.py
ros2 launch agt_rtk_manager rtk_manager.launch.py
ros2 topic echo /agt/rtk/status
```

### 8.2 Bunker

```bash
ros2 launch agt_bunker_base bunker_base.launch.py
ros2 launch agt_base_control cmd_vel_guard.launch.py
```

确认：

```text
publish_odom_tf = false
/mux/cmd_vel receives only guarded command
```

### 8.3 FAST-LIO2 / localization

启动已经验证过的 FAST-LIO2 和当前 global localization backend，然后：

```bash
ros2 launch agt_fastlio_adapter adapter.launch.py
ros2 launch agt_localization_manager localization_manager.launch.py
ros2 topic echo /agt/localization/status
```

Nav2 前必须已经存在正确：

```text
map -> odom
odom -> base_link
```

### 8.4 C1 云台相机

按照 `Autolabor-C1-ROS2` 自己的 Phase-1 bringup 启动，并确认：

```bash
ros2 action list | grep camera_gimbal
ros2 topic echo /camera_gimbal/health
```

本仓库只依赖：

```text
/camera_gimbal/acquire_view
/camera_gimbal/health
```

### 8.5 Nav2

```bash
ros2 launch agt_nav2_bringup navigation.launch.py \
  map:=/absolute/path/to/maps/site_A/navigation/map.yaml
```

当前 baseline：

```text
controller_frequency = 50 Hz
controller            = Regulated Pure Pursuit
planner               = SmacPlanner2D
```

---

## 9. 第四步：启动 RViz 巡检 Demo

先启动任务执行 runtime：

```bash
ros2 launch agt_navigation_runtime runtime.launch.py
```

再启动 RViz waypoint adapter：

```bash
ros2 launch agt_rviz_patrol rviz_patrol.launch.py \
  map_id:=site_A_v1
```

默认三视角模板：

```text
src/agt_rviz_patrol/config/front_sky_three_views.yaml
```

第一版模板：

```text
front_left_sky    heading -45°  pitch +35°
front_center_sky  heading   0°  pitch +45°
front_right_sky   heading +45°  pitch +35°
```

**必须在第一次实机前确认 C1 的 pitch 正负方向。** 如果正值实际朝下，只需要把 YAML 中三个 `pitch` 改成负数，不改代码。

---

## 10. RViz 操作方法

RViz：

1. `Fixed Frame` 设为 `map`；
2. 显示 `/map`；
3. 显示机器人 TF / RobotModel；
4. 增加 `MarkerArray`，topic 选 `/agt/rviz_patrol/markers`；
5. 使用工具栏 **2D Goal Pose** 依次点击巡检点，并设置每个点的车头朝向。

本 Demo 中 `/goal_pose` **只进入队列，不立即启动整条巡检任务**。

每点击一次：

```bash
ros2 topic echo /agt/rviz_patrol/status
```

会看到队列增加，同时 RViz marker 显示已记录的点。

注意：点的顺序就是执行顺序：

```text
第 1 次点击 -> P001
第 2 次点击 -> P002
...
```

---

## 11. 开始 / 清空 / 取消巡检

开始前，`agt_rviz_patrol` 会抓取当前：

```text
map -> base_link
```

作为 HOME，并自动把 `RETURN_HOME` 加在最后一个巡检点之后。

开始：

```bash
ros2 service call /agt/rviz_patrol/start std_srvs/srv/Trigger "{}"
```

实际执行：

```text
P001
 -> Nav2 到点
 -> 底盘等待 base_settle_time
 -> front_left_sky: 云台稳定 -> 新图像
 -> front_center_sky: 云台稳定 -> 新图像
 -> front_right_sky: 云台稳定 -> 新图像
 -> P002
 -> ...
 -> PN
 -> RETURN_HOME
 -> 不拍照
 -> mission completed / 待机
```

清空尚未执行的 RViz 队列：

```bash
ros2 service call /agt/rviz_patrol/clear std_srvs/srv/Trigger "{}"
```

取消当前任务：

```bash
ros2 service call /agt/rviz_patrol/cancel std_srvs/srv/Trigger "{}"
```

取消会继续向当前 Nav2 / C1 子 Action 传递取消请求；这不是安全级急停，物理 E-stop 仍然独立。

---

## 12. 巡检数据记录

默认：

```text
~/.ros/agt_inspection_records/
```

每张成功图像记录：

```text
mission_id
map_id
point_id
view_tag
image_path
image_stamp
map pose at image_stamp
RTK latitude / longitude / altitude / status
RTK time difference
gimbal actual heading / roll / pitch
camera result code
```

RViz 生成的临时 mission 文件保存在：

```text
~/.ros/agt_rviz_patrol/
```

因此每次 Demo 都可以回看“当时在 RViz 点了哪些点、以什么顺序执行”。

---

## 13. ROS 接口约定

```text
Localization
  /agt/odometry/local                nav_msgs/Odometry
  /agt/relocalization/pose           geometry_msgs/PoseWithCovarianceStamped
  /agt/localization/status           agt_robot_interfaces/LocalizationStatus
  map -> odom                        agt_localization_manager only

RTK
  /ins/navsatfix                     sensor_msgs/NavSatFix
  /ins/status                        agt_asensing_driver/INSStatus
  /agt/rtk/status                    agt_robot_interfaces/RTKStatus

Control
  /cmd_vel                           Nav2 velocity smoother final output
  /mux/cmd_vel                       guarded Bunker command
  /wheel/odom                        Bunker wheel odom / diagnostics only

Inspection runtime
  /agt/mission/execute               ExecuteInspectionMission action
  /camera_gimbal/acquire_view        C1 AcquireView action

RViz patrol
  /goal_pose                         geometry_msgs/PoseStamped input queue
  /agt/rviz_patrol/markers           visualization_msgs/MarkerArray
  /agt/rviz_patrol/status            std_msgs/String
  /agt/rviz_patrol/start             std_srvs/Trigger
  /agt/rviz_patrol/clear             std_srvs/Trigger
  /agt/rviz_patrol/cancel            std_srvs/Trigger
```

---

## 14. 当前改造进度

| 模块 | 状态 | 当前情况 |
| --- | --- | --- |
| 架构 / TF policy | ✅ | 主原则已固定 |
| `agt_robot_interfaces` | 🟡 | typed contracts 已落地，待目标机编译 |
| FAST-LIO2 adapter | 🟡 | frame/timestamp gate 已落地，待选定 fork 实测 |
| Localization Manager | 🟡 | map->odom owner 已落地，global backend 仍需接实测 |
| RTK manager | 🟡 | quality/freshness + 记录链已落地 |
| `agt_map_converter` | 🟡 | PCD -> map/elevation/slope/obstacle V1 已落地 |
| Nav2 bringup | 🟡 | no AMCL、RPP/Smac baseline 已落地 |
| Bunker 50 Hz guard | 🟡 | `/cmd_vel -> /mux/cmd_vel` 已落地 |
| Inspection runtime | 🟡 | NavigateToPose + C1 + image_stamp record 已落地 |
| `agt_rviz_patrol` | 🟡 | RViz 点选队列 + 三视角 + RETURN_HOME 已落地 |
| C1 camera-gimbal | ✅ external | 复用公开 AcquireView capability |
| HMI integration | ⏸️ | 当前明确暂停，等 RViz Demo 稳定 |
| Auto readiness manager | ⏸️ | 当前明确暂停，不做纯自动启动编排 |
| Terrain advanced layer | 🔴 | 后续 roughness/ground confidence/custom costmap |
| Power-cycle resume | 🔴 | RViz Demo 稳定之后再做 |
| Full hardware acceptance | 🔴 | 需要 Humble + rosbag + Bunker + C1 实机 |

状态：✅ 可直接复用资产；🟡 代码已落地但未宣称实机 PASS；⏸️ 当前阶段主动延后；🔴 后续阶段。

---

## 15. 当前验收顺序

不要同时调所有模块。按以下顺序验收：

1. **PCD -> Nav2 map**：确认坡地/路沿/树木投影合理；
2. **单独 Nav2**：在 RViz 发送一个点，确认 Bunker 能稳定到点和停止；
3. **单独 C1 三视角**：确认 pitch 正负、云台稳定判定和新图像；
4. **单个 RViz 巡检点**：1 点 -> 3 张图 -> RTK/角度/pose 记录 -> 回 HOME；
5. **3 个 RViz 巡检点**：顺序执行 -> 每点 3 张 -> 回 HOME；
6. **扩展到完整巡检路径**；
7. RViz 链稳定后才开始正式接 `agt_robot_hmi`。

当前开发主线就是把第 1~5 步做稳定。