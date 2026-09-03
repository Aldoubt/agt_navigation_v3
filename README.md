# agt_navigation_v3

AGT 户外履带式巡检机器人 ROS 2 Humble 导航与巡检集成仓库。

目标平台：**Bunker v1 + 倾斜 MID360 + FAST-LIO2 + 3D 全局重定位 + Nav2 + RTK/INS 辅助 + Autolabor C1 云台相机**。

> 当前开发分支：`runtime-v1`
>
> 当前阶段只追求一条可重复、可在 RViz 中人工操作的最小闭环。Qt HMI 和纯自动 readiness 状态机暂缓，等 RViz 闭环稳定后再接入。

## 当前阶段目标

```text
FAST-LIO2 一次建图
    ↓
导出 global_map.pcd
    ↓
agt_map_converter
    ↓
map.yaml + map.pgm + elevation/slope/obstacle
    ↓
3D LiDAR localization + Nav2
    ↓
RViz 依次点选 N 个巡检点
    ↓
/agt/rviz_patrol/start
    ↓
每点：停稳 -> 三个前向/仰视模板拍照
    ↓
记录 image_stamp 对应的 map pose + RTK + 云台实际角度
    ↓
最后一个点完成
    ↓
自动返回启动时记录的 HOME
    ↓
待机
```

本阶段明确不做 HMI 正式任务下发、纯自动 readiness、断电续巡和最终 controller 冻结。

## 设计原则

- LiDAR 定位是主链，RTK 只做辅助、地理锚点、记录和未来全局约束。
- FAST-LIO2 是高频局部运动主源，`agt_localization_manager` 是唯一 `map -> odom` owner。
- Bunker `/wheel/odom` 不发布竞争性的 odom TF，`agt_bunker_base publish_odom_tf=false`。
- MID360 实际倾角保留在 URDF/TF，不在 FAST-LIO2 前把点云拉平。
- FAST-LIO2 timing-preserving 输入和 self-filter/downsample/Nav2 obstacle 分支分开。
- C1 只通过 `/camera_gimbal/acquire_view` capability 接入。
- 每张照片以 `image_stamp` 为主同步时间，关联 map pose、RTK 和云台实际关节角。
- 50 Hz 是端到端控制要求：Nav2/velocity smoother/guard/CAN 刷新链路都要实测。

## 当前代码结构

```text
src/
├── agt_robot_interfaces/       公共 typed interfaces
├── agt_fastlio_adapter/        FAST-LIO2 odom frame/timestamp gate
├── agt_localization_manager/   唯一 map->odom owner
├── agt_rtk_manager/            RTK quality gate
├── agt_map_converter/          PCD -> Nav2 + elevation/slope/obstacle
├── agt_nav2_bringup/           Nav2 Humble baseline, no AMCL
├── agt_base_control/           Bunker 50 Hz cmd_vel guard
├── agt_navigation_runtime/     NavigateToPose -> C1 -> recorder
├── agt_rviz_patrol/            RViz 点选队列 -> mission -> RETURN_HOME
└── agt_system_bringup/         手动分阶段 bringup；暂不做自动 readiness
```

## 构建

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

当前分支还没有在真实 Humble 目标机完成整仓 `colcon build` + rosbag + Bunker 实机验收，因此“代码已实现”不等于 hardware PASS。

## 1. 一次建图

使用选定的 FAST-LIO2 + MID360 完成一次人工建图，最终导出：

```text
global_map.pcd
```

建图时保持 MID360 实际倾角 TF，不做软件 level。具体 FAST-LIO2 launch 命令等最终 fork 确认后冻结。

## 2. PCD 转 Nav2 地图

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

Demo V1 使用每个 XY cell 的最低/最高 Z、点数和 elevation gradient 估算地面、高差和坡度。单格高差超过 `max-step` 或坡度超过 `max-slope-deg` 会直接烘焙成 occupied。当前支持 ASCII 和 binary uncompressed PCD；`binary_compressed` 请先转换。

`0.10 m / 0.22 m / 20°` 都只是起始值，必须用真实 Bunker 场地 PCD 调参。后续再升级 roughness、ground confidence 和 chassis-aware terrain layer。

## 3. 启动 RTK / Bunker / 定位 / C1 / Nav2

RTK：

```bash
ros2 launch agt_asensing_driver asensing.launch.py
ros2 launch agt_rtk_manager rtk_manager.launch.py
```

Bunker：

```bash
ros2 launch agt_bunker_base bunker_base.launch.py
ros2 launch agt_base_control cmd_vel_guard.launch.py
```

FAST-LIO2 / localization：

```bash
ros2 launch agt_fastlio_adapter adapter.launch.py
ros2 launch agt_localization_manager localization_manager.launch.py
```

Nav2 前确认已有正确：

```text
map -> odom
odom -> base_link
```

C1 按 `Autolabor-C1-ROS2` 自己的 Phase-1 bringup 启动，并确认：

```text
/camera_gimbal/acquire_view
/camera_gimbal/health
```

Nav2：

```bash
ros2 launch agt_nav2_bringup navigation.launch.py \
  map:=/absolute/path/to/maps/site_A/navigation/map.yaml
```

当前 baseline 是 RPP + SmacPlanner2D，控制频率 50 Hz；最终 controller 实车 benchmark 后再冻结。

## 4. 启动 RViz 巡检 Demo

```bash
ros2 launch agt_navigation_runtime runtime.launch.py
ros2 launch agt_rviz_patrol rviz_patrol.launch.py map_id:=site_A_v1
```

默认三视角模板：

```text
src/agt_rviz_patrol/config/front_sky_three_views.yaml
```

默认：

```text
front_left_sky    heading -45°  pitch +35°
front_center_sky  heading   0°  pitch +45°
front_right_sky   heading +45°  pitch +35°
```

第一次实机必须确认 C1 pitch 正负方向。如果正值实际朝下，只改 YAML 里三个 `pitch` 的符号，不改代码。

## 5. RViz 点选和执行

RViz：

1. `Fixed Frame = map`；
2. 显示 `/map`、RobotModel/TF；
3. 添加 `MarkerArray`，topic 为 `/agt/rviz_patrol/markers`；
4. 用工具栏 **2D Goal Pose** 依次点击巡检点并设置车头方向。

每个 `/goal_pose` 只进入队列。点击顺序就是任务顺序：第 1 个=P001，第 2 个=P002，以此类推。

开始前节点会抓取当前 `map -> base_link` 作为 HOME，并自动追加一个不拍照的 `RETURN_HOME`。

开始：

```bash
ros2 service call /agt/rviz_patrol/start std_srvs/srv/Trigger "{}"
```

执行：

```text
P001
 -> NavigateToPose
 -> base settle
 -> left sky capture
 -> center sky capture
 -> right sky capture
 -> P002
 -> ...
 -> PN
 -> RETURN_HOME
 -> mission complete / standby
```

清空未执行队列：

```bash
ros2 service call /agt/rviz_patrol/clear std_srvs/srv/Trigger "{}"
```

取消当前任务：

```bash
ros2 service call /agt/rviz_patrol/cancel std_srvs/srv/Trigger "{}"
```

## 6. 数据记录

默认：

```text
~/.ros/agt_inspection_records/
```

每张成功照片记录：

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
camera error code
```

RViz 自动生成的临时 mission 保存在：

```text
~/.ros/agt_rviz_patrol/
```

## 7. 当前 ROS 接口

```text
RViz patrol
  /goal_pose                         geometry_msgs/PoseStamped
  /agt/rviz_patrol/markers           visualization_msgs/MarkerArray
  /agt/rviz_patrol/status            std_msgs/String
  /agt/rviz_patrol/start             std_srvs/Trigger
  /agt/rviz_patrol/clear             std_srvs/Trigger
  /agt/rviz_patrol/cancel            std_srvs/Trigger

Mission
  /agt/mission/execute               ExecuteInspectionMission action

Camera
  /camera_gimbal/acquire_view        C1 AcquireView action

RTK
  /ins/navsatfix                     sensor_msgs/NavSatFix
  /agt/rtk/status                    agt_robot_interfaces/RTKStatus

Control
  /cmd_vel                           Nav2 final smoothed command
  /mux/cmd_vel                       guarded Bunker command
  /wheel/odom                        diagnostics/control reference only
```

## 8. 当前改造进度

| 模块 | 状态 | 当前情况 |
| --- | --- | --- |
| 架构 / TF policy | ✅ | 主原则已固定 |
| `agt_map_converter` | 🟡 | PCD -> map/elevation/slope/obstacle V1 已落地 |
| Nav2 bringup | 🟡 | no AMCL、RPP/Smac baseline 已落地 |
| Bunker 50 Hz guard | 🟡 | `/cmd_vel -> /mux/cmd_vel` 已落地 |
| Inspection runtime | 🟡 | NavigateToPose + C1 + image_stamp record 已落地 |
| `agt_rviz_patrol` | 🟡 | RViz queue + 三视角 + RETURN_HOME 已落地 |
| RTK manager | 🟡 | quality/freshness + 记录链已落地 |
| FAST-LIO2 adapter | 🟡 | 待选定 fork 实机验证 |
| Localization Manager | 🟡 | global backend 仍需实测接通 |
| C1 camera-gimbal | ✅ external | 复用公开 AcquireView capability |
| HMI integration | ⏸️ | 等 RViz Demo 稳定再接 |
| Auto readiness manager | ⏸️ | 当前阶段明确不做 |
| Advanced terrain layer | 🔴 | 后续 roughness/ground confidence/custom costmap |
| Power-cycle resume | 🔴 | RViz Demo 稳定后再做 |
| Full hardware acceptance | 🔴 | 需要 Humble + rosbag + Bunker + C1 实机 |

## 9. 当前验收顺序

1. PCD -> Nav2 map，检查坡地/路沿/树木投影；
2. 单独 Nav2：RViz 发 1 个点，确认 Bunker 稳定到点和停车；
3. 单独 C1 三视角：确认 pitch 方向、稳定判定和新图像；
4. 单巡检点：1 点 -> 3 张图 -> RTK/角度/pose -> 回 HOME；
5. 三巡检点：3 点 -> 9 张图 -> 回 HOME；
6. 扩展完整路径；
7. RViz 链稳定后再正式接 `agt_robot_hmi`。

当前开发主线就是先把第 1~5 步做稳定。