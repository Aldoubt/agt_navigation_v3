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
地图自检
    ↓
3D LiDAR localization + Nav2
    ↓
RViz 依次点选 N 个巡检点
    ↓
/agt/rviz_patrol/start
    ↓
每点：Nav2 SUCCESS -> 实测速度连续归零 -> 额外稳定等待
    ↓
三个前向/仰视模板拍照
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
- Nav2 返回 SUCCESS 不等于“已经适合拍照”；必须再用 `/agt/odometry/local` 实测底盘静止。
- 50 Hz 是端到端控制要求：Nav2/velocity smoother/guard/CAN 刷新链路都要实测。

## 当前代码结构

```text
src/
├── agt_robot_interfaces/       公共 typed interfaces
├── agt_fastlio_adapter/        FAST-LIO2 odom frame/timestamp gate
├── agt_localization_manager/   唯一 map->odom owner
├── agt_rtk_manager/            RTK quality gate
├── agt_map_converter/          PCD -> Nav2 + elevation/slope/obstacle + validator
├── agt_nav2_bringup/           Nav2 Humble baseline, no AMCL
├── agt_base_control/           Bunker 50 Hz cmd_vel guard
├── agt_navigation_runtime/     NavigateToPose -> 停稳门禁 -> C1 -> recorder
├── agt_rviz_patrol/            RViz 点选队列 -> mission -> RETURN_HOME
└── agt_system_bringup/         staged bringup + RViz Demo bringup
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

# RViz Demo 操作流程

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

Demo V1 使用每个 XY cell 的最低/最高 Z、点数和 elevation gradient 估算地面、高差和坡度。单格高差超过 `max-step` 或坡度超过 `max-slope-deg` 会直接烘焙成 occupied。

当前支持 ASCII 和 binary uncompressed PCD；`binary_compressed` 请先转换。

`0.10 m / 0.22 m / 20°` 都只是起始值，必须用真实 Bunker 场地 PCD 调参。后续再升级 roughness、ground confidence 和 chassis-aware terrain layer。

## 3. 转换结果自检

```bash
ros2 run agt_map_converter validate_nav_map \
  /absolute/path/to/maps/site_A/navigation
```

检查：

- `map.yaml` 可解析；
- resolution/origin/threshold 合法；
- 所有 PGM 存在且尺寸一致；
- PGM payload 完整；
- map 中同时有 free / occupied cell；
- 打印 free / occupied / unknown 比例。

必须看到：

```text
MAP VALIDATION PASS
```

这不替代 RViz 人工检查。仍需观察道路、坡地、路沿、树木和摘边区域有没有明显错误。

## 4. 手动启动硬件和定位前置链

当前阶段不做自动 readiness，所以硬件相关模块仍显式手动启动和确认。

RTK/INS driver：

```bash
ros2 launch agt_asensing_driver asensing.launch.py
```

Bunker driver：

```bash
ros2 launch agt_bunker_base bunker_base.launch.py
```

确认：

```text
publish_odom_tf=false
/mux/cmd_vel 可用
/wheel/odom 正常
```

FAST-LIO2 / global localization 按当前实机配置启动，并确认：

```text
map -> odom
odom -> base_link
/agt/odometry/local
/agt/navigation/points_obstacles
```

都正常且时间连续。

C1 按 `Autolabor-C1-ROS2` 自己的 Phase-1 bringup 启动，并确认：

```text
/camera_gimbal/acquire_view
/camera_gimbal/health
```

可用。

## 5. 一键启动 RViz Demo 上层

前置硬件/定位确认完成后：

```bash
ros2 launch agt_system_bringup rviz_demo.launch.py \
  map:=/absolute/path/to/maps/site_A/navigation/map.yaml \
  map_id:=site_A_v1
```

默认拉起：

```text
agt_rtk_manager
agt_nav2_bringup
agt_base_control
agt_navigation_runtime
agt_rviz_patrol
rviz2
```

可选：

```text
enable_rtk:=false
launch_rviz:=false
```

这个 launch 不会自动判断 FAST-LIO2 / localization / C1 / Bunker hardware 是否 ready。当前阶段故意保留人工确认。

## 6. 人工 preflight

在 RViz 点巡检任务前运行一次：

```bash
ros2 run agt_navigation_runtime demo_preflight
```

它只检查一次并退出，不是自动 readiness 状态机。

默认检查：

```text
/navigate_to_pose action
/camera_gimbal/acquire_view action
/agt/odometry/local
/agt/navigation/points_obstacles
map -> base_link TF
```

要求 RTK 也必须有效时：

```bash
ros2 run agt_navigation_runtime demo_preflight --ros-args \
  -p require_rtk:=true
```

全部通过时看到：

```text
DEMO PREFLIGHT PASS
```

若 FAIL，先修复对应链路再点任务。该工具不会自动启动、停止或放行任何模块。

## 7. 三视角模板

默认模板：

```text
src/agt_rviz_patrol/config/front_sky_three_views.yaml
```

第一轮：

```text
front_left_sky    heading -45°  pitch +35°
front_center_sky  heading   0°  pitch +45°
front_right_sky   heading +45°  pitch +35°
```

第一次实机必须单独确认 C1 pitch 正负方向。如果正值实际朝下，只修改 YAML 里三个 `pitch` 的符号，不改任务代码。

## 8. RViz 点选

RViz：

1. `Fixed Frame = map`；
2. 显示 `/map`、RobotModel/TF；
3. 添加 `MarkerArray`，topic `/agt/rviz_patrol/markers`；
4. 用工具栏 **2D Goal Pose** 依次点击巡检点并设置车头方向。

每个 `/goal_pose` 只进入队列，不立即执行。点击顺序就是 P001、P002、P003……

开始任务时 `agt_rviz_patrol` 抓取当前 `map -> base_link` 作为 HOME，并在 mission 尾部自动追加 `RETURN_HOME`，该点只导航、不拍照。

## 9. 开始 / 清空 / 取消

开始：

```bash
ros2 service call /agt/rviz_patrol/start std_srvs/srv/Trigger "{}"
```

清空未执行队列：

```bash
ros2 service call /agt/rviz_patrol/clear std_srvs/srv/Trigger "{}"
```

取消当前任务：

```bash
ros2 service call /agt/rviz_patrol/cancel std_srvs/srv/Trigger "{}"
```

## 10. 到点停稳门禁

Nav2 `NavigateToPose` 返回 SUCCESS 后，runtime 不直接调用相机，而是继续读取 `/agt/odometry/local`。

默认：

```text
linear speed  <= 0.03 m/s
angular speed <= 0.05 rad/s
连续满足       0.8 s
odom 数据新鲜 <= 0.5 s
最大等待       8.0 s
```

满足后再执行 point 自身额外 settle time，最后才调用 C1。

参数：

```text
src/agt_navigation_runtime/config/runtime.yaml
```

如果 Bunker 静止时 FAST-LIO2 twist 噪声高，优先根据静止 rosbag 调阈值，不要直接移除停稳门禁。

## 11. 每个巡检点执行顺序

```text
NavigateToPose
    ↓
Nav2 SUCCESS
    ↓
测得底盘持续静止
    ↓
额外 settle
    ↓
front_left_sky
    ↓
front_center_sky
    ↓
front_right_sky
    ↓
每张图记录 image_stamp 对应：
  map pose
  RTK
  actual gimbal heading/roll/pitch
    ↓
next point
```

最后：

```text
RETURN_HOME -> measured stop -> mission COMPLETED -> standby
```

## 12. 数据记录

默认：

```text
~/.ros/agt_inspection_records/
```

每次任务包含：

```text
mission.yaml
manifest.json
captures.csv
captures.jsonl
images/
```

每张成功照片记录 mission/map/point/view ID、image path、image_stamp、拍照时刻 map pose、RTK 经纬高/status/time delta、云台 actual heading/roll/pitch 和 camera error code。

RViz 临时 mission：

```text
~/.ros/agt_rviz_patrol/
```

# 验收

## A. 单点

只点 1 个巡检点：

```text
P001 -> 3 张图 -> RETURN_HOME
```

找到最新目录：

```bash
ls -dt ~/.ros/agt_inspection_records/* | head -1
```

检查：

```bash
ros2 run agt_navigation_runtime validate_records \
  /path/to/latest/mission_dir \
  --expected-points 1
```

要求 RTK 每张都有效时加：

```text
--require-rtk
```

通过：

```text
DEMO RECORD VALIDATION PASS: points=1 views_per_point=3
```

`RETURN_HOME` 不产生 capture，因此不计入 expected points。

## B. 三点

```text
P001 -> 3 图
P002 -> 3 图
P003 -> 3 图
RETURN_HOME
```

检查：

```bash
ros2 run agt_navigation_runtime validate_records \
  /path/to/latest/mission_dir \
  --expected-points 3
```

预期总 capture 数：9。

## C. 验收器检查项

- capture 数量；
- 每点是否正好 3 个视角；
- 图片文件真实存在；
- `pose_valid=true`；
- gimbal actual heading/roll/pitch 存在；
- `camera_error_code=0`；
- `--require-rtk` 时每张 `rtk_valid=true`。

验收器不判断照片内容。首轮仍需人工确认覆盖方向和曝光时底盘是否视觉上稳定。

# 当前 ROS 接口

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
  /agt/odometry/local                nav_msgs/Odometry, stop gate source

Camera
  /camera_gimbal/acquire_view        C1 AcquireView action

RTK
  /ins/navsatfix                     sensor_msgs/NavSatFix
  /agt/rtk/status                    agt_robot_interfaces/RTKStatus

Obstacle
  /agt/navigation/points_obstacles   sensor_msgs/PointCloud2

Control
  /cmd_vel                           Nav2 final smoother output -> guard input
  /mux/cmd_vel                       guarded Bunker command
  /wheel/odom                        diagnostics/control reference only
```

# 当前改造进度

| 模块 | 状态 | 当前情况 |
| --- | --- | --- |
| 架构 / TF policy | ✅ | 主原则已固定 |
| `agt_map_converter` | 🟡 | PCD -> map/elevation/slope/obstacle + validator 已落地 |
| Nav2 bringup | 🟡 | no AMCL、RPP/Smac baseline 已落地 |
| Bunker 50 Hz guard | 🟡 | `/cmd_vel -> /mux/cmd_vel` 已落地 |
| Inspection runtime | 🟡 | NavigateToPose + measured-stop gate + C1 + image_stamp record 已落地 |
| `agt_rviz_patrol` | 🟡 | RViz queue + 三视角 + RETURN_HOME 已落地 |
| Manual demo preflight | 🟡 | 一次性 Action/topic/TF 检查已落地 |
| Demo record validator | 🟡 | 单点/三点 capture 自动检查已落地 |
| RViz Demo bringup | 🟡 | Nav2/RTK/guard/runtime/patrol/RViz 一键拉起已落地 |
| Validator unit tests | 🟡 | map/record pure-software tests 已落地，待 Humble CI 实跑 |
| RTK manager | 🟡 | quality/freshness + 记录链已落地 |
| FAST-LIO2 adapter | 🟡 | 待选定 fork 实机验证 |
| Localization Manager | 🟡 | global backend 仍需实测接通 |
| C1 camera-gimbal | ✅ external | 复用公开 AcquireView capability |
| HMI integration | ⏸️ | 等 RViz Demo 稳定再接 |
| Auto readiness manager | ⏸️ | 当前阶段明确不做 |
| Advanced terrain layer | 🔴 | 后续 roughness/ground confidence/custom costmap |
| Power-cycle resume | 🔴 | RViz Demo 稳定后再做 |
| Full hardware acceptance | 🔴 | 需要 Humble + rosbag + Bunker + C1 实机 |

# 当前验收顺序

1. PCD -> Nav2 map；
2. `validate_nav_map` PASS；
3. RViz 人工检查地图；
4. 手动启动硬件/定位前置链；
5. `demo_preflight` PASS；
6. 单独 Nav2：1 个普通目标，确认 Bunker 到点和停车；
7. 单独 C1 三视角，确认 pitch 方向、稳定判定和新图像；
8. 1 个巡检点 -> 3 图 -> 返回 HOME；
9. `validate_records --expected-points 1` PASS；
10. 3 个巡检点 -> 9 图 -> 返回 HOME；
11. `validate_records --expected-points 3` PASS；
12. 扩展完整路径；
13. RViz 链稳定后再正式接 `agt_robot_hmi`。

当前开发主线就是先把第 1~11 步做稳定。
