# NAV_TEST_002 离线导航审计

审计对象：`/home/yangxuan/ros2_ws/agt_data/field_acceptance/nav_test_002`  
审计方式：只读 rosbag / 只读当前配置和源码；未播放 bag，未修改源码、参数、地图或 Git 历史。  
结论日期：2026-09-13。

## 结论摘要

本 bag **不能复现“持续原地旋转”**。记录到的 `/cmd_vel` 以正向行驶为主，且 `/agt/odometry/local` 在 180.95 秒内累计航向仅变化 **+2.44°**。因此，不能根据此 bag 将持续绕圈归因于 Nav2 控制器、TF 延迟或原地旋转恢复行为。

不过，bag 确实显示 local costmap 在机器人 padded footprint 内出现最高值 `100` 的占用/致命代价：694 帧中 15 帧发生，集中在 bag 起点后 **103.995–108.899 s** 与 **120.697 s**。这证明该时段 local costmap 已将机器人自身 footprint 覆盖区域判为不可通行或近似不可通行，是应优先复现实物 LiDAR 自障碍的证据；但本包没有录制 `/agt/navigation/points_obstacles`，故尚不能从本包证明这些点就是后方安装杆。

## Bag inventory

| 项目 | 结果 |
|---|---|
| 存储 | sqlite3，`nav_test_002_0.db3`，849.5 MiB |
| 时间范围 | 180.948 s；2026-09-13 15:09:46.457 至 15:12:47.405 |
| 消息总数 | 14,216 |
| 规划 | `/plan`：16 条 |
| 控制命令 | `/cmd_vel`：803 条 |
| 本地里程计 | `/agt/odometry/local`：1,788 条 |
| Local costmap | `/local_costmap/costmap`：694 条，`odom` frame，8 m × 8 m，0.05 m/cell |
| Global costmap | `/global_costmap/costmap`：139 条，`map` frame，0.05 m/cell |
| LiDAR 原始输入 | `/livox/lidar`：1,797 条，`livox_frame`，Livox `CustomMsg` |
| TF | `/tf`：8,977 条；`/tf_static`：1 条 |

未录制：`/agt/navigation/points_obstacles`、`/cmd_vel_nav`、速度平滑器输出、`cmd_vel_guard` 输出、底盘反馈、controller action feedback/diagnostics、costmap voxel debug topic。因此该包不能区分“controller 原始输出”和“最终送到底盘的命令”，也不能在离线条件下把 obstacle cell 溯源到具体 LiDAR 点。

## 现象与证据

### 1. 控制命令与里程计不支持持续旋转假设

`/cmd_vel` 的 803 条命令中 793 条非零：

| 指标 | 结果 |
|---|---:|
| linear.x | min `0.000`，max `0.450`，mean `0.381 m/s` |
| angular.z | min `-0.5625`，max `+0.5286`，mean `+0.0011 rad/s` |
| angular.z > +0.02 | 466 条 |
| angular.z < -0.02 | 262 条 |
| `abs(linear.x) ≤ 0.02` 且 `abs(angular.z) ≥ 0.2` | **0 条** |
| `/agt/odometry/local` 累计平面路径长度 | `23.79 m` |
| odom 累计 yaw 变化 | **+2.44°** |
| odom yaw rate | p95(abs) `0.0746 rad/s`；max(abs) `0.4410 rad/s` |

这说明本记录中的机器人总体是前进而非自转。`/plan` 在 53.697–69.136 s 持续重规划，终点固定为 `(13.186, -4.595)`（`map`）；计划 pose 数从 56 降至 3，符合沿同一目标前进的表象。

### 2. TF 链连续，未见 map->odom 跳变

录制的 TF 边为：

```text
map -> odom                 5,392 samples, 1.247–180.948 s
odom -> base_link           1,788 samples, 1.276–180.877 s
base_link -> chassis_cad_link  (static)
chassis_cad_link -> lidar_mount_link (static)
lidar_mount_link -> lidar_link       (static)
lidar_link -> livox_frame            (static)
camera_init -> body         1,797 samples
```

`map -> odom` 首末样本相同：translation `(7.9573, -1.4717, -0.0181)` m，RPY `(0.561°, 0.158°, -12.313°)`；没有记录到足以解释“持续旋转”的 map->odom 漂移或切换。`/agt/odometry/local` 的 frame contract 也一致：`odom -> base_link`。

### 3. local costmap 存在 footprint 内高代价

Nav2 发布的 `OccupancyGrid` 值域是 `0–100`，不能把其 `100` 误当作原始 Nav2 `unsigned char` 的 `254`。按当前 Nav2 padded footprint（前/后 `0.55 m`，半宽 `0.43 m`）将每帧 local costmap 投影到最近时间的 `T_odom_base_link`：

| 证据 | 结果 |
|---|---:|
| local costmap 帧数 | 694 |
| 全图最大 OccupancyGrid 值 | 100 |
| footprint 内 value=100 的帧 | **15** |
| footprint 内 value=100 的最大 cell 数 | **5** |
| footprint 内 value≥100 的最大 cell 数 | 5 |
| footprint 内 value≥200 的帧 | 0（该消息类型最大为 100） |
| footprint 内最高值 | 0 / 68 / 79 / 99 / 100 |

value=100 的时间（相对 bag 起点）为 103.995、104.795、105.395、105.595、105.895、106.095、106.396、106.695、106.896、107.495、107.698、107.898、108.395、108.899 和 120.697 s。在这些采样点，最近 `/cmd_vel` 为零；因此该记录显示“停止后 footprint 内代价出现”，而非“旋转命令造成代价出现”。

全局 costmap 只有 static + inflation 来源；local costmap 为 voxel + inflation 来源。两者均有非零代价，但本 bag 中没有 `map` 以外的 global obstacle layer。

### 4. LiDAR 自障碍：可疑，但本 bag 尚不足以定责

原始 LiDAR 的 frame 是 `livox_frame`（首帧 19,968 点）。静态链表明传感器相对底盘不是水平零位：`base_link -> chassis_cad_link` z=`0.6238 m`，`chassis_cad_link -> lidar_mount_link` pitch=`+13.000°`、x=`0.1581 m`、z=`0.1037 m`。

当前 navigation obstacle 支路的配置是：

- `/livox/lidar` 经 bridge 生成 `/agt/livox/points`；
- preprocessor 在 `base_link` 中执行 self filter，输出 `/agt/navigation/points_obstacles`；
- VoxelLayer 对该输出 marking + clearing，最小 obstacle/raytrace range 均为 `0.30 m`，高度范围 `0.10–2.0 m`。

自过滤盒为中心 `(0,0,0.20)`、尺寸 `(1.023,0.778,0.400)` m、padding `0.05 m`，即约 x `±0.562 m`、y `±0.439 m`、z `[-0.05,0.45] m`。后方杆若位于此盒之外（特别是更高或更靠后），可以进入 VoxelLayer。

另有一个配置/实现不一致需要现场复核：YAML 中有 `rear_filter.enabled` 与 box 参数，但当前 `obstacle_cloud_node.cpp` 读取的是 `rear_sector.enabled`；当前实际 `rear_sector.enabled: false`。因此 YAML 的 `rear_filter` 块本身不会提供后方排除。此结论是源码审计，不是建议立即修改配置。

## 根因排序

| 排名 | 假设 | 结论 | 证据与边界 |
|---:|---|---|---|
| 1 | **local obstacle/self-obstacle 使 footprint 内出现高代价** | 高优先级、已观察到 costmap 现象 | 15 个 local costmap 帧在 footprint 内 value=100；但未录 obstacle cloud，不能确认是后杆而非真实近障碍。 |
| 2 | 后方安装杆进入 LiDAR costmap | 中等优先级、待证实 | 后方过滤未实际启用，self box 高度/范围有限，且传感器有安装 pitch；需直接看 obstacle cloud 定责。 |
| 3 | inflation/footprint 边界余量导致局部阻塞 | 中等优先级 | footprint 为 1.10 × 0.86 m，inflation radius `0.55 m`；value=99/100 已进入 footprint。不能仅凭当前 bag 判断是尺寸设置错误。 |
| 4 | RPP controller 造成持续旋转 | 低优先级（本包不支持） | RPP 允许 rotate-to-heading，阈值 0.60 rad，角速度 0.55 rad/s；但此包没有原地旋转 `/cmd_vel`，odom yaw 仅 +2.44°。 |
| 5 | TF 延迟 / map->odom 不稳定 | 低优先级（本包不支持） | `map->odom` 在完整记录中保持固定；`odom->base_link` 持续发布。未录 TF latency diagnostics，不能声明“零延迟”。 |

## 相关运行配置（只读）

当前 `nav2_params.yaml` 与 `nav2_acceptance_params.yaml` 的关键参数一致：

| 项目 | 值 |
|---|---|
| `robot_base_frame` | `base_link` |
| footprint | `[[0.52,0.40],[0.52,-0.40],[-0.52,-0.40],[-0.52,0.40]]` |
| footprint padding | `0.03 m` |
| local costmap | `odom`，rolling 8 × 8 m，0.05 m，VoxelLayer + InflationLayer |
| global costmap | `map`，StaticLayer + InflationLayer |
| inflation radius / scaling | `0.55 m` / `3.0` |
| controller | Regulated Pure Pursuit，`desired_linear_vel=0.45`，rotate-to-heading enabled |
| rotate threshold / angular velocity | `0.60 rad` / `0.55 rad/s` |

`rviz_field_demo.launch.py` 默认加载普通 `nav2_params.yaml`；`acceptance_field.launch.py` 才默认加载 `nav2_acceptance_params.yaml`。本 bag 未记录参数快照，故不能仅从 bag 证明现场使用的是其中哪一个；但两份文件上述关键值相同。

## 下一步现场实验建议（不先改参数）

1. **先停在安全区域，不下发导航目标**，同时录制 `/agt/navigation/points_obstacles`、`/local_costmap/costmap`、`/local_costmap/published_footprint`、`/cmd_vel`、`/cmd_vel_smoothed`（若存在）、cmd_vel_guard 输出与底盘反馈。RViz 将 obstacle cloud、local costmap、footprint 固定在 `base_link` 观察 30 s。
2. **复现最小实验**：只遮挡/拆除后方杆的 LiDAR 可见部分一次，保持地图、目标和全部参数不变；比较 footprint 内 `OccupancyGrid=100` 的帧数。该实验可验证“杆”而非凭视觉推测。
3. **确认预处理实际参数**：`ros2 param get /agt_obstacle_cloud_preprocessor self_filter.enabled`、`rear_sector.enabled`、`rear_sector.center_deg`、`rear_sector.width_deg`，并执行 `ros2 topic echo --once /agt/navigation/points_obstacles --field header`。不要将 YAML 的 `rear_filter.enabled` 当作运行时生效证据。
4. 若障碍云在停机时仍落在 padded footprint 内，保存一段包含原始 `/livox/lidar`、bridge 输出和 obstacle 输出的短 bag，再决定是否提出过滤范围或传感器遮挡的变更；本审计不建议直接改参数。
5. 若停机时 footprint 内没有高代价，再以同一目标复现运动，并完整录制 controller/guard/底盘命令链和 action feedback；只有届时出现线速度近零、持续非零角速度，才将 RPP rotate-to-heading 或恢复行为升级为主假设。

## 可直接执行的只读现场命令

```bash
# Costmap、规划、控制和 obstacle 支路频率/类型
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
ros2 topic hz /plan
ros2 topic hz /cmd_vel
ros2 topic hz /agt/navigation/points_obstacles

# 固定一帧确认 frame 与 footprint 周围的可视化（RViz 更适合看 Grid）
ros2 topic echo --once /local_costmap/costmap --field header
ros2 topic echo --once /agt/navigation/points_obstacles --field header

# TF 与 Nav2 生命周期；均为读取操作
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 param get /agt_obstacle_cloud_preprocessor self_filter.enabled
ros2 param get /agt_obstacle_cloud_preprocessor rear_sector.enabled
```

建议现场录制至少包含：

```bash
ros2 bag record -o nav_self_obstacle_check \
  /livox/lidar /agt/livox/points /agt/navigation/points_obstacles \
  /local_costmap/costmap /global_costmap/costmap /plan \
  /cmd_vel /cmd_vel_smoothed /tf /tf_static \
  /agt/odometry/local /navigate_to_pose/_action/feedback
```

若 `/cmd_vel_smoothed` 不存在，`ros2 topic list | rg 'cmd_vel|controller'` 后只录制实际存在的 guard/底盘下游 topic。
