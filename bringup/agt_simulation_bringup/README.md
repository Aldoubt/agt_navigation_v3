# AGT P3 Gazebo 软件验收平台

`agt_simulation_bringup` 是 P3 field acceptance 前的软件闭环验收入口。它复用
现有 `agt_gazebo_sim` 履带底盘模型、`agt_sim_odometry_adapter` 和
`agt_nav2_bringup`；不仿真 SLAM，也不伪造全局定位。

## 验证边界

该平台验证：

- Gazebo 履带模型、`robot_state_publisher` 与传感器/静态 TF；
- `agt_sim_odometry_adapter` 作为仿真唯一的 `odom -> base_link` 发布者；
- 外部 `/agt/relocalization/pose` 到 `LocalizationManager` 的交接，以及唯一的
  `map -> odom` ownership；
- Nav2 的地图加载、lifecycle、planner、controller、behavior server 与 `/cmd_vel`。

它不验证 SLAM、全局重定位算法、真实底盘动力学、CAN、相机或现场安全流程。

## Launch

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 仅启动 Gazebo、履带模型、robot_state_publisher 和仿真局部里程计。
ros2 launch agt_simulation_bringup gazebo_field_acceptance.launch.py gui:=true

# 仅启动 Nav2 地图、planner/controller/behavior server 和 lifecycle。
ros2 launch agt_simulation_bringup nav2_sim_acceptance.launch.py \
  map:=/absolute/path/to/navigation/map.yaml

# 组合 Gazebo + LocalizationManager + Nav2。
ros2 launch agt_simulation_bringup full_stack_sim.launch.py \
  map:=/absolute/path/to/navigation/map.yaml gui:=true
```

启动 `full_stack_sim` 后，初始仅应有 `odom -> base_link`。平台不会创建
`map -> odom`。向 `/agt/relocalization/pose` 提供具有 `map` frame、有效协方差和
与本地 odom 对齐时间戳的外部全局 pose 后，`agt_localization_manager` 才会建立
`map -> odom`。

## P3 检查

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo /agt/odometry/local
ros2 topic echo /agt/localization/status
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
```

在 global pose 被接受后再检查：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

完整的 P3 gate、外部 localization 输入要求和现场边界见
`docs/acceptance/P3_RUNTIME_ACCEPTANCE_AUDIT.md`。

## 当前依赖与限制

- 需要 Gazebo Classic、`gazebo_ros`、`ros2_livox_simulation`、
  `agt_gazebo_sim`、`agt_sim_odometry_adapter`、`agt_nav2_bringup` 与
  `agt_localization_manager` 已在 overlay 中构建。
- 模拟模型来自 `agt_gazebo_sim/urdf/bunker_mid360_sim.urdf.xacro`。它包含 Gazebo
  运动和传感器插件；物理 `tracked_chassis_description` 是现场 URDF/calibration
  authority，但不单独提供 Gazebo 运动插件。
- 没有外部重定位 pose 时，这是有意保持的 `WAIT_GLOBAL` 状态，不是 simulator
  failure。
