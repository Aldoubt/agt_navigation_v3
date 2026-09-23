# MID360 + Bunker 实机验收流程

> 此文档保留 V3 验收基线，其中的旧地图路径和分终端启动示例尚未迁移到 V4，不能直接用于 V4 现场启动。V4 应先按 [导航启动文档](../导航启动文档.md) 通过 `run_field_stack.sh --map auto --robot bunker_v1` 解析已验证地图，并以 [V4 验收矩阵](acceptance/V4_ACCEPTANCE_MATRIX.md) 判定是否可进入实机测试。

本流程只采集证据，不改变 v3.1 Runtime Refactor 的四入口、节点所有权、TF
authority、导航参数或安全限速。验收工具不会发布速度、目标点或 TF。

## 1. 场地与人员

- 使用平整、干燥、至少直径 3 m 的封闭区域，移除人员和松散物品。
- 一名操作员负责导航，一名安全员全程手持急停；先验证急停能切断底盘运动。
- 检查履带张紧、MID360 固定、线束余量、电池电量、`can0` 和以太网链路。
- 记录机器人编号、MID360 序列号、地图版本、Git commit 和操作人员。
- 不得为通过验收而绕过 `cmd_vel_guard`，也不得直接向底盘注入速度。

## 2. 构建与启动

首次运行或源码更新后：

```bash
cd /home/yangxuan/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

所有运行终端都先 source 同一工作空间。按顺序启动，保持三个终端运行：

```bash
# Terminal 1: hardware
ros2 launch agt_system_bringup hardware.launch.py bunker_can_port:=can0

# Terminal 2: localization
MAP_ROOT=/home/yangxuan/ros2_ws/maps/bunker_mid360_mapping_20260901_205036/v003-indexed
ros2 launch agt_system_bringup localization.launch.py \
  global_map:=$MAP_ROOT/localization/global_map.pcd \
  relocalization_assets:=$MAP_ROOT/localization/relocalization

# Terminal 3: navigation
MAP_ROOT=/home/yangxuan/ros2_ws/maps/bunker_mid360_mapping_20260901_205036/v003-indexed
ros2 launch agt_system_bringup navigation.launch.py \
  map:=$MAP_ROOT/navigation/map.yaml \
  map_id:=bunker_mid360_v003
```

等待全局重定位成功、Nav2 lifecycle 全部 active，再进入验收。`debug.launch.py`
仅在需要 RViz 观察时启动。

## 3. 自动验收

第四个已 source 的终端运行：

```bash
cd /home/yangxuan/ros2_ws/src/agt_navigation_v3
ACCEPTANCE_ROBOT_ID=bunker-01 \
ACCEPTANCE_MID360_SERIAL=MID360_SERIAL_HERE \
ACCEPTANCE_MAP_ID=bunker_mid360_v003 \
ACCEPTANCE_OPERATOR=OPERATOR_NAME \
ACCEPTANCE_DURATION_SEC=180 \
scripts/run_hardware_acceptance.sh
```

脚本依次完成：

1. 核对 `/agt_localization_manager`、`/robot_state_publisher`、
   `/agt_pointcloud_preprocessor` 各一份。
2. 通过 DDS publisher GID 将每条 `/tf`、`/tf_static` 边追溯到 ROS 节点，拒绝
   同一 child frame 的多 parent 或多 publisher。
3. 并行采样 MID360、Bunker、定位、障碍点云和速度链路的频率。
4. 打开 30 秒运动中心观察窗口。操作员通过已批准的 Nav2 控制链给出低速原地转向，
   建议累计转角不低于 90°；工具自身不发速度。
5. 打开默认 180 秒录包窗口。操作员发送预先批准的 Nav2 路线，覆盖起步、直行、
   左右转、原地调头、减速停车和到点。
6. 在证据目录生成 `acceptance_report.md`。

脚本在任一运动前检查失败时停止，不会继续运动中心或导航测试。无人值守编排可设置
`ACCEPTANCE_NON_INTERACTIVE=1`，但仍必须有现场安全员和外部批准的控制任务。

## 4. 分项运行与复测

```bash
EVIDENCE=$PWD/hardware_acceptance_manual
mkdir -p "$EVIDENCE"

scripts/check_runtime.sh | tee "$EVIDENCE/runtime_check.txt"
scripts/check_tf_publishers.sh "$EVIDENCE/tf_authority.json"
scripts/check_topic_rates.sh "$EVIDENCE/topic_rates.json"

# 在窗口内用正常 Nav2 控制链执行低速原地转向
scripts/check_base_footprint_center.sh "$EVIDENCE/base_footprint_center.json"

# 在录制窗口内执行批准的短程导航路线
ACCEPTANCE_DURATION_SEC=180 scripts/record_navigation_run.sh "$EVIDENCE"
scripts/generate_acceptance_report.sh "$EVIDENCE"
```

临时跳过原始 MID360 点云和 IMU 可设置 `ACCEPTANCE_RECORD_RAW=0`，但这不满足完整
硬件验收的原始传感器留档要求。没有 GNSS 的场景可设置
`ACCEPTANCE_RECORD_GNSS=0`；`/ins/navsatfix` 本身是频率检查中的可选项。

## 5. 验收判据

- TF：profile 中每条边必须出现，parent、topic、publisher 必须匹配，所有已观察 child
  frame 不得存在多个 parent 或多个 publisher GID。
- 频率：必选 topic 均需达到
  `tests/acceptance/hardware_acceptance_profile.yaml` 的上下限并满足最短采样时长。
- 运动中心：`base_footprint → base_link` 静态偏置符合标定；原地转向的平面最大漂移
  不超过 0.15 m，有效转动半径不超过 0.20 m。
- 数据：`navigation_bag/metadata.yaml` 存在且消息数大于零。
- 报告：五项全部为 `PASS` 才能接受；缺项为 `NOT_RUN`，任何失败为 `FAIL`。

阈值是验收基线，不是运行架构。现场确需调整时，应复制 profile、记录理由并通过
`HARDWARE_ACCEPTANCE_PROFILE=/absolute/path/profile.yaml` 显式选用，禁止隐式修改运行参数。

## 6. MID360 + Bunker 路线建议

1. 静止 20 秒：观察定位状态和静态点云，无底盘运动。
2. 原地左转约 90°，停车 5 秒；原地右转回到初始朝向，停车 5 秒。
3. 低速直行 2–3 m，停车；执行一次左弯和一次右弯。
4. 导航到距起点 3–5 m 的无遮挡目标，再返回起点附近。
5. 在路线中保留至少一次障碍物减速或绕行，但禁止人员充当动态障碍物。
6. 结束后确认 `/mux/cmd_vel` 回零、底盘静止，再关闭 launch。

若 MID360 低于 8 Hz，先检查供电、网卡丢包与驱动配置；若 `/wheel/odom` 或
`/bunker_status` 低于 20 Hz，先检查 CAN 错误计数和底盘通信。TF authority 冲突属于
架构运行异常，不得通过放宽频率或运动阈值规避。

## 7. 交付物

每次运行使用独立目录，完整保存：

- `acceptance_report.md`
- `runtime_check.txt`、`tf_authority.json`、`topic_rates.json`
- `base_footprint_center.json`
- `navigation_bag/`、`bag_info.txt`、`rosbag_record.log`
- `nodes.txt`、`topics.txt`、TF topic endpoint 信息和 `run_context.txt`

不得拼接不同机器人、不同地图或不同运行时段的证据来形成一次 PASS 报告。
