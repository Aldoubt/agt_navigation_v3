# agt_navigation_benchmark

ROS 2 Humble 的 Navigation Quality Benchmark 离线工具。它读取已有 rosbag2 sqlite3 或导出的地图资产，只做分析和报告生成，不启动导航节点、不修改 Nav2 参数、不修改现有导航代码。

## 用途

该工具用于：

1. Nav2 导航问题离线诊断；
2. 地图质量验收；
3. PCD → PGM 地图资产验证（只检查，不重建、不转换）；
4. Planner / Controller 性能比较的统一数据出口。

分析链路为：

```text
agt-lio-pgo-mapping
        |
  Map Asset Validator
        |
agt_navigation_benchmark
        |
  Map Quality -> Planner Quality -> Controller Quality
```

## 输入

bag 分析默认读取：

- `/map`、`/global_costmap/costmap`、`/local_costmap/costmap`；
- `/plan`；
- `/cmd_vel`；
- `/agt/odometry/local`；
- `/tf`、`/tf_static`（用于将 odometry 对齐到 path frame；缺失时报告会标记 frame mismatch）。

配置见 [`configs/default.yaml`](configs/default.yaml)。`/cmd_vel` 没有 header，因此使用 rosbag 记录时间；其他带 header 的消息优先使用消息 header 时间。

## Navigation Recorder Profile

现场测试前可以根据当前 ROS graph 生成推荐录包命令：

```bash
python3 scripts/generate_record_command.py \
  --output record_command.sh
cat record_command.sh
```

profile 位于 [`configs/recorder_profile.yaml`](configs/recorder_profile.yaml)，覆盖 planning、controller、localization、goal 和 runtime 五类数据。脚本执行 `ros2 node list`、`ros2 topic list -t`、`ros2 action list`、`ros2 service list`，并写出 `reports/navigation_stack_discovery.md`。profile 使用 namespace 无关的 glob（例如 `*/plan`、`*/cmd_vel`、`*/global_costmap/*`、`*/navigate_to_pose*/_action/goal`），只把当前实际存在的匹配 topic 写入 `record_command.sh`，并打印每一类的 `PASS` / `PARTIAL` / `MISSING` 状态；localization 支持 `/odom` 到 `/agt/odometry/local`、`/agt/chassis/odometry` 的平台别名。

同一命令还会调用 `ros2 lifecycle nodes` 和 `ros2 lifecycle get`，生成 `reports/runtime_diagnosis.md`，检查 `ROS_DOMAIN_ID`、`planner_server`、`controller_server`、`bt_navigator`、`map_server` 的可见性及 lifecycle 状态。关键节点状态使用 `FOUND`、`MISSING`、`INACTIVE`；Nav2 未出现在当前 graph 时会提示检查 launch、namespace 和 ROS_DOMAIN_ID。

对已有 bag，主分析会生成 `reports/dataset_completeness_report.md`。其中 core navigation 数据完整时置信度为 `HIGH`；goal/runtime 缺失但 planning/controller/localization 可用时为 `MEDIUM`；core 数据缺失时为 `LOW`。该检查只读 bag，不启动或修改导航系统。

## 运行 rosbag 分析

```bash
cd /home/yangxuan/ros2_ws/src/agt_navigation_benchmark
python3 scripts/analyze_nav_bag.py \
  --bag /home/yangxuan/ros2_ws/agt_data/field_acceptance/nav_test_002
```

或者构建后使用 ROS 2 标准入口：

```bash
cd /home/yangxuan/ros2_ws
colcon build --packages-select agt_navigation_benchmark --symlink-install
source install/setup.bash
ros2 run agt_navigation_benchmark analyze_nav_bag --bag /path/to/rosbag
```

## Map Quality Analyzer

`map_analyzer.py` 对每条 OccupancyGrid 统计 unknown/free/occupied 比例，并用 8-connected component 计算 unknown 连通区域数量和最大 unknown 区域面积。质量等级为：

- unknown `< 10%`：`GOOD`；
- `10% <= unknown <= 30%`：`WARNING`；
- unknown `> 30%`：`BAD`。

风险等级为 `LOW` / `MEDIUM` / `HIGH`，其中 unknown 超过 50% 为 HIGH。输出 `reports/map_quality_metrics.csv`，报告包含 Global Map 的 unknown ratio、largest unknown region 和风险等级。

## Planner Quality Analyzer

Planner 质量输出包括：

- 曲率 mean/std/P50/P90/P95/P99；
- heading change mean/std；
- sharp turn ratio；
- 0 到 1 的 Path Smoothness Score，越接近 1 越平滑；
- `SMOOTH` / `UNSTABLE` 评估。

结果写入 `reports/planner_quality_metrics.csv`。旧的 `path_metrics.csv` 继续保留，便于兼容之前的分析结果。

## Offline Planner Replay A/B

`scripts/run_nav2_planner_ab_test.py` 是真实 Nav2 后端，会自动创建：

```text
experiments/nav_test_002/planner_replay/
├── experiment_pose.yaml
├── baseline/
├── unknown_free/
└── comparison/
```

它从 bag 第一条有效 `/plan` 提取 start/goal，分别隔离启动 `map_server`、`planner_server` 和 `lifecycle_manager`，通过 `nav2_msgs/action/ComputePathToPose` 获取真实 `nav_msgs/Path`。每个分支保存 `map.yaml`、`planner.yaml`、`pose.yaml`、`result.md`、`nav2_replay.log` 和对应的 plan CSV。

```bash
python3 scripts/run_nav2_planner_ab_test.py \
  --bag nav_test_002
```

也可以显式指定 planner：

```bash
python3 scripts/run_nav2_planner_ab_test.py \
  --bag /home/yangxuan/ros2_ws/agt_data/field_acceptance/nav_test_002 \
  --planner-plugin nav2_smac_planner/SmacPlanner2D \
  --planner-id GridBased
```

底层实现见 [`nav2_replay_backend.py`](agt_nav_benchmark/nav2_replay_backend.py)。每次 replay 使用唯一 namespace，完成后自动停止全部临时进程，不接入现有导航 namespace。

对比输出为 `reports/baseline_plan.csv`、`reports/unknown_free_plan.csv`、`reports/planner_ab_comparison.csv` 和 `reports/planner_ab_report.md`。当 unknown ratio 下降、`curvature_p95` 下降至少 30% 且 smoothness 提升时，报告输出 `Evidence: Unknown space caused planner degradation.`；否则提示检查 inflation、planner 参数和 obstacle representation。

## Unknown-Free A/B 实验

该脚本从 rosbag 的 `/map` 生成原始地图和 unknown 全转 free 的对照地图，不会运行 planner：

```bash
python3 scripts/map_unknown_ab_test.py \
  --bag /home/yangxuan/ros2_ws/agt_data/field_acceptance/nav_test_002
```

输出到 `reports/map_ab_test/`：

- `original_map.pgm` / `original_map.yaml`；
- `unknown_free_map.pgm` / `unknown_free_map.yaml`；
- `comparison.md`，对比 unknown/free/occupied 比例。

## PCD → PGM Asset Validator

Validator 支持检查 ASCII、binary 和 PCL `binary_compressed` PCD，以及 PGM/YAML：

```bash
python3 scripts/validate_map_asset.py \
  --pcd /path/to/global.pcd \
  --pgm /path/to/map.pgm \
  --yaml /path/to/map.yaml \
  --output reports/map_asset_report.md
```

PCD 输出 point count、bounding box、z-range、XY density estimation；PGM/YAML 输出 resolution、size、unknown ratio、occupied ratio 和引用文件完整性。该工具只读，不实现点云重建。

bag 分析本身也会生成 `reports/map_asset_report.md`，用于记录 `/map` OccupancyGrid 快照的资产质量；如果有 PCD/PGM/YAML 路径，再使用上面的 validator 获取外部资产详情。

## 输出

默认输出目录是 `reports/`：

- `nav_test_002_report.md`：自动诊断报告；
- `map_quality_metrics.csv`：地图质量和 unknown 连通域；
- `planner_quality_metrics.csv`：Planner 曲率、heading、smoothness；
- `planner_ab_comparison.csv`、`planner_ab_report.md`：baseline / unknown-free Planner A/B 结果；
- `controller_metrics.csv`：Controller 汇总；
- `path_metrics.csv`、`costmap_metrics.csv`、`cmd_vel_metrics.csv`：兼容旧版本的指标；
- `localization_metrics.csv`、`planner_controller_correlation.csv`：定位层和跨层时间关联；
- `odom_frequency.csv`：候选 odometry topic 的存在性、频率、异常间隔和 publisher provenance 状态；
- `odom_runtime_report.md`：odometry 频率、TF 和 plan 时间关联审计；
- `tf_conflict_report.md`：目标 TF edge 与可观察 TF topic 冲突证据；
- `controller_tracking.csv`：每个时间匹配的 odometry 到当前 `/plan` 最近 waypoint 的 cross-track error；
- `cmd_vel_dynamics.csv`：线速度/角速度及其按 bag 时间戳计算的线加速度/角加速度；
- `map_asset_report.md`：地图资产检查摘要；
- `controller_tracking_report.md`（兼容别名 `controller_report.md`）：Planner/Tracking/Controller 三层关联诊断；
- `dataset_completeness_report.md`：按 recorder profile 检查 bag 数据完整性和诊断置信度；
- `navigation_stack_discovery.md`：当前 ROS node/topic/type/action/service graph 快照；
- `runtime_diagnosis.md`：ROS domain、Nav2 lifecycle 和关键节点运行状态诊断；
- `path_curvature.png`、`unknown_ratio.png`、`cmd_vel_angular.png`、`tracking_error.png`：可直接查看的图像。

## Planner Runtime Audit

`planner_runtime_analysis.py` 审计现场记录的全部 `/plan` 序列，统计连续规划结果的路径偏差、replan 频率，并将相邻 plan 前后的 global/local costmap 占用类别变化压缩为 `costmap_trigger_score`。`goal_distance` 定义为该条 Path 首尾 waypoint 的欧氏距离；`plan_change_distance` 是两条路径按归一化弧长重采样后的平均偏差。

完整 bag 分析会生成：

- `planner_runtime_metrics.csv`；
- `planner_runtime_report.md`；
- `planner_runtime_config.yaml`：从 `/parameter_events` 中筛选 `planner_server` 参数；没有该 topic 或没有 planner_server 事件时内容为 `MISSING`；
- `plan_sequence.png`、`plan_curvature_timeline.png`。

runtime audit 的启发式结论为：连续 plan 几乎一致表示 planner 稳定；plan 高频变化但 costmap 变化小，优先检查 planner 参数/goal/runtime 状态；plan 变化与 costmap 变化同步，则提示 costmap 影响。该结论用于离线证据筛查，不等价于因果证明。

## Nav2 Goal / Action Runtime Audit

`nav2_goal_analysis.py` 检查 `/goal_pose`、`/goal`、`/behavior_tree_log` 以及常见的 `NavigateToPose` action topic：

- `/navigate_to_pose/_action/goal`；
- `/navigate_to_pose/_action/feedback`；
- `/navigate_to_pose/_action/result`；
- `/navigate_to_pose/_action/status`。

输出 `goal_timeline.csv`、`nav2_action_metrics.csv` 和 `nav2_runtime_report.md`。报告关联 goal 时间、连续 plan 变化和 costmap trigger，给出 Goal-triggered plan、unchanged-goal plan、costmap-synchronized plan 统计及 Root Cause Ranking。topic 未录制时 CSV 仍保留固定表头，报告明确标记 `MISSING`；因此“没有 goal 证据”和“goal 未变化”不会混淆。

## Controller Tracking Analyzer

该层将 `/plan` 与 `/agt/odometry/local` 按时间关联：每条 odometry 选择当前有效规划路径，再匹配路径上的最近 waypoint，计算 cross-track error，并输出 mean、maximum、P95。默认最后一条路径只保持 2 秒，避免把旧路径错误地应用到整个 bag。分析会校验 `frame_id`；如果 path 与 odometry 不同坐标系，会保留 raw 距离但在报告中标记 `FRAME_MISMATCH`，避免将未经过 `/tf` 变换的距离误诊为控制器误差。

`/cmd_vel` 使用 rosbag 记录时间戳计算相邻样本的 linear/angular acceleration。完整分析会自动生成 tracking 输出；也可以直接运行：

```bash
python3 scripts/analyze_controller_tracking.py \
  --bag /home/yangxuan/ros2_ws/agt_data/field_acceptance/nav_test_002
```

诊断规则为：

- path smooth + tracking error high：Case A，优先检查 controller/base/odometry；
- path bad + tracking good：Case B，优先检查 planner；
- path bad + tracking bad：Case C，planner 与 controller 均需检查。

阈值位于 [`configs/default.yaml`](configs/default.yaml) 的 `tracking` 节中。

`oscillation_score` 定义为：忽略 `|angular.z| <= angular_deadband` 后，连续有效符号切换次数除以有效样本间隔数，范围为 0 到 1。所有诊断阈值都在 YAML 中，可按平台控制频率和场景调整。

## Odom Runtime Audit

`odom_runtime_analysis.py` 对 rosbag 中的 `/odom`、`/agt/chassis/odometry`、`/agt/odometry/local`、`/tf` 和 `/tf_static` 做离线审计：

- 检查候选 odometry topic 是否存在，并输出消息频率、50 Hz / 100 Hz 频段判断和异常时间间隔；
- 输出 `odom_frequency.csv`，其中包含 `publisher_count` 和 `publisher_nodes` 字段；
- 对已存在 topic，publisher provenance 标记为 `UNKNOWN`，因为标准 rosbag2 topic metadata 不记录 publisher node；缺失 topic 标记为 `MISSING`；
- 对 `odom -> base_link`、`base_link -> base_footprint` 做 TF 证据检查，生成 `tf_conflict_report.md`；TFMessage 不含 authority，因此不会把重复 TF 消息猜测为多个节点；
- 将 plan change timestamp 与 odometry 异常间隔做时间关联，生成 `odom_runtime_report.md`。

主分析命令会自动生成这些文件：

```bash
python3 scripts/analyze_nav_bag.py \
  --bag /home/yangxuan/ros2_ws/agt_data/field_acceptance/nav_test_002
```

频率异常或接近 100 Hz 只能作为运行时证据，不能单独证明存在两个 publisher。要确认现场节点来源，需要在 live ROS graph 中使用 `ros2 topic info -v`；本模块不会修改底盘代码或 Nav2 配置。

## 自动诊断规则

- unknown ratio `> 50%` + path curvature high + controller oscillation low：`Map representation is the primary suspect`；
- path smooth + cmd_vel oscillation high：提示 Controller tracking issue，检查 RPP/MPPI；
- unknown low + path smooth + cmd_vel unstable：提示 Base controller or odometry issue；
- path unstable + occupied ratio high：提示 costmap inflation 或地图质量问题。

这些结论是离线证据关联，不是根因的数学证明，应结合 CSV、图像、实际地图和参数复核。

## 实验记录

当前样例位于 [`experiments/nav_test_002`](experiments/nav_test_002/)：

- `metadata.yaml`：bag、robot、map、planner、controller 元数据；
- `result.md`：该次实验的摘要和报告链接。

## 后续扩展

- Localization Benchmark；
- Planner Benchmark；
- Controller Benchmark；
- Perception Benchmark；
- RPP / MPPI 对比和更完整的四层导航评估体系。

Planner 模板位于 `planner_benchmark/configs/`：`navfn.yaml`、`smac.yaml`、`theta_star.yaml`，用于未来接入 NavFn、Smac Hybrid 和 Theta* 参数扫描。
