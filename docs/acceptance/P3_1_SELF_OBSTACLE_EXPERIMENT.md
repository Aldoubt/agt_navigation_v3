# P3.1 Self-Obstacle Rear-Sector Validation Experiment

目标：以可复现、最小风险的方式验证 `rear_sector` 是否能移除机器人自身（尤其后方安装杆）反射，并消除 local costmap footprint 内的高代价。此实验不修改 Nav2、地图、定位、控制器或导航策略。

## 已确认的节点与数据链

实际 filtering node 是 `agt_obstacle_cloud_preprocessor`（executable: `obstacle_cloud_node`）。完整运行链：

```text
/livox/lidar (CustomMsg, livox_frame)
  -> agt_livox_tools /agt/livox/points (PointCloud2)
  -> agt_obstacle_cloud_preprocessor
  -> /agt/navigation/points_obstacles
  -> Nav2 local_costmap VoxelLayer (marking + clearing)
  -> local_costmap InflationLayer / controller collision checking
```

过滤判断以 `base_link <- cloud_frame` 的消息时间 TF 进行；输出 cloud 保留输入 header/frame，由 VoxelLayer 在其自身 TF 合同下处理。

## 合同修复与观测能力

`rear_filter` box key 已从正式 YAML 移除，因为节点从未读取它。唯一正式、可实验的后向过滤合同是：

```yaml
rear_sector:
  enabled: false              # baseline 默认值；实验时才设 true
  center_deg: 180.0
  width_deg: 10.0
  min_range_m: 0.5
  max_range_m: 4.0
```

新增的 observation 参数均默认关闭/空值，因此不改变既有 field runtime：

| 参数 | 默认 | 用途 |
|---|---|---|
| `rear_sector.enabled` | false | 实验性后向 sector 开关 |
| `statistics_output` | empty | 节点正常退出时写 cumulative YAML |
| `debug_log_interval_sec` | 0.0 | 周期性打印统计；0 表示关闭 |

统计输出包含：`input_points`、`output_points`、`removed_points`、`removed_ratio_of_input`、`rear_removed_points`、`rear_sector_removed_ratio_of_input`，以及 self/range/voxel 分项。这些是累计值，不应和单帧比例混淆。

## 试验设计

### 0. 前置安全与固定条件

- 机器人静止、周边空旷、安全员在场；不下发 navigation goal。
- 使用同一 map、同一 LiDAR 安装、同一 `base_link` TF、同一 Nav2 配置。
- 每组至少 30 秒；仅改变 `rear_sector.enabled`，不要同时改变 footprint、inflation、self box 或 Nav2 参数。

### A：Baseline（rear sector OFF）

启动 field demo 时保持默认 false，同时开启统计：

```bash
ros2 launch agt_system_bringup rviz_field_demo.launch.py \
  map:=<navigation/map.yaml> global_map:=<localization/global_map.pcd> \
  obstacle_rear_sector_enabled:=false \
  obstacle_statistics_output:=/tmp/p3_1_rear_off.yaml \
  obstacle_debug_log_interval_sec:=1.0
```

录制：

```bash
ros2 bag record -o p3_1_rear_off \
  /livox/lidar /agt/livox/points /agt/navigation/points_obstacles \
  /local_costmap/costmap /local_costmap/published_footprint \
  /tf /tf_static /agt/odometry/local
```

在 RViz 中以 `base_link` 为 fixed frame 观察 obstacle output、local costmap 与 footprint。停止节点后保存 `/tmp/p3_1_rear_off.yaml`。

### B：Rear-sector ON

在与 A 相同的位置、姿态、环境下，只改变：

```bash
obstacle_rear_sector_enabled:=true
```

输出路径改为 `/tmp/p3_1_rear_on.yaml`，bag 名称改为 `p3_1_rear_on`。默认 sector 为中心 180°、宽 10°、0.5–4.0 m；它只是首个证伪/确认实验窗口，未被视为最终现场参数。

## PASS / FAIL 判据

| 判据 | PASS | FAIL / 不可判定 |
|---|---|---|
| 合同 | 日志显示 `rear_sector_enabled=true`，且最终 YAML 存在 | 仍为 false、输出文件缺失或 TF drop |
| 过滤作用 | `rear_removed_points>0`，output/rear ratio 与 A 有可解释差异 | rear removed 为 0：杆不在 sector、没有杆点或配置未生效 |
| 自障碍 | B 中安装杆点从 `/agt/navigation/points_obstacles` 消失，footprint 内 local-costmap=100 现象消失/显著减少 | 仍出现：扩大排查 self box、真实近障碍、TF 或 costmap 其他来源；不要直接加大 mask |
| 安全保持 | 已知外部测试障碍仍显示于 obstacle output/local costmap | 外部障碍被过滤：本 sector 不可接受 |

## 结果记录模板

```yaml
experiment_id: P3.1
map_package: bunker_mid360_mapping_20260901_205036/v003-indexed
hardware_installation: <identifier/photo reference>
environment: <stationary location and nearby objects>
baseline_off:
  statistics_file: /tmp/p3_1_rear_off.yaml
  rear_removed_points: <value>
  rear_sector_removed_ratio_of_input: <value>
  footprint_cost_100_samples: <value>
sector_on:
  statistics_file: /tmp/p3_1_rear_on.yaml
  rear_removed_points: <value>
  rear_sector_removed_ratio_of_input: <value>
  footprint_cost_100_samples: <value>
  external_obstacle_visible: <true|false>
result: <PASS|FAIL|INCONCLUSIVE>
evidence: [<bag paths>, <RViz screenshots>]
```

## 解释边界

若 B 的 `rear_removed_points=0`，不能直接得出“没有 self obstacle”：杆可能不在 180°±5°、距离不在 0.5–4.0 m、或反射不被 LiDAR 看见。若 B 消除了 footprint 高代价，也仍应使用外部障碍测试确认没有引入后方盲区。只有在该验证完成后，才提出独立、小范围的现场配置变更。
