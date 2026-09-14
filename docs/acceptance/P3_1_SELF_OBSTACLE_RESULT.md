# P3.1 Self-Obstacle OFF/ON Result

Status: **NOT_EXECUTED**

在本轮只读 preflight 中，`/home/yangxuan/ros2_ws/agt_data` 下未发现用户指定的 `P3_1/off` 与 `P3_1/on` rosbag 目录，因此不能编造 A/B 指标或判定 rear sector 有效。

分析器已准备好：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/agt_navigation_v3/tools/p3_1_self_obstacle_replay_analyzer.py \
  --off <P3_1/off> --on <P3_1/on> \
  --off-statistics <optional-rear-off.yaml> \
  --on-statistics <optional-rear-on.yaml> \
  --metrics ~/ros2_ws/src/agt_navigation_v3/docs/acceptance/p3_1_metrics.yaml \
  --report ~/ros2_ws/src/agt_navigation_v3/docs/acceptance/P3_1_SELF_OBSTACLE_RESULT.md
```

它只读取 bag，并比较：

- `/agt/livox/points` 与 `/agt/navigation/points_obstacles` 的输入/输出点数；
- `statistics_output` YAML 中可归因于 rear sector 的删除比例；
- `/local_costmap/costmap_raw`（无该 topic 时回退到 `/local_costmap/costmap`）在 padded footprint 内的 value≥100 / ≥253 证据；
- `/cmd_vel` 的时间加权停止比例与原地旋转比例。

rear-sector 删除比例不能从 bag 中的 output 点数单独推断：input→output 差还包含 self box、range 和 voxel filtering。请同时提供两组 `statistics_output` YAML，或让 analyzer 在报告中保持该指标为 `NOT_AVAILABLE`。
