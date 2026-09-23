# Benchmark 迁移审计（R8）

原 `src/agt_navigation_benchmark` 没有独立 Git 元数据，已整体迁入
`agt_navigation_v3/tools/benchmark/agt_navigation_benchmark`。ROS package 名与 CLI 保持不变，
历史 `reports/` 和 `experiments/` 原样保留，避免丢失现场对照证据。

| 组件 | 判断 | 理由 |
|---|---|---|
| `agt_nav_benchmark/bag_reader.py`、`nav2_goal_analysis.py`、`planner_replay.py` | KEEP | 离线 rosbag / Goal / Planner 回放，与生产 runtime 无依赖 |
| `controller_tracking.py`、`costmap_analysis.py`、`odom_runtime_analysis.py`、`localization_analysis.py` | KEEP | 已有历史报告使用；V3 的 `controller_metrics.py` 只覆盖控制器基线，暂不合并算法 |
| `reports/`、`experiments/` | ARCHIVE_DATA | 历史实验，保留原始路径结构，不混入运行时地图 |
| V3 `tools/navigation_benchmark` | KEEP | 控制器配置与接受工具；未发现生产 package 依赖外部 benchmark |

迁移前后：benchmark Python 测试均为 `7 passed`；`colcon list` 保持 80 个包。
当前没有足以证明两套控制器指标完全同义的同 bag parity 结果，因此没有删除其中任一
指标实现，也没有把报告改写成新的指标定义。后续若合并指标，先用同一 bag 比较输出。
