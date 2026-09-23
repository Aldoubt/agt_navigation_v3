# Pointcloud Pipeline 收敛审计（R8）

仓库已从 `src/agt_pointcloud_pipeline` 移至 `src/shared/agt_pointcloud_pipeline`，
其 `.git` 目录与历史保留。三个 ROS package 继续可被 colcon 发现；没有把实验滤波器
加入生产导航链。导航生产链仍由 `agt_pointcloud_preprocessor` 唯一发布
`/agt/navigation/points_obstacles`；建图链仍使用自身的 LiDAR 前端输入。

| 包 | 结论 | 依据 |
|---|---|---|
| `agt_pointcloud_core` | KEEP_SHARED | ROS 独立的几何滤波算法，只有同仓 pipeline 依赖 |
| `agt_pointcloud_pipeline` | ARCHIVE_DEBUG | 插件链与 navigation/mapping/analysis profile 仅由同仓 debug launch 使用；生产 launch 未引用 |
| `agt_pointcloud_tools` | KEEP_DIAGNOSTIC | 离线 bag、静态探测和调参工具，不发布生产障碍点云 |
| V3 `agt_pointcloud_preprocessor` | KEEP_NAVIGATION | 当前唯一 Nav2 障碍云来源，保留后杆掩蔽开关及统计 |

`rg` 扫描 `package.xml`/launch/CMake：共享仓以外没有对上述三个包的直接生产依赖。
迁移前后 Python 工具测试均为 `10 passed`；core GTest 3 项通过；迁移后四个包
`colcon build` 通过。尚未对共享插件链运行同 bag 回放或云图 A/B，故不删除代码，
不宣称滤波数值与生产导航等价。`agt_pointcloud_pipeline/AGENTS.md` 的 ROS 独立 core、
帧和 TF 约束继续适用。
