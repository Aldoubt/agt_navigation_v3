# P3 Map / Costmap Field Navigation Audit

本报告整合 [NAV_TEST_002_AUDIT.md](NAV_TEST_002_AUDIT.md)、[MAP_STATISTICS_REPORT.md](MAP_STATISTICS_REPORT.md) 和 [SELF_OBSTACLE_CHAIN_AUDIT.md](SELF_OBSTACLE_CHAIN_AUDIT.md)。范围是 v003 field package 和 `nav_test_002` 的只读诊断；不构成参数、地图或算法变更。

## 审计结论

1. 当前 active map 是 `bunker_mid360_mapping_20260901_205036/v003-indexed`；PGM 源自可追溯的 v002-edited `global_map.pcd`，但 P0.7 base_link footprint audit 仍为 **MAP=REVIEW**（5 occupied cells / 3 regions）。
2. PGM/`map.yaml` trinary 语义正确：`0` occupied、`205` unknown、`254` free；`free_thresh=0.196` 特意低于 `205` 对应的 `50/255≈0.196078`，不会将 unknown 错判为 free。
3. `nav_test_002` 不存在持续原地旋转命令，也没有 map->odom 跳变证据；因此“控制器持续自旋”和“TF 不稳定”不应作为当前主因。
4. local costmap footprint 内出现 value=100 是当前最强运行时异常。LiDAR self-obstacle 是高优先级假设，但还未被该 bag 的点云输出证实。
5. `rear_filter` YAML 与运行实现脱节；只有 disabled 的 `rear_sector` 会被代码读取。该问题能解释“期望存在后向过滤、但实际没有”的现象。

## PGM 与 Nav2 语义

`map.yaml`：

```yaml
mode: trinary
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

对于 `negate=0`，Nav2 occupancy probability 是 `(255 - gray) / 255`：

| PGM gray | 概率 | Nav2 classification |
|---:|---:|---|
| 0 | 1.0 | occupied |
| 205 | 0.196078 | unknown（介于两个阈值） |
| 254 | 0.003922 | free |

PGM 的三种实际值恰为这三类；未发现“unknown 被 YAML 解释为 occupied/free”或“free 被解释为 unknown”的阈值错误。

## Costmap 来源

| Costmap | frame | layers | LiDAR 输入 |
|---|---|---|---|
| local | `odom` | VoxelLayer + InflationLayer | `/agt/navigation/points_obstacles`，marking/clearing true |
| global | `map` | StaticLayer + InflationLayer | 无 live obstacle source |

robot reference 均为 `base_link`。footprint polygon 为 `[±0.52, ±0.40] m`，padding `0.03 m`，即审计使用的 padded 近似 front/rear `0.55 m`、half-width `0.43 m`。inflation radius 为 `0.55 m`。

## 最小修复方案（提案，未实施）

### P0：必须先消除证据盲区

| 项目 | 位置 | 风险 | 验证方法 |
|---|---|---|---|
| 在下一次现场短 bag 纳入 obstacle output、footprint、guard/底盘命令链 | 仅 rosbag 录制命令 | 低；不影响运行参数 | 静止 30 s，确认每个 footprint value=100 cell 能回溯至 obstacle cloud 或排除它 |
| 核对并修复 `rear_filter` 与 `rear_sector` 的配置/实现 contract（先提出小 patch，不直接启用） | `agt_pointcloud_preprocessor` config + node parameter contract | 中；错误的后向 mask 可能移除真实障碍 | 单元 test：参数生效、base_link box/sector 边界确定；现场前后点云对比 |

### P1：现场改善（仅在 P0 定责后）

| 项目 | 位置 | 风险 | 验证方法 |
|---|---|---|---|
| 若确认安装杆反射，提出最小、base_link 定义的 self mask 或明确 rear exclusion patch | `obstacle_cloud.yaml` 及相应已验证参数 | 中；mask 过大可能漏检后方真实障碍 | 静止与缓慢倒车场景：杆点消失，外部测试障碍仍被标记 |
| 用 RViz 复核 P0.7 的 3 个 map review regions，并仅在人工确认是 artifact 时提出带 patch history 的新 map package | map patch workflow，不改 v003 | 中；错误清图会产生静态障碍漏检 | 同 footprint audit、人工 review、hash/provenance 审查 |
| 若 self-obstacle 被排除仍有局部卡顿，记录 footprint 与 inflation overlay 后再评估 footprint/inflation margin | Nav2 参数（后续独立变更） | 中高；改变通行安全边界 | 同一目标的 A/B、最小间隙和急停观察；不以单次成功作为验收 |

### P2：长期优化

| 项目 | 位置 | 风险 | 验证方法 |
|---|---|---|---|
| 让 preprocessor 定期发布可审计的 filter statistics（input/self/rear/output point counts） | pointcloud preprocessor observability | 低 | 与 rosbag 点云和 costmap cell 关联，确认数值可复现 |
| 录制完整 controller→smoother→guard→CAN feedback 观测链 | bringup/acceptance recording profile | 低 | 可精确区分 Nav2 意图、guard 限制和底盘实际执行 |
| 为路线的 unknown coverage / clearance 生成 preflight 指标 | acceptance map/mission evidence | 低 | 每个目标在下发前给出 free corridor、unknown 和 footprint overlap 摘要 |

## 下一步顺序

先执行 P0 的“可观测性短录制”，而不是调 Nav2、改地图或关闭障碍过滤。若 `points_obstacles` 明确包含后方杆，再进入 P1 的最小 filter patch；若没有，则把现场高代价 cell 与地图 review regions、真实近障碍逐一对齐。只有在这两项排除后，才重新讨论 controller 行为。
