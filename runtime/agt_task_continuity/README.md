# Greenhouse task continuity

研究入口为 `/navigation/execute_task_segment`（`agt_navigation_interfaces/action/ExecuteTaskSegment`）。
一次任务固定 `task_id / request_epoch / robot_profile / map_id / map_version / map_hash`，
从可信的全局定位和当前 odom epoch 建立局部授权区段。只完成局部路径不表示地图任务完成。
固定终点是冻结路线区段的实际终点；`max_local_distance_m` 从首次 LOCAL_TASK 起累计，
因此任务前半段的 GLOBAL 行驶不会提前用尽局部行驶预算。

## 运行入口

完整组合入口：

```bash
ros2 launch agt_system_bringup greenhouse_research.launch.py
```

默认只运行感知、里程计质量和模式观测，`enable_motion=false`。已有 LIO、原始雷达驱动和
robot_state_publisher 应各自只有一个实例；研究组合不能与 V1 导航和另一套定位管理器同时启动。
移动实验使用该 launch 的显式 manifest、参数、车辆状态映射和导航地图参数。

单独运行模式节点：

```bash
ros2 launch agt_task_continuity task_continuity.launch.py
```

参数文件为 `config/task_continuity.yaml`。默认 `research_enabled=false`、`field_verified=false`，
质量阈值、制动减速度、总停车检测延迟和净空余量未填，节点不授权运动。

## 模式和权限

| 模式 | 所用路径 | 条件 |
|---|---|---|
| GLOBAL | map | 当前冻结地图的全局质量、实测底盘权限、局部里程计和前进扫掠净空有效 |
| DEGRADED | 暂不授权 | 旧控制权限立即撤销，等待稳定的行道证据 |
| LOCAL_TASK | odom | 同一 odom epoch、固定终点、剩余距离和时间、QR/QO 与制动净空有效 |
| RECOVERING | odom | 异步全局恢复期间继续使用受限局部路径 |
| SAFE | 暂不授权 | 硬证据失效、epoch 变化、时钟异常、预算用尽等；新的任务请求才可解锁 |

每次模式/计划变化都更新 `control_epoch`。任务执行器先撤销权限，确认旧 FollowPath 的终态，
再发布新的 `plan_stamp` 和 TaskContext，等待模式节点授权后提交新路径。
若无法确认 Nav2 终态，能力层保留控制租约，返回 `CANCEL_UNCONFIRMED`，直到旧句柄终态。
恢复完成同时匹配 `RecoveryStatus` 与正式 `GlobalQuality` 的地图、odom epoch、job_id 和验证时间。
局部模式会按 backoff 继续请求候选；同一地图/epoch 的新鲜 `RecoveryStatus.active=true` 时暂停额外请求，
让已接受的恢复目标完成平滑与独立观测验证。

## 话题连接

| 话题 | 类型 / 用途 |
|---|---|
| `/agt/research/task_context` | TaskContext，唯一任务执行器心跳和固定局部授权区段 |
| `/agt/research/navigation_mode` | NavigationMode，唯一模式权限发布器 |
| `/agt/local_row/state` | LocalRowState，QR、可观测净空和帧证据 |
| `/agt/local_row/path` | nav_msgs/Path，当前 odom 行道路径 |
| `/agt/odometry/quality` | OdomQuality，QO 与 odom epoch |
| `/agt/localization/quality` | GlobalQuality，正式定位管理器输出 |
| `/agt/localization/recovery_status` | RecoveryStatus，实际恢复和验证进度 |
| `/agt/base/state` | BaseState，驱动权限、急停、遥控、故障及真实速度 |
| `/agt/research/cmd_vel_source` | VelocityCommand，ResearchFollowPath 插件计算时生成 |
| `/agt/research/cmd_vel_smoothed` | VelocityCommand，保留原始命令时间、计划和 epoch |
| `/agt/base/cmd_vel` | 经过 C++ guard 的底盘速度 |

## 参数约束

- `q_global_low < q_global_high`、`q_row_low < q_row_high`，均在 `(0,1]`；QO 阈值也需标定。
- 全局注册使用独立 `global_quality_timeout_sec`；能力层对应
  `research_global_quality_timeout_sec / research_recovery_timeout_sec`。默认 3 秒仅是未标定的开发窗口，
  须按匹配器延迟和查询周期确定；不会放宽 row、odom、底盘状态的 0.3 秒源证据门。
- 距离、持续时间、位置/角度误差上界和速度上限由 RouteSegment 显式提供。
- 允许局部模式还需显式正值 `max_local_lateral_error_m / max_local_heading_error_rad`。
  本版仅接受单一前进行道的冻结路线：初始实测锚点、当前车位和局部感知路径均须在该路线的横向/航向边界内。
  越界、相邻行道或超过固定终点的感知路径不会成为新授权路线；复杂转弯和跨行道需新的全局区段。
- `braking_decel_mps2 / stopping_latency_sec / clearance_margin_m` 与几何感知、guard 使用同一实测值。
- 总停车延迟必须覆盖所配置的检测 watchdog 窗口；模式节点还覆盖 task heartbeat 和授权 TTL。
- 真实驱动状态缺失、源时间陈旧、速度测量无效、遥控介入、急停或故障会撤销研究权限。
- 模式节点、能力层超时和 C++ 命令链使用稳态时钟监视；ROS 时钟暂停/回退撤销当前权限。
- 此版本的可见性授权只覆盖前进扫掠。ResearchFollowPath 必须
  `allow_reversing=false`、`use_rotate_to_heading=false`。插件、smoother 和 guard 均拒绝倒车和原地旋转。
- RPP 原始 Twist 输出不连接车辆；研究速度只通过带来源时间与 epoch 的命令链。
- 原始 Livox CustomMsg 到 LIO 的点时间路径保持完整；车辆自体过滤只在辅助 PointCloud2 分支。

## 当前验证范围

代码完成了 ROS 2 Humble 的 `BUILD_TESTING=OFF` 编译和 Python 语法检查。没有新增或运行测试，
没有启动真实驱动或发出车辆运动命令。实际底盘协议、外参、制动、净空、质量分布、误差与终点容差
仍需现场数据确认；缺少这些参数时研究运动保持禁用。Nav2 成功后，相机采集仍须通过原有实测停车门。
