# R3.1 独立复核（2026-09-25）

结论：原 P1a/P1b/P2 均有对应实现，但机械臂启动与监控仍有以下缺口，软件闭环暂不验收通过。未启动实车、真实机械臂或相机。

## P1：stop_picker 不能撤销正在等待的启动

`agt_arm_capability/capability.py` 的 `_start_picker` 在进入 `_gate` 前检查 fault；`_gate` 默认 should_cancel=False。另一个回调执行 `_stop_picker` 时仅锁存 fault、终止当前 picker，没有撤销等待操作。随后门通过，原操作直接 `_spawn`，不再检查 fault。`_startup_spawn` 同样存在该路径；Action 等待只看 goal cancel，不看 manual stop/fault。

独立无硬件探针调用真实 `_start_picker`，在 fake `_gate` 返回 OK 前执行 latch_fault('manual stop_picker')；记录结果：stop latched → SPAWN CALLED，response.success=True，fault 仍为 manual stop_picker。

修复要求：统一操作取消 token/代次；stop/fault 使在途等待失效；取得门结果后与 spawn/trigger 在同一同步边界内复核，不能仅在函数入口检查。测试三个入口在等待期间 stop、门刚通过时 stop、并发确认/启动。

## P1：LIO 的数据时间新鲜度没有验证

`_on_odom` 用 time.monotonic() 接收时间入样本，完全忽略 header.stamp。频繁投递同一旧姿态会被认为是连续新鲜的静止数据。独立探针重复传 header.stamp.sec=1 的同一 Odometry，接收时间模拟 100~101.2s，真实 StationaryMonitor 返回 (True, 'stationary')。

修复要求：同时检查 ROS 时间下数据 stamp 的年龄/未来偏差、时间戳递增和消息间隔；等待超时仍用 monotonic；明确回放时 use_sim_time 规则。拒绝时间冻结、旧包连续到达和无效姿态。测试 feeder 也必须填真实时间戳，否则现有测试隐藏该缺口。

## P1：服务/自动启动没有保持整个作业的监控

`_start_picker` 和 `_startup_spawn` 在 `_spawn` 返回后立即执行 `_release`，将 running 与 _monitor_motion 关闭；但注释与旧脚本行为均表明 spawn 已开始第一轮采摘。只有 Action 路径持续等待 wait_cycle。

结果：服务/自动启动触发的第一轮实际作业期间，不会执行所声明的底盘运动锁存；进程退出也不经过 Action 的异常分支。`_heartbeat` 没有 alive→dead 的故障锁存逻辑。当前 permission 仍可能拒绝未知状态，但不能据此宣称失败已锁存、后续启动必须人工确认。

修复要求：三种入口复用同一完整作业生命周期；如服务异步返回，由后台作业持有占用直至结束，持续监控并收集异常。补服务和自动启动后的移动、进程退出、状态丢失测试。

## 其他边界

- _monitor_motion 只在收到 odometry 且 reason=base_translating/base_rotating 时锁存。作业中 LIO 不再发布、guard hold 失效，没有定时检测；应明确降级/故障策略，至少锁存并阻止任务继续，不能静默报告完整成功。
- guard payload_hold=True 也可由许可 missing/stale 产生，故它目前表示“保护在阻止运动且输出零”，不严格表示“已收到本次 deny”。建议确认关联本次作业请求，避免复用先前确认。
- 物理 installed 决定互锁与故障锁存优先于 waiting 的修改方向正确；这不消除上述启动竞态。
- 测试文件重写已明确披露；可以审核当前覆盖，但没有原内容不能证明与原测试等价。不要把“重写”标为“恢复原文件”。

## 验证范围

阅读交接和现有日志（声称 126 passed），独立重新执行五组 pytest，结果在本文件末尾记录。本次未重建或操作真实设备。上述两个探针使用真实方法和可控替身，不发布 ROS 命令、不启动 picker。

独立复跑：arm 30、mission 24、base_control 8、robot_bringup 37、system_bringup 27，共 **126 passed**；五组真实退出码均为 0。使用隔离 ROS_DOMAIN_ID=94。现有测试可复现通过，但未覆盖上述缺口；不能用通过数替代这些场景的验证。
