# R3.1b 独立复核（2026-09-25）

范围：上一轮 R3.1 的具体缺口、相关取消路径及五组离线测试。本次不修改运行代码，不操作真实硬件，也不替代现场验收。

## 已有对应修复的项目

- stop_picker 在锁内使操作代次失效；gate/commit 检查代次，commit 与 stop 共用 RLock。
- LIO 校验零时间戳、递增、过旧、未来、姿态有效性及采样间隔；拒绝时清空窗口。
- 服务与自启通过后台监督线程持有作业至结束；与 Action 共用 finish。
- 作业期间定时检查静止/观测有效性，失效锁存；空闲进程退出新增锁存。
- guard hold 只在收到新鲜 deny 且输出零时成立；机械臂要求本次 claim 后接收到 hold。

以上认可为源码层面的修复进展；具体实机行为仍须验证。日志中的五项负对照记录已阅读，本次未再次改写源码运行负对照。

## 剩余 P1：Action cancel 未参与启动提交的原子裁决

位置：`agt_arm/agt_arm_capability/agt_arm_capability/capability.py` 的 `_cancel`、`_commit`、`_execute`。

`_cancel` 仅返回 ACCEPT，不撤销操作 token。`_gate` 检查 goal.is_cancel_requested，但返回 OK 后到 `_commit` 执行之间仍可收到取消。`_commit` 只检查代次、fault、base_stopped，不检查该目标的取消。因此取消已被接受后仍可能调用 spawn/trigger；进入 `_finish` 后才处理取消并可能杀进程。

独立离线探针调用真实 `_execute` 和 `_commit`，将 gate 替换为“返回 OK 前把 goal.is_cancel_requested 置为 True”，其他硬件动作使用无副作用替身。结果：

```text
cancel accepted after gate check
SPAWN CALLED
最终返回 CANCELED
```

这个结果说明最终报告取消不等于避免了取消后的启动。现有“gate 与 spawn 之间 stop”测试只调用 stop_picker，不覆盖 Action cancel；中途取消测试也不覆盖启动前窗口。

修复要求：按目标归属将 Action cancel 接入同一操作代次/锁或等价同步机制，启动提交与取消必须有确定先后。不要让其他被 BUSY 拒绝目标的取消误撤销当前作业。仅在 commit 前多读一次布尔值仍存在检查到启动间的竞态。

新增回归：

1. Action 在 gate 返回后、spawn 前接受取消，spawn 从未调用。
2. 已有 picker 时，在 gate 返回后、trigger 前取消，不写入触发字符。
3. 并发提交取消与启动，验证同步顺序和目标归属，结束状态与实际行为一致。

## 验收界限

此轮比上一轮更完整，但上述取消路径修复前，不通过机械臂作业生命周期的软件闭环验收。此结论不否认已通过的独立子项。

作业失去底盘保持条件时“锁存但允许本轮结束”是当前实现策略，需要现场风险评估；不能把进程终止等同于机械臂硬件安全停止。YHS BLOCKED、实测外参/档位、真实 TF/Action、回放、Git 归属和异机部署仍按交接保留未完成状态。

重写的 test_arm_drive_gate.py 保留“重新编写、不能证明与丢失版本等价”的说明即可；不要求伪造恢复历史。

## 独立测试

五组 pytest 在 ROS_DOMAIN_ID=94 下顺序执行，直接记录真实退出码，无 tail 管道掩盖失败。测试结果追加于下方。

Arm 组独立结果：49 passed，rc=0。退出阶段出现多条 `The following exception was never retrieved: cannot use Destroyable because destruction was requested`。不把这些异常算成测试断言失败，但需整理后台线程/executor/node 的退出顺序，当前退出日志并非完全干净。

全部独立复跑完成：Arm 49、Mission 24、Base Control 9、Robot Bringup 37、System Bringup 27，共 **146 passed**，五组退出码均为 0。没有重建，也没有现场/回放验收；上述启动取消探针仍能在当前源码复现。
