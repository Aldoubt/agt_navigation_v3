# V4 Health 失联与 Mission 跳点安全收敛

日期：2026-09-25。状态：**软件局部验证通过；A5/A6 总体验收与 B/C 门均未签发**。

依据：[统一改造方案](upgrade/WORKSPACE_CONSOLIDATION_PLAN.md)、[V4 验收矩阵](acceptance/V4_ACCEPTANCE_MATRIX.md)。本轮选择可在隔离 ROS 域验证的运行收敛项；不调整控制器、几何、传感器外参、正式地图或运动参数，不启动任何驱动和运动目标。已有未提交修改原样保留，未合并、提交或推送。

## 起点与复现

- `agt_navigation_v3` 分支 `refactor/navigation-runtime-v4`（实施前 HEAD `798200c`）、`agt_mission` 分支 `refactor/arm-interlock`（实施前 HEAD `17c5b91`），两仓原本均为脏工作树。
- 已有 Navigation Capability 在收到错误 Health 时撤销 Nav2 目标，但 Health **不再发布**时只有结果检查，运行中的 Nav2 不会及时收到取消。
- Mission 的 `on_failure: skip` 原本不区分 `HEALTH_DEGRADED`、`NOT_READY` 等系统错误，且会丢失 Navigation Capability 的具体错误码，连续跳过后续航点。
- 先写隔离 Action 回归再改实现。原源码下的 [导航红测](../../../experiments/mcp_health_gate_20260925/nav_red.log) 为 **2 failed / 1 passed**；[任务红测](../../../experiments/mcp_health_gate_20260925/mission_red.log) 为 **4 failed**。红测中导航用例失败后的 teardown 曾有 `Destroyable` 噪声；之后已调整测试夹具先等待 fake action 退出，最终绿测无该报错。源码修改前副本保存在 `experiments/mcp_health_gate_20260925/{nav,policy,mission}.before.py`。

## 收敛的行为契约

1. [Navigation Capability](../capability/agt_navigation_capability/agt_navigation_capability/capability.py) 每 0.1 秒检查 Health：保持原有 1.5 秒新鲜度门，匹配机器人与地图版本，运动所需状态为 READY/BUSY。错误更新或停止更新都锁存一次原因；在途 Nav2 目标只发起一次取消，请求最终以 `HEALTH_DEGRADED` 而非成功结束。等待 Nav2 服务后、发送目标前再次检查 Health 与机械臂行驶许可。
2. [Health 策略](../capability/agt_navigation_capability/agt_navigation_capability/policy.py) 将 1.5 秒阈值命名为共享常量，避免接收门和看门狗两套时限漂移。已丢失的状态不会因为后来收到一条正常更新就在同一目标中重新放行。
3. [Mission Runtime](../../agt_mission/agt_mission_runtime/agt_mission_runtime/runtime.py) 保留 Capability 的原始错误码。新 [跳点策略](../../agt_mission/agt_mission_runtime/agt_mission_runtime/navigation_policy.py) 仅允许明确的单点 `NAV2_FAILED` 依据 `on_failure: skip` 跳过，并发布 `WAYPOINT_SKIPPED` 状态事件；定位/Health 降级、未就绪、机械臂互锁失败、超时及未知错误均终止整条任务。任务动作失败时既有策略与机械臂互锁保持不变。

## 验证与证据

| 项目 | 结果 | 记录 |
| --- | --- | --- |
| 原有纯逻辑基线 | 9 passed | 隔离 ROS 域内显式 pytest |
| 新用例，修改后首次 | 16 passed | [trial1.log](../../../experiments/mcp_health_gate_20260925/trial1.log) |
| 宿主机隔离构建 | `agt_navigation_capability`、`agt_mission_runtime` 两包通过 | [build.log](../../../experiments/mcp_health_gate_20260925/build.log)；仅使用 `experiments/mcp_health_gate_20260925/colcon_{build,install,log}`，未覆盖正式 build/install |
| 最终相关回归（含 R3 机械臂 fake picker 与真实 cmd_vel_guard） | 45 passed | [final_regression.log](../../../experiments/mcp_health_gate_20260925/final_regression.log) |
| 补充的显式 DEGRADED 锁存用例 | 导航测试文件 4 passed | [nav_extra_case.log](../../../experiments/mcp_health_gate_20260925/nav_extra_case.log) |
| 静态检查 | 新增/修改的 Python 文件 AST 可解析；相关仓库 `git diff --check` 通过 | 文件改动与修改前副本逐项比较 |

回归使用 `ROS_LOCALHOST_ONLY=1`、非零 `ROS_DOMAIN_ID`、`AGT_RUN_ISOLATED_ROS_TESTS=1`。导航用例运行真实 Navigation Capability 与伪造的 Nav2 Action/服务，覆盖 NavigateTo、FollowRoute、Health 静默过期、显式降级及健康心跳保持；任务用例运行真实 Mission Runtime 与伪造的 Navigation Capability Action，覆盖系统错误终止与可跳过的单点失败。[导航测试](../capability/agt_navigation_capability/test/test_health_watchdog_ros.py)、[任务 Action 测试](../../agt_mission/agt_mission_runtime/test/test_navigation_skip_ros.py)、[纯策略测试](../../agt_mission/agt_mission_runtime/test/test_navigation_failure_policy.py) 均为源码资产，默认不启动真实硬件。

## 未完成与下一门

- **不能把本轮标为 A5/A6 全部 PASS**：还需要在同一隔离 ROS graph 中串起 Supervisor → Navigation Capability → Mission，并验证取消不应答、真实 Health 变化与零误转发；跨文件地图异常、任务暂停/取消等原验收矩阵条件也未覆盖。
- 现有软件 `cmd_vel_guard` 的硬停策略未在本轮修改；纯 fake Action 不能证明底盘最终零速、TF 唯一或机械臂安全停机。仿真/回放 B 门、现场 C 门仍为 `NOT_RUN`，有急停和看护的现场验收必须单独执行。
- YHS 驱动档位、运动学、外参及机械臂硬件安全状态仍按统一方案标记 `BLOCKED`；本轮不推断实测值，也不解除阻塞。
- 如需还原本轮源码的最小差异，对比上述 `.before.py` 与当前文件后定点撤销；不要 `git reset`/`git clean`，以免抹去实施前已有修改。
