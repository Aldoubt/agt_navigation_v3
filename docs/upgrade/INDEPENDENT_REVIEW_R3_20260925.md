# R3 独立复核（2026-09-25）

结论：软件实现已有实质进展，但以下缺口仍阻止机械臂整机闭环验收。本次未启动真实驱动、相机、机械臂或发送实车目标。YHS 继续 BLOCKED 是正确的；无法远程代替物理现场验收。

## 必须修复

### P1：开始采摘前缺少底盘停稳门和互锁确认

`agt_arm_capability/capability.py` 的 `_goal` 无条件接受；`_execute` 设置 running 后直接 `_spawn()` 或 `picker.trigger()`。许可由独立 10 Hz timer 发布，未等待消费者收到禁止，也未订阅 LIO 验证持续静止。Mission runtime 在导航成功后直接调用任务，Navigation Capability 的成功只检查 Nav2 终态和健康状态，没有测得静止门。

因此独立 arm_cli pick、/arm/start_picker，或导航完成但仍在减速时，都可能先启动机械臂、后收到底盘停车命令。三层“机械臂动时底盘停”的拦截不等于“底盘已停后机械臂才动”。

要求：在机械臂能力入口统一获取作业互锁，先发布禁止并确认命令链停止，再以新鲜 LIO 连续静止判据放行；超时/缺数据拒绝启动脚本。覆盖独立 CLI、service、Mission 和多周期，新增移动中请求采摘、停稳消息丢失及并发导航测试。不得仅在 Mission 添加等待。

### P1：installed=true、enabled=false 会解除机械臂互锁

`agt_robot_bringup/tools/robot_config.py:427` 使用 installed AND enabled 推导 payload_interlock。关闭机械臂软件并不能证明物理机械臂已收好。另 `navigation.launch.py:112` 在 robot_config 为空时默认 required=False，单独 launch 的保护边界也须明确。

要求：按物理 installed 决定必须互锁；disabled、无状态发布者时保持拒绝。任何豁免必须是单独且明确的已验证机械固定流程，不能复用 enabled 开关。测试安装但禁用、无配置、过期许可及显式关闭冲突。

### P2：invalidate 没有锁存故障，等待状态可再次放行

`DrivePermission.invalidate()` 只清 operator_confirmed。可离线复现：invalidate 后 evaluate(False, True, True) 返回允许。与“取消/失败后必须重新人工确认”的声明不一致。默认 kill_on_cancel 可覆盖部分路径，但异常、未杀死进程、配置 kill_on_cancel=false 等路径仍需审计；is_waiting 也没有检查 tainted。

要求：显式锁存失败/取消/超时，不因 PTY 仍处于等待模式自动解除；验证人工确认的清锁规则，并测试故障后进程仍活着且 raw stdin 的场景。

## 已确认进展

- 配置路径已改用配置快照传给硬件；显式 bunker_inspection 的 navigation dry-run 相机为 true，RTK false，payload_interlock false，解决了上一轮模式强制关闭相机的问题。
- guard、Capability、Mission 均新增许可检查；Mission 后置门位于 skip 之前。
- YHS 依赖本地 HEAD 为 6d002bd，适配器有单位转换、输入超时零速及默认禁倒车处理；本次未独立验收真实 CAN 协议、档位或制动效果。
- 等待按键判据目前基于 PTY raw 模式和旧脚本控制流，并非连续关节状态反馈；不能称为实测机械臂状态监控。实机验收需验证故障、外部手动操作及进程暂停的处理。

## 测试范围

独立按交接中的七组 pytest 路径重新运行，直接收集每组退出码，未使用 tail 管道作为成功判据。结果见本文件后续记录。未重编译，也未重复 CAN 或独立 CLI 冒烟。

原 r3_tests.sh 缺少 pipefail，构建和 pytest 输出管道接 tail；以后应保存每组真实退出码，避免前段失败被隐藏。

## 后续

先补上述代码与离线回归，再提交新一轮软件复核。YHS 真实档位、Profile、传感器外参和现场动作仍由具备现场条件的人员确认。agt_arm 尚无 Git 归属，部署可复现性仍未完成。不要将未提交改动或 fake 测试成功表述为整轮交付完成。

独立复跑结果：Robot Bringup 35、YHS adapter 10、Arm 18、guard 6、Navigation Capability 4、System Bringup 24、Mission 23；总计 **120 passed**，七组退出码全部为 0。说明现有测试可复现，但未覆盖本报告指出的缺口。
