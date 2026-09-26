# 整机配置改造独立复核（2026-09-25）

结论：完成了 Bunker 配置加载的局部阶段，但存在配置传递缺陷，暂不通过该阶段完整验收；更不代表整个工作空间收敛目标完成。交接报告明确将范围收窄到 Bunker 母板与接口预留，是否批准这一收窄需按用户与外部工具的实际约定判断。

## 发现

1. **P1：外部配置路径丢失。** `scripts/run_field_stack.sh:273` 使用用户路径预检，但读取结果时没有保存 `robot_config_dir`，`:620` 只传 `robot_config:=$ROBOT_CONFIG_ID`。硬件 launch 在安装目录按 ID 重新解析。用临时 bunker_inspection 配置将 CAN 改为 can9，预检解析 can9，模拟正式启动按 ID 再解析得到 can0。外部配置若用新 ID，则会预检通过后在硬件启动失败。需传规范化配置路径或同一份已验证快照，并增加跨入口回归测试。
2. **P2：模式覆盖设备 YAML。** `scripts/run_field_stack.sh:267` 始终以 ENABLE_INSPECTION 覆盖相机开关，`:622` 再次传递。因此 navigation 无论 YAML 默认如何均关闭相机，inspection 均要求开启。保留旧命令行为可以理解，但不满足“显式整机 YAML 决定设备、任务模式独立”的目标。应明确旧命令兼容策略与显式配置优先级，并分别测试。
3. **任务闭环未完成。** Mission/Arm/guard 未在本轮实现互锁；`agt_mission_runtime/runtime.py:272` 任务失败且 on_failure=skip 会继续，无机械臂可行驶条件。机械臂真实接入不可放行。缺实测参数不妨碍实现默认拒绝、状态新鲜度、fake 状态和互锁测试。
4. **现有测试证据覆盖有限。** experiments/robot_config_evidence/launch_selection_check.py 将名为“run_field_stack navigation”的样例手工设成 enable_camera_gimbal=true，与实际脚本 false 不符。它只打印结果，捕获异常后不做预期断言，不能单凭退出成功验收。需测试实际脚本传递的参数及动作选择。
5. **机械臂阻塞需细化。** 原 kiwipickingandmove.py:2052 附近已包含 injstep_neg 返回目标及 target_diff=0.03、0.2s 检查循环；该阈值是关节绝对误差之和，不等于已经批准的逐关节行驶容差。可以继续提取与验证，不能将“未取得 home”理解为完全没有可审计材料。

## 独立执行的检查（均未启动硬件）

- python3 -m pytest -q agt_robot_platform/agt_robot_bringup/test/test_robot_config.py：25 passed。
- navigation + bunker_inspection dry-run：成功；MID360/Bunker 开、camera/RTK 关，FAST-LIO2，auto 定位。
- inspection + bunker_inspection + --rtk dry-run：成功；MID360/Bunker/camera/RTK 开。
- 临时外部配置双阶段解析：复现 can9 → can0；临时目录已自动清除。
- 静态核对 Mission launch：inspection 启动 Mission，同时显式 enable_legacy_inspection=true；机械臂默认不启用，未见本轮任务互锁接入。

本次没有重跑构建、实机、TF、完整 Action 集成或回放；不能据此推断它们通过或失败。

## 当前入口

- run_field_stack.sh --mode navigation：硬件→LIO→初始化定位→Nav2/Health/Capability；无 Mission、无相机；可附加 --rtk、--rviz。
- --mode inspection：增加相机、Mission 与旧巡检兼容链；不等于新的纯 Mission 闭环。
- --robot-config yhs_harvesting：明确拒绝，尚不支持 YHS 运行。
- 仅硬件通过 robot_hardware.launch.py；其他模式仍分散于原 launch/建图/回放工具，未形成七种统一 CLI 模式。
- HMI 单独启动；它不是后端启动器。现有导航 --robot bunker_v1 仍受兼容支持。

后续优先级：修外部路径传递→明确设备/任务配置优先级→改真实参数回归测试→补互锁与 fake 集成→YHS 真驱动/Profile→部署与现场验收。保留现有分支与用户修改，不以全文件覆盖方式回退。
