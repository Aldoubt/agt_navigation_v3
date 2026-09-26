# YHS + 机械臂上车准备复核（2026-09-26）

## 结论与范围

R3.1c 的 Action cancel 归属和启动原子裁决已进入源码；独立针对该缺口的 5 项测试通过。整个工作空间升级和 YHS 真实采摘闭环尚未完成。保持 yhs_harvesting BLOCKED，不能仅删除阻塞标记来启动。

本次只读源码、执行 dry-run、5 项 fake/rclpy 回归和文件存在性检查；未重建、未复跑所有 151 项、未运行真实机械臂脚本或驱动。

## 独立证据

- `_claim` 保存目标 UUID；`_cancel` 在同一锁内按 UUID 设置取消；`_commit` 优先拒绝取消。旧问题的对应源码已修复。
- ROS_DOMAIN_ID=94，执行 test_arm_capability_permission.py，筛选 cancel_after_gate / cancel_racing_commit / cancel_of_busy / foreign_goal：**5 passed, 23 deselected，rc=0**。
- 测试退出仍出现多条 `cannot use Destroyable because destruction was requested`；本次无法复现交接报告“退出异常为零”的结论，需继续定位 teardown，不将其混同为上述断言失败。
- YHS navigation dry-run 明确 BLOCKED：缺 profile、外参/网络/验证地图、实车驱动验证和 drive_gear。
- `config/robot_profiles/` 只有 bunker_v1，没有 yhs_v1。
- 真实采摘配置指向的 Python 存在；脚本内 `/home/test/桌面/kiwi/jakakiwi/MIHOUTAO@.pt` 和 `JAKA A 5-V01_20240812.urdf` 本机不存在。

## 上车前软件准备

1. **采摘任务启动链尚未接通。** run_field_stack 仍只有 navigation/inspection。navigation 不启动 Mission/Arm；inspection 强制云台相机且启用旧巡检链，与 YHS 配置冲突。Mission launch 有 enable_arm，但默认 false，脚本不传；它还组合 navigation，不能在已启动导航后直接再开一份。需提供复用现有模块的 YHS 任务入口，传递整机配置、机械臂配置、专属 Nav2 配置，且保持硬件/LIO/Nav2 各一个 owner。Mission launch 当前未转发 nav_config_dir，亦需补齐。
2. **真实采摘依赖预检。** 修正缺失的模型/URDF 路径；核对 Python、JAKA SDK、Orbbec SDK、夹爪/串口权限、机械臂 IP（脚本当前 10.5.5.100）及标定。不要通过直接运行脚本做无动作检查：脚本会加载模型并可能命令机械臂。
3. **车型和参数。** 创建实测 yhs_v1 Profile、模型/TF；准备独立 robot/navigation/controller/costmap/perception/safety YAML。navigation.launch 已要求 YHS 专属配置和 field_profile.yaml，不能复制 Bunker 值或先把 field_verified 改为 true 代替验证。
4. **复现与文档。** agt_arm 尚无 Git；源码、SDK、模型分发和版本清单未冻结，运行改动未提交。更新导航操作文档给出验收后的真实入口。代码归档不是单项硬件检查的物理前提，但受控测试前应至少冻结可恢复的测试基线。

## 必须现场取得或核实的数据

- YHS 具体型号/运动学、CAN 参数、drive_gear（当前 null，文档 3/4 含义冲突）、正负方向、速度/角速度单位、制动/失联和遥控优先权。
- 车体及收臂状态的 footprint、运动限制、机械臂安装位置和允许行驶姿态。
- YHS MID360 与内置 IMU：独立网络 JSON、雷达和主机 IP、实际 XYZ/RPY、重力/TF 与 LIO 初始化。
- 与 yhs_v1 兼容且现场验证的地图包：定位 PCD、重定位资产、专属导航栅格。可复用既有环境资产须证明兼容，不能仅改 robot 名称。
- 采摘回位姿态和允许误差、LIO 静止噪声；确认软件“等待状态”与机械臂物理可行驶状态一致。
- 明确作业中失去底盘保持时的处置：当前仅锁存、允许本轮结束；以及 stop_picker 杀进程后真实机械臂是否停止。两者须由硬件语义验证。

## 分阶段现场顺序

A. 无运动：设备识别、接线/IP/CAN、TF/模型、传感器、LIO 静止数据；不发送驱动速度或采摘命令。
B. 底盘单项：确认档位/协议后，在受控条件下验证最低必要运动、方向、零速、命令超时、断链与遥控/急停；机械臂保持已确认收拢。
C. 原地机械臂：保持底盘不可自主移动，验证一次真实采摘、回位、取消与故障处理；明确真实停止机制。
D. 互锁：底盘未停拒绝抓取、作业期间新导航被拒、许可失效停车、传感器丢失锁存、取消后不自动续行。
E. 两站点闭环：定位→到点→LIO 停稳→采摘→确认可行驶→下一点；再测失败/暂停/取消。须先有兼容地图和完整任务入口。

当前可以准备并按现场条件实施 A 类静态检查；B/C 各自在协议/设备依赖/控制边界确认后单独推进。D/E 不能以软件 fake 测试通过替代前置条件。
