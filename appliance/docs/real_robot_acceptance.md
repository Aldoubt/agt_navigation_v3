# YHS 现场 R0–R13 验收清单（中文）

> **按顺序验收，有一项失败就停止，不要直接跳级。** 每项记录操作时间、人员、底盘型号、profile SHA/地图 Bundle hash、日志或 bag。现场必须有物理急停、遥控优先权与人员监护；`STOP ALL` 是软件停止命令，**不能代替硬件急停**。全部实车 Gate 默认 PENDING。

| 步骤 | 要做什么 | 通过时必须留下什么证据 |
|---|---|---|
| **R0：Docker/Qt** | 安装、`./agt up`、双击中文 AGT YHS Control、`doctor --report` | Qt 可见、容器健康、卸载/重启不丢地图和配置 |
| **R1：CAN 静止测试** | 物理急停保持有效、`./agt can up`、核对接口/bitrate | CAN 波特率与厂商一致、无错误、底盘不动 |
| **R2：ROS1 状态和里程计** | 启动已审计的 YHS 厂商驱动，只读状态，不启用速度链 | chassis/estop/odom 新鲜、里程计尺度与符号正常、无竞争 TF |
| **R3：低速方向** | 优先使用物理遥控检查前后、角速度正负、停车与遥控优先权 | 明确测得方向、零速度与停止行为；软件 teleop 暂缓到 R6/R9 之后 |
| **R4：URDF/TF** | 检查车体参考点、实际雷达 Pitch +15°、厂家 IMU 固定关系 | `base_footprint → base_link → lidar_link → imu_link` 正确，只有一个 `map→odom` 发布者 |
| **R5：MID360+IMU** | 启动雷达、检查以太网包、消息类型、频率、时间戳、外参 | LiDAR/IMU 正常、静止 LIO 输出稳定 |
| **R6：静止定位预检** | 已有经过验证的地图，机器人静止启动定位 | LIO 新鲜、定位状态真实、未 READY 不允许运动 |
| **R7：实机建图** | 新 Map ID/version，物理遥控驾驶，`Stop & Build Map` | 原始 bag 正常关闭、PGO 优化、PCD、keyframe patches、poses、checksum 有效 |
| **R8：地图审核封存** | 原生定位资产、PCD2Grid、MapStudio 审核→Confirm/Seal→Activate | Mapping/Localization/Navigation 三类资产同源；Bundle READY 和 hash 校验通过 |
| **R9：自动全局重定位** | 不使用 initialpose，在不同行道位置多次冷启动 | BBS+GICP 正确位姿、耗时/成功率、跟踪连续性；没有错配邻行 |
| **R10：单点低速 Nav2** | 在测得的低速约束下给一个可达点 | 局部障碍反应、TF 稳定、Motion Guard 正常、停车阈值实测 |
| **R11：三点与停顿** | 航点 P1=5s、P2=20s、P3=0s | 每个 Nav2 Action 成功后才计停留时间，最终任务 COMPLETED |
| **R12：暂停/恢复/取消** | 行驶中暂停、停留中暂停、取消、定位丢失、Gateway 掉线 | 目标真正取消、不会重放旧目标、及时归零、恢复前重新检查门禁 |
| **R13：30 分钟运行** | 重复路线＋录包＋断开重连＋STOP ALL | CPU/内存/磁盘、时延、真实 watchdog 触发记录、bag 正常关闭且无孤儿进程 |

## 两项不能误解的安全规则

### R3 为什么软件 teleop 需要往后放？

V3 的 Motion Guard 要求新的、有效的 LocalizationStatus。刚到现场没有完成 R6/R9 时，**不能制造假的 `LocalizationStatus=READY` 或把 `require_localization_status` 改为 false**。先由物理遥控确认方向与 odom，定位有效之后再重复软件低速控制测试，最后才能进入 R10。

### Gateway 掉线后一定会停车吗？

软件链路可在超时后尝试发零速度，但如果 **ROS1 Gateway 本身死掉**，能否停车取决于真实 YHS driver/CAN 控制器的独立 watchdog。必须分别停止 ROS2 Runtime、ROS1 Gateway，断开 CAN/MID360，验证实际底盘在已测量期限内停止；驱动看门狗数值只有实测后才能写 `driver_watchdog_verified: true`。

## 一次性报告和保存路径

~~~bash
./agt status
./agt logs
./agt doctor --report
~~~

日志和报告保存在宿主机 `~/agt/logs`、`~/agt/diagnostics`，rosbag 在 `~/agt/bags`，Map Bundle 在 `~/agt/maps`。诊断 ZIP 默认不装入大体积 bag；在验收表里记录相应 bag 的实际路径即可。

[返回中文安装指南](installation.md) · [普通 MID360/TF 参数](../assets/yhs_mid360/README.md) · [实车参数待办](REAL_ROBOT_TODO.md)
