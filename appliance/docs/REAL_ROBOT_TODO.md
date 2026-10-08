# YHS 实机参数待办表（中文）

> 此表只记录**尚需现场确认的物理事实和安全验收证据**。模板路径是 `~/agt/profiles/yhs/`。未测量、未拿到 YHS 厂商驱动信息时不得填造数据，也不能套用 Bunker 的尺寸与速度限制。

## 第一优先级：能不能安全接 YHS 底盘？

| 核对项目 | 应填位置 / 如何确认 |
|---|---|
| YHS 具体型号、ROS1 Noetic 驱动仓库/commit/启动命令 | `robot.yaml`，记录驱动版本；必须知道真实型号 |
| CAN 口和波特率 | `base.yaml: can_interface/can_bitrate`；核对厂商协议和 `ip -details link show` |
| ROS1 Master 与启动环境 | `base.yaml: ros1_master_uri`；验证宿主机 Noetic 与厂商 workspace |
| 速度/里程计/底盘状态/急停话题 | `topics.yaml: ros1_cmd_vel/ros1_odom/ros1_chassis/ros1_estop` |
| 消息类型与适配 | Gateway 需要 `geometry_msgs/Twist`、`nav_msgs/Odometry`、`std_msgs/String`（JSON）、`std_msgs/Bool` |
| odom 比例、左右履带方向、角速度正负 | 物理遥控小幅移动，核对数据与机器人真实方向 |
| 厂商驱动掉线时是否自动停机 | `base.yaml: driver_watchdog_sec/driver_watchdog_verified`；分别停止 ROS1、ROS2 Gateway 观察**真实**停机 |
| 物理急停和遥控优先级 | 人员在旁监护，确认急停真实断开动力；ROS Bool 不能代替物理急停 |

### 为什么这里不能跳过？

ROS2 Gateway 断连可以发零速度，但**ROS1 厂商驱动自己崩溃时，零速度不一定能发出去**。必须验证厂商驱动/CAN 控制器的独立命令超时停止机制，而不是仅用软件 Mock 报告来证明安全。

## 第二优先级：URDF、普通 MID360 和静态 TF

| 核对项目 | 应填位置 / 如何确认 |
|---|---|
| `rotation_frame` 名称 | `robot.yaml: rotation_frame: base_footprint`，这是 V3 的软件约定 |
| `base_footprint` 的真正地面 X/Y 运动学参考点 | 实物结构、履带几何及转向观测；`calibration/base_geometry.yaml` |
| `base_footprint → base_link` 高度 | 当前 **+0.1041 m** 是 CAD 最低点估算，不能当成实测值 |
| `base_link → lidar_link` | CAD 给定 `[0.346672506,0,0.589682277]`、Pitch +15°，检查安装位姿及是否为 LiDAR 光学原点 |
| MID360 内置 IMU 相对点云原点 | 普通 MID360 厂商值 `T_lidar_imu=[0.011,0.02329,-0.04412]` m；核对设备型号与 driver frame |
| 真实 URDF / 可选 STL meshes | `robot_description/urdf` / `meshes`；候选无 STL 可发布 TF |
| FAST-LIO 的内参/外参 YAML | `localization.yaml: fastlio_config`，同一文件用于建图和导航 |
| LiDAR/IMU/底盘校准的审阅者及证据 | `calibration/lidar_extrinsics.yaml`、`imu_extrinsics.yaml`、`base_geometry.yaml`、`calibration_version.yaml`；只有验证后才标 `VERIFIED` |

提示：FAST-LIO 用 `T_imu_lidar=[-0.011,-0.02329,+0.04412]` m 和单位旋转矩阵。**15° 不写进 `r_il`**，只属于车体到雷达安装变换。更换车体基准原点时必须同步变换雷达外参。

[打开完整中文 MID360/TF 说明](../assets/yhs_mid360/README.md)。

## 第三优先级：雷达网络、建图、地图投影

| 核对项目 | 应填位置 / 如何确认 |
|---|---|
| 工控机以太网口、IP、MID360 IP | `sensors.yaml`、真实 `livox_config` JSON；雷达走 Ethernet/UDP |
| Livox `CustomMsg` 与 IMU 单位、频率和时间戳 | `ros2 topic list -t`、`ros2 topic hz`、检查 `header.frame_id` |
| 点云与车体的静态 TF 唯一性 | 避免驱动、URDF、static broadcaster 多方重复发布；唯一 `map → odom` |
| PCD2Grid 投影和障碍物相对地面高度 | `mapping.yaml: projection_config`；对坡地、地膜、细杆分别审核 |
| footprint、padding、自过滤盒、传感器离地参考 | `navigation.yaml`，必须实测，不用 STL 示意盒尺寸冒充导航尺寸 |
| 前进/倒车/转弯最高速度及加减速度 | `navigation.yaml: motion_limits`；先低速后逐级测试 |
| 停车速度阈值与最长等待时间 | `base.yaml: stop_linear_mps/stop_angular_radps/stop_timeout_sec`；实测停止过程 |

## 今天下午建议如何划分结果？

- **可先完成（不启用软件运动）：** Docker/Qt、传感器、TF、静止 LIO、物理遥控采图、PGO 导出、MapStudio 审核、Map Bundle 生成与哈希校验。
- **需要额外门禁：** 集成 Runtime 中的自动重定位启动也要求真实运动参数完整；在未通过时只做安全的独立定位/离线测试，不绕过 Guard。
- **必须完整通过后才能做：** Qt 手动控制、Nav2 单点、多点停顿与急停故障注入。

每做完一项运行 `./agt doctor --report`，记录配置版本、操作人、现场时间及 `~/agt/diagnostics/` 下的报告。真实里程计、点云时序、BBS+GICP 错匹配、导航误差和停车距离必须靠实机数据验证，不接受云端 Mock 当作实车 PASS。

[安装说明](installation.md) · [R0–R13 验收表](real_robot_acceptance.md)
