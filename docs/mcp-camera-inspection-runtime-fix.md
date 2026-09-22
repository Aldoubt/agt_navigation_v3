# V3 巡检任务修复与上车测试前置清单

**日期：2026-09-21｜基线：c7c3452ecccb58d2847b65db1e3d1c25cbe17302｜本次未提交、未推送 Git**

## 结论与边界

已完成 V3 巡检任务层的修复、宿主机单包构建和软件验证，可进入**有人监督、具备遥控／急停条件的单点实车联调**。

这不等于实车验收通过，也不代表可无人值守巡检。本轮没有启动真实硬件／导航 launch，没有操作 CAN、云台串口或真实相机，没有向实车发送导航目标或速度指令。真实 ROS 测试只使用隔离 domain 与随机命名空间内的模拟服务。

## 1. 已修复内容

### 1.1 executor 兼容等待

- 移除任务执行器中的 `asyncio.sleep()`。
- 新增基于 **rclpy Future＋STEADY_TIME timer** 的等待器，直接兼容 Humble 的 ROS executor。
- 底盘停稳等待、额外稳定时间、阶段间暂停均使用该等待方式。
- 任务 watchdog 和里程计接收新鲜度使用单调时间；图像／TF／GNSS 关联继续使用消息时间，没有混改传感器时基。
- 保留原停稳速度、位姿差和持续时间门槛，没有靠放宽阈值解决问题。

### 1.2 Action 总超时与晚到目标

导航和相机子 Action 都有应用层总预算，覆盖服务发现、目标握手及执行结果等待。不会再直接无限 await 目标提交／结果 Future。

握手超时后仍保留 pending goal 的处理能力：如果目标晚到才被接受，会继续请求取消，而不是丢掉句柄让旧目标失去管理。

### 1.3 取消确认与故障闸门

- “取消请求已受理”不再等同于“动作已经结束”。
- 确认子 Action 的结果或状态已进入 SUCCEEDED／CANCELED／ABORTED 等终态。
- 导航相关失败／取消后的收尾，还使用原有门槛观察底盘是否停止。
- 不能确认子 Action 结束，或不能观测到所需底盘停稳时，任务进入失败并锁存故障，拒绝新巡检目标。
- 新增 `/agt/mission/reset_fault`。它不会发出运动目标；只有不存在活动任务、相关子 Action 已终态、里程计新鲜且持续静止时才允许解除本节点故障闸门。
- HMI 取消通过父 Action 的取消协议完成，避免把正在拍必需照片时的用户取消误报为普通拍摄失败。
- 同时提交多个巡检任务会被拒绝；尚未实现的 `resume=true` 明确拒绝，不会默默从头重跑。

**这个闸门不是硬件制动器，也不拦截遥控器或其他独立 Nav2 客户端。** 未确认停止时，应先按现场安全规程人工处理，而不是反复点 Start 或强行清状态。

### 1.4 成功／失败／取消终态与记录

统一更新三处结果：

| 情况 | ROS Action | MissionStatus／HMI | manifest.json |
|---|---|---|---|
| 正常结束 | SUCCEEDED | COMPLETED | `completed` |
| 用户取消且收尾确认 | CANCELED | CANCELED | `canceled` |
| 任务失败／超时／未确认状态 | ABORTED | ERROR | `failed` |
| 进程退出收尾 | 受退出时通信条件限制 | 尽力处理 | 尽力标记 `interrupted` |

- 必需照片失败也会先写入采集记录，再结束任务，保留相机原始错误码和消息。
- 任务目录新增 `events.jsonl`，记录阶段、子 Action 接受／取消／终态等事件。
- manifest 增加计划视图、记录数量、结束时间、完成点数、失败点／视角、故障锁存状态等附加字段；终态采用原子替换写入。
- 保留原 `captures.csv` 字段合同；更丰富的错误说明保存在 JSONL／事件记录。
- 相机声称成功但文件缺失／为空、归档失败或图像时间戳无效，不再默默当作有效必需采集。
- 终态一旦提交，晚到回调或退出清理不会把失败／取消改成成功。
- 校验器和报告生成器同步检查任务终态。即使照片已拍齐，若返程失败、任务被取消或仍在运行，也不能显示为完整任务 PASS。

**历史记录兼容提示：** 旧 manifest 缺少 `status` 时，新校验器默认不认证任务完成。显式使用 `validate_records --allow-legacy` 只能做结构检查，并输出“任务完成未验证”，不能拿它给旧记录补作成功证明。已有 `failed/canceled/running/interrupted` 状态不会被该开关放行。

断电、SIGKILL、永久磁盘故障无法承诺最终记录已落盘。若看到残留 `running` 而没有可信终态，不能把它当作完整成功批次。

## 2. 保持不变的内容

已与本次基线逐字节核对，以下内容未改：

- `run_field_stack.sh` 的两种模式及正式分阶段启动入口；
- hardware、localization、navigation launch；
- 导航算法、控制器、代价地图、感知、安全及车辆配置；
- 默认三视图角度、必需／保存开关；
- 原停稳阈值与 0.8 秒持续静止门槛；
- `ExecuteInspectionMission.action` 和 `MissionStatus.msg` 接口定义。

未修改底盘协议、C1 云台协议、相机驱动、TF ownership、地图内容或 RTK 策略。相机仍需停稳后按视角顺序拍摄，没有改成边走边拍。

## 3. 默认预算与故障处理

配置文件：`navigation/nav2/agt_navigation_runtime/config/runtime.yaml`。

| 配置项 | 默认值 | 语义 |
|---|---:|---|
| `nav_action_timeout_sec` | 300 s | 每个 Nav2 子 Action 总预算 |
| `camera_action_timeout_sec` | 30 s | 每个 AcquireView 子 Action 总预算 |
| `goal_response_timeout_sec` | 5 s | 目标提交握手上限，同时受总预算约束 |
| `cancel_confirm_timeout_sec` | 5 s | 触发取消／超时后额外的终态确认窗口 |
| `runtime_poll_interval_sec` | 0.05 s | ROS executor 内的检查间隔 |
| `stationary_timeout_sec` | 原值 8 s | 测量停稳窗口，收尾确认也复用该门槛 |

这是应用层 deadline，不是操作系统硬实时或机械制动保证。触发超时后还可能需要取消确认和停车观察时间；人为暂停可以保持，直到恢复或取消。同步磁盘写入的极端阻塞不属于本轮已证明的硬实时边界。

新增／相关时间参数必须是有限正数，停止阈值须合法；活动任务期间拒绝修改这些安全／时序参数和时基，避免靠临时改值让正在等待的任务“通过”。如需按路线调整预算，应在任务未运行时修改配置。

常见结果：

| 码 | 说明 |
|---|---|
| 1200 | Nav2 子 Action 总预算／相关等待超时 |
| 1201 | 相机子 Action 总预算／相关等待超时 |
| 1300 | 子 Action 终态未确认，故障闸门锁存 |
| 1301 | 收尾时无法观测到底盘停稳，故障闸门锁存 |
| 1302 | 父任务取消协议未能得到所需确认 |
| 1400 | 图片归档／成功回包的图像时间戳等异常 |
| 1401 | 终态记录保存失败 |
| 1402 | 进程退出时的 interrupted 记录 |

原有 C1 错误（如 301 无新图）保留，而不是全部包装成通用 202。最终应同时看 Action 状态、manifest `status` 和错误详情，不只看一个数字。

## 4. 验证结果

环境：宿主机 Ubuntu 22.04／ROS 2 Humble、Python 3.10.12；模拟 ROS 测试使用 `rmw_fastrtps_cpp`、`ROS_DOMAIN_ID=217`、`ROS_LOCALHOST_ONLY=1`。

| 检查 | 实际结果 |
|---|---|
| `agt_navigation_runtime` 定向构建／symlink install | 1 个 Python 包完成；未全量重编 C++ 工作区 |
| 最终完整测试套件 | **66 passed，0 failures，0 errors，0 skipped** |
| 其中单元／静态测试 | 44 项 |
| 其中真实 ROS executor＋模拟服务 | 22 项 |
| 隔离 ROS 场景重复执行 | **22 passed**；是重复验证，不计成额外独立用例 |
| 两种模式 dry-run | navigation：camera=false、inspection=false；inspection：两者均 true |
| 安装模块与工作树对应 | 已确认修复模块经当前 install／egg-link 加载到本次源码 |
| `git diff --check` | 通过 |

覆盖的主要场景：

- 一点三视图加 RETURN_HOME 正常完成，照片与位姿记录落盘；
- 必需相机失败保留原始错误，停止后续点；可选视图失败明确记录；
- Nav2／相机结果超时及服务不可用；
- Action 取消、HMI 取消、阶段暂停时取消；
- 握手期间取消、目标响应晚到后的取消；
- 取消 ACK 已收但无终态、取消被拒绝等场景；
- 未确认状态时拒绝新任务，终态和新鲜停稳证据具备后才允许人工复位；
- 里程计失效时不拍照／不复位，图片不存在时不报成功；
- ROS 仿真时钟不推进时看门狗仍能超时；
- 非法或任务中途的安全时间参数修改被拒绝；
- 终态写入失败不会误提交成功，事件追加失败不能推翻已原子提交的终态；
- 三张照片已经保存、但 RETURN_HOME 失败时，校验器和报告仍明确标记任务未成功。

模拟测试的每次节点／Action／话题／TF 使用随机前缀，未启动真实 Nav2、底盘、Livox 或 C1 驱动。模拟照片仅是生成的测试文件，全部位于实验测试目录，未混入真实巡检归档，也不能作为真实相机画质或导航精度证据。

测试采用较短预算验证边界，不是让每个生产默认 300 秒预算都实等一次。故障注入场景中的 `MISSION FAULT LATCHED` 错误日志是预期结果，JUnit 中相应用例通过。

构建日志中有当前同名包已存在于 underlay 的提示；本轮为同工作区 Python 包定向重建，未改变 ROS 消息接口或 C++ ABI，构建已完成。没有据此宣称全工作区冷安装通过。

## 5. 实际文件与证据位置

源码修改集中在：

```text
src/agt_navigation_v3/navigation/nav2/agt_navigation_runtime/
  agt_navigation_runtime/
    mission_runtime.py
    record_writer.py
    validate_records.py        # 校验终态，显式区分旧档案结构检查
    generate_demo_report.py    # 报告任务终态，不只统计已有照片
    action_execution.py        # 新增：Action 预算、确认与晚到目标管理
    ros_wait.py                # 新增：executor 原生定时等待
  config/runtime.yaml
  package.xml                  # 补齐 rcl_interfaces 依赖声明
  test/
    test_action_execution.py
    test_record_terminal.py
    test_validate_records.py   # 增加终态与历史记录检查
    test_mode_contract.py
    test_runtime_isolated_ros.py
```

宿主机本次验证目录：

```text
/home/yangxuan/ros2_ws/experiments/camera_runtime_validation/20260921_e0a2918e/
  build.log
  final-tests.log
  final-tests.xml
  repeat-ros-tests.log
  repeat-ros-tests.xml
  navigation-dry-run.txt
  inspection-dry-run.txt
  verification-summary.json
  ros_logs/
```

本次关键模块 SHA-256：

- `mission_runtime.py`：`8ad59e8aba9c75b2497010d5ecbba4100ab5e849e06eb7f8753ddfc8edfd5433`
- `action_execution.py`：`804baedbfd57c13da1cda8789896fa929df8e8426ff2d70b1126b00f9a3d1f36`
- `ros_wait.py`：`cebaac1ef022763dcf4243dee46db4f22e19cc18af71405960cca56e9a6fc457`
- `record_writer.py`：`cc741ff2b6fcf671e3c398ee2225159beff01b8ce7b96f666698ebd51ec62778`

## 6. 两个模式如何继续使用

**旧 Python 进程不会因为源码已修改就自动热更新。** 先按现有现场规程结束旧任务并确认停车，再退出旧栈、重新 source 和启动。不要在车辆运动中靠杀任务进程代替取消／停车确认。

先做不启动硬件的检查：

```bash
cd /home/yangxuan/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

src/agt_navigation_v3/scripts/run_field_stack.sh --mode navigation --dry-run
src/agt_navigation_v3/scripts/run_field_stack.sh --mode inspection --dry-run
```

真正上车前，人员、场地和设备安全准备完成后，仍使用原两个入口。**下面两条会启动真实硬件／运行时；本轮没有执行：**

**两种模式只选其一；切换前先安全结束并退出旧栈，不要连着执行。**

纯导航（不启动相机和巡检任务节点）：

```bash
src/agt_navigation_v3/scripts/run_field_stack.sh --mode navigation --rviz
```

相机巡检（到点、停稳、拍照，再进入下一点）：

```bash
src/agt_navigation_v3/scripts/run_field_stack.sh --mode inspection --rviz
```

当前脚本默认使用现有 `v005-confirmed-keepout` 地图包，本次未改。换场地前必须核对地图、标定、实际位置及所有点位，不能照搬文档里的示意坐标。

### 故障闸门处理

收到 1300／1301 时，不应继续发送新巡检或切另一模式绕过状态。先人工／遥控确认实际停车，检查日志和子动作状态。

只有满足软件检查条件时，才可人工请求复位：

```bash
ros2 service call /agt/mission/reset_fault std_srvs/srv/Trigger '{}'
```

该服务不发运动目标，也不自动恢复任务。若它拒绝复位，保持安全状态并排查通信／动作／里程计；不要手工改变量强行放行。

## 7. 上车单点验收顺序

1. **静态准备**：遥控／急停可用；CAN、LiDAR、里程计、唯一 TF owner、地图版本及时间源正确；C1 角度方向和机械中心已确认；存储空间和输出目录权限充足。
2. **相机独立确认**：在安全、静止条件下确认 C1 的实际到位、取新帧、保存和取消行为。公共 Action 终态不等于机械停止已经被本轮验证。
3. **纯导航基线**：只选择已知自由的近点，确认到点和停车；检查纯导航不会创建新的巡检任务归档。
4. **单点三视角**：进入 inspection，人工排一个安全点，执行默认三视角；检查任务终态、三张真实 PNG、captures 记录和图像时间的位姿关联。
5. **取消检查**：在经评估安全的条件下验证取消，观察真实停止与记录；不要通过行驶中拔线等危险方式模拟故障。软件故障注入已在隔离环境做过。
6. **短路线**：单点通过后才增加少量点位和 RETURN_HOME；任务点数仍包含返回点，不能直接当作采集对象数。

默认保存位置未改：

```text
C1 原图：~/autolabor_c1_capture/YYYYMMDD/
任务归档：~/.ros/agt_inspection_records/<mission_id>_<timestamp>/
```

重点检查 `manifest.json` 的终态和错误、`events.jsonl` 的动作链，以及真实图片；不要只看目录存在或服务可见。

## 8. 尚未证明的内容

- 实车定位／制动精度、真实云台机械停止和果园通行能力；
- 真实照片清晰度、小目标可见性和合作方平台导入；
- 操作系统／存储极端卡死下的硬实时保证；
- 强杀／断电后的动作终止与自动续跑；
- C1 底层协议本身的取消语义，仍沿用原实现。

本次完成的是任务层软件修复和受控测试前置条件。实际车辆与相机验收必须在上述安全流程下继续，不得把软件测试通过当作无人值守许可。
