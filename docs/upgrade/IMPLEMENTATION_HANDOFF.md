# IMPLEMENTATION_HANDOFF — 整机配置收敛（Bunker 母板 + 机械臂预留）

日期：2026-09-24。范围按用户最新指示收窄为：**以 Bunker 为母板理清整机配置架构，预留机械臂/YHS 接口**。
本文件不宣称任何实机验收通过；由独立审查者验收。

## 1. 执行环境与基线
- MCP 命令运行在 Docker 容器（无 ROS）；构建/测试经容器内 `ssh yangxuan@172.17.0.1` 在**宿主机**（Ubuntu 22.04.5 + ROS 2 Humble）执行，工作空间同一份文件。
- 审计时宿主机无 ROS 栈运行，`can0` 不存在（未接车）。本轮**未启动**任何驱动、底盘运动、机械臂动作或导航目标。
- 修改前基线：`~/ros2_ws/experiments/consolidation_baseline_20260924_221646/`（17 个仓库的分支/HEAD/status/diff/untracked）。
- 未提交、未推送、未合并；导航 main 与 refactor 分支保持分离。

## 2. 实现了什么 / 复用了什么
| 项 | 内容 | 仓库 |
|---|---|---|
| 整机配置目录 | `agt_robot_bringup/config/robots/{bunker_inspection,yhs_harvesting}/{robot,devices}.yaml` | agt_robot_platform |
| 加载器 | `agt_robot_bringup/tools/robot_config.py`：纯 Python，校验 schema、选择、冲突、阻塞；CLI `list/resolve`（exit 0/2/3） | agt_robot_platform |
| 硬件入口扩展 | `robot_hardware.launch.py` 新增 `robot_config`；设备默认值改由 YAML 提供（launch 默认 `""`=取配置）；原 profile 校验逻辑并入加载器 | agt_robot_platform |
| 编排脚本扩展 | `run_field_stack.sh --robot-config`；解析失败时不启动任何进程；dry-run 打印 robot_config 与硬件开关；去掉硬编码 `bunker_can_port:=can0`（改由 devices.yaml） | agt_navigation_v3 |
| 测试 | `agt_robot_bringup/test/test_robot_config.py`（25 用例，接入 ament_cmake_pytest） | agt_robot_platform |
| 静态测试同步 | `test_launch_ownership_static.py` 中"camera 默认开启"断言改为检查 bunker_inspection/robot.yaml | agt_navigation_v3 |
| 文档 | `agt_robot_bringup/README.md` 新增整机配置、字段、解析规则、机械臂预留契约 | agt_robot_platform |

复用：Robot Profile（`bunker_v1.yaml` + schema.json，未修改）、原 launch include 结构、run_field_stack 编排、mapping live_launch（未修改，参数仍有效）。
**未修改**：控制参数、运动学、外参、Nav2、LIO、Mission、agt_arm、HMI、Mapping。

## 3. 值的归属（消除双重默认）
- 几何/外参/footprint/速度 → Robot Profile（agt_robot_description）。robot.yaml 只引用 `robot_profile`，测试禁止出现几何字段。
- 装了哪些设备、默认是否启用 → robot.yaml；端口/设备路径/相机模式 → devices.yaml（数值原样搬自旧 launch 默认值）。
- 已消除的冲突：旧 `robot_hardware.launch.py` 默认 `enable_rtk:=true`，与 profile `gnss.enabled:false`、run_field_stack（默认关）不一致。现单一来源 `rtk.enabled:false`，`--rtk`/`enable_rtk:=true` 显式开启。**这是直接调用 robot_hardware.launch.py 时唯一的行为变化。**

## 4. 可用命令
当前可用（已 dry-run / 选择逻辑验证，未实机）：
```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run agt_robot_bringup robot_config.py list
src/agt_navigation_v3/scripts/run_field_stack.sh --mode navigation --map auto --robot-config bunker_inspection --dry-run
src/agt_navigation_v3/scripts/run_field_stack.sh --mode navigation --map auto --robot bunker_v1 --dry-run   # 旧命令，等价
```
实车（需现场前置检查，由用户执行）：同上去掉 `--dry-run`；巡检加 `--mode inspection`，RTK 加 `--rtk`。

**尚不可用（会明确拒绝，不要当示例执行）**：`--robot-config yhs_harvesting`（exit 2，BLOCKED）。

## 5. 测试证据（宿主机，日志在 `~/ros2_ws/experiments/robot_config_evidence/`）
| 验证项 | 命令/日志 | 结果 |
|---|---|---|
| 变更包构建 | `colcon build --packages-select agt_robot_bringup --symlink-install` → build_agt_robot_bringup.log | PASS（rc=0） |
| 加载器单元测试 | `colcon test --packages-select agt_robot_bringup` → colcon_test.log | PASS（26 tests, 0 failures） |
| YAML 校验、冲突、未安装设备、阻塞 | test_robot_config.py | PASS |
| Bunker 默认行为回归（参数级） | `test_bunker_defaults_equal_previous_hardware_launch` + launch_selection_check.log | PASS：无参/`robot:=bunker_v1`/`robot_config` 均启动 description+MID360+bunker_base+C1；`--rtk` 追加 asensing |
| Mapping 调用兼容 | launch_selection_check.log（live_launch 参数）| PASS：只启动 livox Node |
| YHS 缺依赖明确失败 | launch_selection_check.log、dryrun_yhs.log | PASS：launch 与脚本均拒绝，无回退 |
| run_field_stack dry-run | dryrun_*.log（7 组） | PASS：bunker 3 种写法一致；yhs/冲突 rc=2 |
| --show-args | show_args.log | PASS |
| system_bringup 静态测试 | pytest_system_bringup_static.log | PASS（9） |
| robot_description 测试 | pytest_robot_description.log | PASS（8） |
| mapping live mode 测试 | pytest_mapping_live.log | PASS（9） |
| 源码归属检查 | source_ownership.log | FAIL（既有问题：agt_arm 两包无 Git；agt_rviz_route_tools manifest 未跟踪）——非本轮引入 |
| 真实 launch 进程/节点/TF 唯一性 | — | NOT_RUN（未接车，不启动驱动） |
| 实车 Bunker 回归 | — | NOT_RUN（需用户现场执行） |
| 轮速/LIO/任务/互锁/回放相关项 | — | NOT_RUN（本轮范围外，代码未改） |
| 机械臂互锁与可行驶状态 | — | BLOCKED（契约已预留，home 关节位/容差未取得） |

复现：
```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select agt_robot_bringup --symlink-install
colcon test --packages-select agt_robot_bringup && colcon test-result --test-result-base build/agt_robot_bringup
python3 experiments/robot_config_evidence/launch_selection_check.py   # 只构造 launch 动作，不启动进程
```

## 6. 变更文件清单（均未提交）
agt_robot_platform（新分支 `refactor/robot-config`，基于 main @e407c88）：
- M `agt_robot_bringup/CMakeLists.txt`、`package.xml`、`README.md`、`launch/robot_hardware.launch.py`
- A `agt_robot_bringup/tools/robot_config.py`、`test/test_robot_config.py`、`config/robots/**`

agt_navigation_v3（refactor/navigation-runtime-v4，文件原本已有他人未提交修改，仅做定点插入）：
- M `scripts/run_field_stack.sh`（修改前副本：experiments/robot_config_evidence/run_field_stack.sh.before）
- M `bringup/agt_system_bringup/test/test_launch_ownership_static.py`（替换 1 条断言）
- A `docs/upgrade/IMPLEMENTATION_HANDOFF.md`

## 7. 阻塞项与待确认
1. YHS 底盘：无 ROS 2 驱动（仅 ROS 1 源码+产物），型号/CAN 协议/运动学未核实。
2. `yhs_v1` Robot Profile：几何、footprint、坐标系、外参需实测。
3. YHS 导航雷达/IMU 型号与安装未核实。
4. 机械臂可行驶状态：需从原采摘代码取得 home 关节位与容差；互锁（cmd_vel_guard 强制零速 + Capability 拒绝 + Mission 不前进）仅写入契约，未实现。
5. agt_arm 不是 Git 仓库；real 配置含本机绝对路径和 kiwi_env —— 不可复现。
6. 既有 Mission 风险：`on_failure: skip` 在任务失败后直接去下一点，未检查机械臂状态（本轮未改，接入机械臂前必须修）。

## 8. 风险与回退
- 回退：`git -C src/agt_robot_platform checkout main`（新文件需手动删除 config/robots、tools/robot_config.py、test/）；run_field_stack.sh 用 `.before` 副本对比回退本轮插入段；重建 agt_robot_bringup。
- 风险：直接调用 robot_hardware.launch.py 的外部脚本若依赖旧默认 RTK=开，现在需显式 `enable_rtk:=true`。

## 9. 下一步实机验收（用户执行）
前置：车辆静止、遥控器在手可接管、急停可用、周围无人。
1. `run_field_stack.sh --mode navigation --map auto --robot-config bunker_inspection --dry-run`，核对 hardware_enable_* 与旧命令一致。
2. 去掉 `--dry-run` 启动；预期日志含 `[robot_config] bunker_inspection profile=bunker_v1 base=bunker`，overrides 与 dry-run 的 hardware_launch_args 一致，`/livox/lidar`、`/livox/imu`、`/wheel/odom` 出现。
3. 运行 `ros2 run agt_robot_bringup check_robot_bringup_contract.py` 与 `scripts/check_tf_publishers.sh`：robot_state_publisher、bunker_base、MID360 各 1 个；`map→odom` 仅 localization manager。
4. 巡检模式重复一次（`--mode inspection`），确认 C1 启动；再加 `--rtk` 确认 asensing 启动。
5. 执行 `--robot-config yhs_harvesting`，确认立即拒绝、无任何节点启动。
停止条件：任一节点重复、TF 多发布者、车辆出现非预期运动、日志中 overrides 与 dry-run 打印的 hardware_launch_args 不一致 → Ctrl+C 有序退出并保留 `~/.ros/agt_field_stack/<时间戳>/`。

---

## 10. 独立复核（INDEPENDENT_REVIEW_20260925）处理记录 — R2

| 复核发现 | 处理 | 状态 |
|---|---|---|
| P1 外部配置路径丢失（预检 can9，启动按 ID 得 can0） | 脚本读取 `robot_config_dir`；启动前把**通过预检的目录**快照到 `$RUN_DIR/robot_config/<id>/`，并把该路径传给 `robot_config:=`；解析结果写入 `$RUN_DIR/robot_config/resolved.txt` | 已修复并测试 |
| P2 模式覆盖设备 YAML | 规则集中在脚本一处：`inspection` 必须有相机（强制开，未安装则拒绝）；`navigation` + 显式 `--robot-config` → YAML 决定；`navigation` 且未给 `--robot-config`（旧命令）→ 相机关闭，和原来完全一样；`--rtk` 强制打开 RTK。预检和启动用同一组 override。dry-run 打印 `camera_policy` 与 `hardware_launch_args` | 已修复并测试 |
| #4 证据只打印、样例参数不真实 | 新增 `tools/hardware_launch_plan.py`（只构造 launch 动作，不启动进程）+ `test_hardware_launch_plan.py`（带断言）；新增 `agt_system_bringup/test/test_run_field_stack_robot_config.py`：**执行真实脚本**，用导出的 bash 函数拦截 `setsid`（start_child 启动 launch 的唯一入口）和 `ros2 topic/node`，记录实际 argv，并对硬件 launch 计划做断言。旧的 `launch_selection_check.py` 已作废，不再作为验收证据 | 已补 |
| #3 任务闭环/互锁 | 未实现（按用户"预留机械臂"的范围）。Mission 里 `on_failure: skip` 的风险仍在；机械臂真实接入**不可放行** | 未做，需用户决定是否进入下一轮 |
| #5 机械臂阻塞项要细化 | 静态审计了原脚本：生效的返回位 `injstep_neg`（第 1559 行）、`sum|dq|<0.03 rad`、0.2 s×100 次检查、ready marker 在检查之前打印。已写入 README 和 yhs 阻塞项；0.03 rad 是六个关节误差之和，不能当逐关节行驶容差 | 已细化（仅静态审查） |

### R2 测试证据（宿主机，`experiments/robot_config_evidence/`）
| 验证 | 日志 | 结果 |
|---|---|---|
| 编译 agt_robot_bringup | build_r2.log | PASS |
| bringup 测试（加载器 25 个 + launch 计划 7 个） | pytest_bringup_r2.log | PASS（32 passed） |
| 真实脚本参数传递（旧导航命令 / 显式配置 / inspection+rtk / 外部路径 can9 / 外部新 ID / yhs 拒绝 / 冲突拒绝） | pytest_run_field_stack_r2.log | PASS（7 passed）；测试后宿主机上无 ros2 launch 进程 |
| 负对照：同一 can9 用例跑修复前的脚本（run_field_stack.sh.before_r2） | negative_control_p1.log | 复现缺陷：`bunker_can_port=can0` |
| dry-run 策略矩阵 | dryrun_r2_*.log | PASS：旧导航命令 camera=false；显式配置 camera=true（来自 YAML）；inspection+rtk 两者都开 |
| system_bringup 静态测试 | — | PASS（9） |
| 实机、TF、Action 集成、回放 | — | NOT_RUN |
| 机械臂互锁 | — | BLOCKED / 未实现 |

复现：
```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select agt_robot_bringup --symlink-install
python3 -m pytest -q src/agt_robot_platform/agt_robot_bringup/test
python3 -m pytest -q src/agt_navigation_v3/bringup/agt_system_bringup/test/test_run_field_stack_robot_config.py
```

### 行为变化（R2）
- `--robot-config bunker_inspection --mode navigation` 现在**会启动相机**（YAML 为 `enabled: true`）。只想要导航不要相机时，用旧命令（不带 `--robot-config`），或把 YAML 改为 `enabled: false`。
- 每次实车运行都会在日志目录保存一份整机配置快照。

### R2 变更文件（均未提交）
- agt_robot_platform：A `tools/hardware_launch_plan.py`、`test/test_hardware_launch_plan.py`；M `CMakeLists.txt`、`README.md`、`config/robots/yhs_harvesting/robot.yaml`
- agt_navigation_v3：M `scripts/run_field_stack.sh`（R2 前副本 `run_field_stack.sh.before_r2`）；A `bringup/agt_system_bringup/test/test_run_field_stack_robot_config.py`

## 11. R3：机械臂互锁 + YHS 驱动 + 组合绑定 + 机械臂任务化（2026-09-25，均未提交）

> 全部为离线/fake/rclpy 证据。没有启动真实底盘、机械臂、相机或导航目标。实机验收由独立复核人执行。

### 11.1 范围与结论

| # | 项 | 状态 | 证据 |
|---|---|---|---|
| 1 | 机械臂互锁（fail-closed，覆盖所有软件运动入口） | 离线 PASS / 实机 NOT_RUN | `experiments/arm_interlock_evidence/r3_tests.log`、`guard_payload_*.log`、`mission_*.log`、`arm_cli_smoke.log` |
| 2 | YHS 驱动（官方 TK-mid-ros2 @6d002bd）+ `agt_yhs_adapter` | 编译 PASS、桥接 smoke PASS / CAN 实机 NOT_RUN | `build_yhs_driver.log`、`yhs_bridge_smoke.log` |
| 3 | 组合绑定：Bunker=云台+RTK；YHS=机械臂+奥比中光 TOF | 配置/解析 PASS | `dryrun_r3.log`、bringup pytest 35 passed |
| 4 | 机械臂融入任务框架（cycles / stop_on_no_target / 路线离线校验 / arm_cli） | 离线 PASS | mission pytest 23 passed、`arm_cli_smoke.log` |
| – | `yhs_harvesting` 仍 BLOCKED | 预期 | 缺 yhs_v1 profile、导航雷达未核实、档位未确认、驱动未上车 |

### 11.2 互锁设计

* 话题 `/agt/payload/drive_permission`（Bool，10 Hz）+ `_reason`（String）。仅 `agt_arm_capability` 发布。
* True 仅当：无 goal 运行，且 (picker 存活并阻塞在 `wait_for_q()`（脚本自身 `sum|q-injstep_neg|<0.03 rad` 通过后才会进入）或 picker 未运行且操作员 `/arm/confirm_stowed` 被接受)。启动/取消/失败/超时/进程退出/stop_picker ⇒ False。
* 消费方均要求“新鲜 True”（0.5 s，未收到=False）：
  * `agt_cmd_vel_guard`：`require_payload_drive_permission`；导航与手动(HMI)两种模式硬停并清缓存指令，重新放行不回放旧指令。
  * `agt_navigation_capability`：拒绝 `navigate_to`/`follow_route`（`ARM_NOT_DRIVE_SAFE`），运行中丢失则取消 Nav2 goal。
  * `agt_mission_runtime`：每个导航目标前、每个 `arm.*` 任务后等待 `drive_permission_wait_sec`（默认 10 s），超时 FAILED `ARM_NOT_DRIVE_SAFE`，`on_failure: skip` 也不会越过。
* 开关：`navigation.launch.py payload_interlock:=auto|true|false`（auto 由 `robot_config` 决定，装了机械臂即开启；对机械臂机器人 `false` 报错）；`mission.launch.py enable_arm:=true` 强制开启；`run_field_stack.sh` 把 `robot_config:=<快照>` 和解析出的 `payload_interlock` 传给导航/任务 launch。Bunker 默认关闭，行为不变。
* 底盘急停/遥控器不受影响（硬件优先级）。

### 11.3 测试（主机 Ubuntu 22.04 + Humble）

| 包 | 结果 | 负对照 |
|---|---|---|
| agt_robot_bringup | 35 passed | – |
| agt_yhs_adapter | 10 passed | – |
| agt_arm_capability（含 fake picker rclpy 权限测试） | 18 passed | – |
| agt_base_control `test_guard_payload_interlock.py` | 6 passed | R3 前 guard：5 failed / 1 passed（`guard_payload_negative_control.log`） |
| agt_navigation_capability | 4 passed | – |
| agt_system_bringup（互锁 launch、归属静态、run_field_stack） | 24 passed | – |
| agt_mission（bt + runtime，含 fake 端到端） | 23 passed | R3 前 runtime：3 failed（`mission_gate_negative_control.log`） |
| YHS 桥接 smoke | 无档位拒绝启动；无输入 49.998 Hz 零指令；0.2 m/s,0.1 rad/s→0.2,5.73°/s；输入停止→零 | – |
| arm_cli smoke（fake） | UNKNOWN→pick×2→HOME_WAITING→stop→UNKNOWN→confirm→CONFIRMED | – |

复现：`bash experiments/arm_interlock_evidence/r3_tests.sh`、`yhs_bridge_smoke.sh`、`arm_cli_smoke.sh`。

### 11.4 行为变化

* Bunker 解析结果多一个 `enable_yhs_can=false`；dry-run 多打印 `payload_interlock=`。
* 导航/任务 launch 新增参数 `robot_config`、`payload_interlock`（任务另有 `drive_permission_wait_sec`）。
* 任务：箱式/相机任务失败码仍为 `TASK_FAILED`；`arm.pick_kiwi` 新增 payload `cycles`（1..50）、`stop_on_no_target`；`mission_route_runner --validate` 离线校验。
* 机械臂机器人启动后必须先让 picker 进入等待态或 `arm_cli confirm-stowed`，否则首个导航 10 s 后失败（有意为之）。

### 11.5 YHS 驱动审计要点

* 来源 `https://github.com/YUHESEN-Robot/TK-mid-ros2`，`6d002bddec319a9175a7ada9703f9e7bcb2a7d71`，放在 `src/drivers/yhs_tk_mid_ros2`（独立 git，detached 于该提交）。**上游无 LICENSE**，许可证未知。
* CAN ID 与本地 ROS1 副本一致（0x98C4D1D0/D2D0/D7D0，反馈 D1EF/D7EF/D8EF/DAEF；ROS2 另有 E1EF/E2EF/E8EF/E9EF）。
* 订阅 `ctrl_cmd`（非 cmd_vel），发布 `odom`（odom→base_link，积分，π 取 3.14）与 `chassis_info_fb`；**不发 TF**。
* 文档冲突：README 表格字段名（velocity/steering，4=D）与源码（linear/angular）及 PDF（3=运动学控制档）不一致 ⇒ 档位不猜，`drive_gear: null` 保持 BLOCKED。倒车语义未确认 ⇒ 桥接默认拒绝负速度。

### 11.6 实机步骤（用户执行，须有人监护、架空履带或急停在手）

1. `sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up`；`candump can0` 看到 0x98C4D1EF 等反馈。
2. 仅启动驱动（不启动桥接）：`ros2 run yhs_can_control yhs_can_control_node --ros-args -p can_name:=can0 -r odom:=/wheel/odom -r chassis_info_fb:=/yhs/chassis_info_fb`，确认 `ctrl_fb_gear` 等反馈正常。
3. 与厂家/DBC 确认档位与倒车语义后填写 `devices.yaml base.drive_gear`；在架空状态用桥接发 0.05 m/s 验证方向/单位/停止。
4. 测量并建立 `yhs_v1` profile、核实导航雷达后，逐条移除 `robot.yaml` blockers。
5. 互锁实机：机械臂运行中尝试 HMI 手动与导航目标，应全部被拦截；取消后必须 confirm-stowed 才能行驶。

### 11.7 回退

* 导航：`cp experiments/arm_interlock_evidence/cmd_vel_guard.py.before …/cmd_vel_guard.py`；`nav_capability.py.before`、`nav_policy.py.before`、`navigation.launch.py.before`；`scripts/run_field_stack.sh.before_r3`。
* 任务：`experiments/arm_interlock_evidence/mission_before/`（runtime.py、nodes.py、route_runner.py）、`mission.launch.py.before`；或 `git checkout main`（分支 `refactor/arm-interlock` 保留了原未提交改动）。
* 机械臂：`experiments/arm_interlock_evidence/agt_arm_backup_before_interlock.tgz`。
* 平台：删除 `agt_yhs_adapter/`，恢复 `robot_config.py`/`robot_hardware.launch.py`/YAML 至 R2 版本；删除 `src/drivers/yhs_tk_mid_ros2`。

### 11.8 R3 变更文件

* platform：`agt_robot_bringup/tools/robot_config.py`、`launch/robot_hardware.launch.py`、`config/robots/yhs_harvesting/{robot,devices}.yaml`、`test/test_robot_config.py`、`README.md`；新增 `agt_yhs_adapter/`。
* drivers：新增 `src/drivers/yhs_tk_mid_ros2`（上游克隆）。
* navigation：`agt_base_control/cmd_vel_guard.py`、`agt_navigation_capability/{capability,policy}.py`、`agt_system_bringup/launch/navigation.launch.py`、`scripts/run_field_stack.sh`；新增测试 `test_guard_payload_interlock.py`、`test_capability_payload_gate.py`、`test_payload_policy.py`、`test_payload_interlock_launch.py`。
* mission（分支 `refactor/arm-interlock`）：`agt_mission_bt/nodes.py`、`agt_mission_runtime/{runtime,route_runner}.py`、`agt_mission_runtime/package.xml`、`agt_mission_bringup/launch/mission.launch.py`；新增 `test_wait_drive_permission.py`、`agt_mission_runtime/test/`。
* agt_arm（无 git）：`capability.py`、`setup.py`；新增 `drive_permission.py`、`arm_cli.py`、两个测试、`docs/ARM_TASK_INTEGRATION.md`。

### 11.9 仍开放

* 每关节容差：目前沿用脚本自身的 sum<0.03 rad 回零判定；如需逐关节阈值需脚本发布关节角。
* 暂停时取消机械臂会杀掉脚本，恢复后重新启动脚本——脚本在非零位启动时的行为须实机确认。
* RTK 在 Bunker 组合中“已安装、默认关闭”（保持 R2 行为）；如需默认开启，把 `bunker_inspection/robot.yaml` 中 `rtk.enabled` 改为 true。

## 12. R3.1：独立复核（INDEPENDENT_REVIEW_R3_20260925）处理记录（2026-09-25，均未提交）

复核结论：R3 的 120 项自动化测试可以复现，但指出三个安全缺口。本轮全部修复，并用负对照证明新测试确实能抓到问题。**这不等于现场验收**：实车、真实 TF/Action、回放、真实机械臂依旧是 NOT_RUN。

### 12.1 P1a：机械臂开始作业前，底盘必须已停住（门控放在机械臂能力入口）
- 所有会让机械臂动起来的入口都走同一个门控：PickKiwi action、`/arm/start_picker`、`spawn_on_startup`；`arm_cli` 和多循环任务都经由 action 进入，所以也被覆盖。门控步骤：
  1. `_claim()` 占用作业，并立即发布 `drive_permission=False`；
  2. 等 `agt_cmd_vel_guard` 在 `/agt/cmd_vel_guard/payload_hold` 上回报 True。这个话题的含义是：互锁已启用、guard 收到了 deny、并且本周期输出为零；
  3. 等 `/agt/odometry/local` 连续静止：窗口 1.0 s，消息最大间隔/最大陈旧 0.3 s，位移 ≤0.01 m，偏航变化 ≤0.01 rad，判定依据是位姿本身，不信 twist；
  4. 在 `base_stop_timeout_sec` 内没满足，就返回 `ERROR_BASE_NOT_STOPPED=205`，picker 不启动。这种情况机械臂根本没动，所以不锁存故障。
- 作业过程中如果检测到底盘在动，锁存故障 `BASE_MOVED_DURING_ARM_WORK`。
- 新文件是 `agt_arm_capability/work_gate.py`（纯逻辑，可以单独测试）。同时新增错误码 `ERROR_FAULT_LATCHED=206`。
- 顺带修了 guard 的一个日志问题：原来 localization 回调只看定位状态，就打印 "motion gate OPEN"，而实际上 payload 仍在拦车（运动输出一直是零，只是日志误导）。现在日志报告的是综合门控状态。

### 12.2 P1b：互锁由“物理安装”决定
- `robot_config`：只要有一个 `installed: true` 的机械臂，互锁就开启，即使 `enabled: false` 也一样。
- `navigation.launch.py`：`robot_config` 为空时，按旧 profile 映射解析（bunker_v1 → bunker_inspection → 关闭）。没有映射的 profile 直接报错，不再默认 `required=False`。BLOCKED 的配置（yhs_harvesting）在 auto/true/false 三种模式下都会启动失败。

### 12.3 P2：故障锁存
- `DrivePermission.latch_fault()` 会在以下情况触发：取消、失败、超时、异常、进程退出、`stop_picker`、作业中底盘移动。锁存期间，PTY 即使回到 raw 等待状态，也不会自动恢复放行；新的 PickKiwi goal 会被拒绝（206）。
- `invalidate()` 只清除人工确认，不算故障。处于 `tainted` 的等待状态不放行。
- 解除锁存只能通过 `/arm/confirm_stowed`。条件是：没有作业在跑，并且（进程已退出，或进程处于未 tainted 的等待状态）。
- 心跳在作业期间不再轮询 PTY。

### 12.4 测试（宿主机，`experiments/arm_interlock_evidence/r31/`，脚本 `r31_tests.sh`：`set -o pipefail`，每组记录真实退出码）
见 `r31/summary.txt`，结果汇总在交付说明里。新增/改写的测试：
- `test_drive_permission.py`：复核里的复现用例（锁存后 PTY 处于等待也保持 False）、tainted 情况、确认规则。
- `test_work_gate.py`：缺失、陈旧、空洞、平移、旋转、NaN、guard hold 缺失/False/陈旧。
- `test_arm_capability_permission.py`：使用真实 guard、fake picker 和 fake LIO，覆盖以下场景：
  - 移动中请求 pick 被拒，picker 不启动；
  - 底盘停下满 1 s 才开始；
  - 里程计丢失时拒绝；
  - `start_picker` 同样被门控；
  - 取消后锁存（kill_on_cancel=false，进程仍活、PTY 回到等待，permission 仍为 False，新 goal 返回 206，confirm 后解除）；
  - 进程退出后锁存；
  - 作业中底盘移动时锁存；
  - `stop_picker` 后锁存；
  - 作业期间导航持续下发 0.3 m/s，guard 输出始终为零。
- `test_arm_drive_gate.py`（mission 端到端，真实 guard）：R3 版本的文件在本轮上传时被我误清空，已按原有 5 个用例重写，并新增“导航报告成功但底盘仍在滑行，机械臂必须等待”。
- `test_guard_payload_interlock.py`：新增 payload_hold 的两个用例。
- `test_robot_config.py`：新增“已安装但未启用”和“空配置”两个用例。
- `test_payload_interlock_resolution.py`：launch 解析行为。
- 负对照（`gate_negative_control.log`）：把 `_gate()` 改成直接放行后，移动中拒绝、里程计丢失、start_picker 门控、滑行等待这 4 个用例全部 FAIL；随后已恢复代码（备份在 `r31/capability.py.pre_negctl`）。

### 12.5 行为变化
- 机械臂能力节点现在必须收到 guard 的 payload_hold 和 `/agt/odometry/local`，否则拒绝作业。也就是说，**没有导航栈（guard + LIO）时，能力节点无法驱动机械臂**。独立使用机械臂的方式是直接运行原始 `kiwipickingandmove.py`，这条路径不在互锁范围内，由操作员负责底盘静止。
- 任何异常之后都必须人工执行 `/arm/confirm_stowed`，任务才能继续。

### 12.6 R3.1 变更文件
- `agt_arm/agt_arm_capability/`：`drive_permission.py`、`work_gate.py`（新）、`capability.py`、`package.xml`；
- `agt_arm/agt_arm_capability/test/`：三个测试文件；
- `agt_arm/agt_arm_interfaces/action/PickKiwi.action`；
- `agt_base_control/cmd_vel_guard.py`、`test/test_guard_payload_interlock.py`；
- `agt_robot_bringup/tools/robot_config.py`、`test/test_robot_config.py`；
- `agt_system_bringup/launch/navigation.launch.py`、`test/test_payload_interlock_resolution.py`（新）；
- `agt_mission_runtime/test/test_arm_drive_gate.py`（重写）、`package.xml`。

修改前的备份在 `experiments/arm_interlock_evidence/r31_before/`。

### 12.7 仍开放
- 静止阈值（0.01 m / 0.01 rad / 1.0 s）是保守的初值，需要在实车上用 LIO 的静止噪声标定。标定前如果噪声超过阈值，结果会是拒绝作业（安全侧）。
- 实车、TF、Action、回放、真实机械臂：NOT_RUN。
- agt_arm 仍然没有 git。

## 13. R3.1b：独立复核（INDEPENDENT_REVIEW_R31_20260925）处理记录（均未提交）

### 13.1 P1：stop_picker 必须能撤销还在等待中的启动
- 每次启动都持有一个操作令牌（generation）。`/arm/stop_picker` 会在锁内锁存故障并让代次 +1。等待中的门控把“令牌已失效或故障已锁存”当作取消条件，返回 `REVOKED`。
- 门控通过之后，`_commit()` 会在同一把锁内再复核一遍：令牌仍有效、未锁存、底盘仍然静止，并且 guard hold 是占用之后才收到的。复核通过才执行 spawn/trigger。`stop_picker` 和 `confirm_stowed` 用的是同一把锁，所以 stop 只可能出现在两种时刻：启动之前（启动被拒绝），或者启动之后（新进程会被终止）。
- 三个入口（action、`/arm/start_picker`、`spawn_on_startup`）都走 `_claim → _gate → _commit`。

### 13.2 P1：LIO 数据的时间新鲜度
- `StationaryMonitor.add()` 必须传入 `stamp`（取自 header.stamp）和 `ros_now`（节点的 ROS 时钟）。以下情况会拒绝样本并清空窗口：时间戳为 0、不严格递增（冻结、重复、乱序）、年龄超过 0.5 s、比当前时间超前 0.1 s 以上、数值非有限、四元数模长异常。
- 窗口内相邻时间戳间隔必须 ≤0.3 s，时间戳跨度必须 ≥ hold−0.3 s。接收时间仍按 monotonic 判断陈旧、空洞和超时。
- 回放规则：`use_sim_time=true` 时 ROS 时钟就是 /clock，bag 的时间戳和 bag 时间比较；不开 sim time 时，回放的旧时间戳一律拒绝，即 fail-closed。
- 测试里的 feeder 现在都填真实时间戳。复核里的探针（stamp.sec=1 反复投递）已经做成单元测试和节点级测试，结果都是拒绝。

### 13.3 P1：整个作业期间持续监控
- 服务和自动启动在 spawn 之后由后台线程执行 `_finish()`，一直持有占用（drive permission 为 False），直到出现 ready marker 且 stdin 回到 raw 状态，或者出错；每一种非成功结果都会锁存故障。Action 也复用同一个 `_finish()`。
- 20 Hz 看门狗：作业中只要 `base_stopped` 不成立（底盘移动、LIO 缺失/陈旧/空洞/时间戳异常、guard hold 丢失）就锁存 `BASE_NOT_HELD_DURING_ARM_WORK:<reason>`。当前这一轮允许做完，但结果是 FAULT_LATCHED，不会报告成功。
- 心跳：picker 在空闲时从 alive 变成 dead，会锁存 `PICKER_EXITED_WHILE_IDLE`。锁存只保留第一个原因。

### 13.4 payload_hold 的含义收紧
- guard 只在满足以下全部条件时才发布 hold=True：互锁已启用、**收到了新鲜的 False**、并且输出为零。permission 缺失或陈旧虽然也会让底盘停，但不再算作确认。
- 机械臂端还要求这条 hold 消息是在本次占用之后收到的，避免沿用之前的确认。

### 13.5 测试（`experiments/arm_interlock_evidence/r31b/`，脚本 `r31b_tests.sh`，每组记录真实退出码）

| 组 | 结果 |
|---|---|
| arm_capability | rc=0，49 passed |
| mission | rc=0，24 passed |
| base_control | rc=0，9 passed |
| robot_bringup | rc=0，37 passed |
| system_bringup | rc=0，27 passed |

合计 146 passed。

新增的节点级用例：
- 冻结时间戳时拒绝；
- 服务在门控处等待时 stop，结果为 REVOKED 且不 spawn；
- Action 在门控处等待时 stop；
- 门控刚通过、spawn 之前 stop：对 action、service、startup 三个入口各测一次；
- spawn 进行中另一个线程 stop：新进程被终止，故障被锁存；
- 启动进行中请求确认：被拒绝；
- 服务启动后仍持有占用，此时底盘移动：锁存；
- 服务启动后进程退出：锁存；
- 自动启动后受监控，LIO 丢失：锁存；
- Action 作业中 LIO 丢失：结果不是成功；
- 空闲时进程被 SIGKILL：锁存。

guard 新增用例：permission 缺失或陈旧时不发布 hold。

### 13.6 负对照（`r31b_negctl.sh` → `negctl_summary.txt`，改完即恢复，sha256 校验一致）

| 变异 | 结果 |
|---|---|
| 去掉 commit 复核 | 3 个用例全部失败 |
| 去掉时间戳递增检查 | work_gate 的探针用例失败；节点级用例仍被“年龄检查”拦住（纵深防御） |
| 服务 spawn 后立即释放占用 | “持有占用并锁存移动”的用例失败；进程退出仍被空闲退出锁存捕获（纵深防御） |
| 关闭看门狗 | 2 个用例全部失败 |
| 关闭空闲退出锁存 | 1 个用例失败 |

### 13.7 说明
- `test_arm_drive_gate.py` 是按 R3 日志中的用例名**重写**的，不是“恢复”；无法证明它和原文件等价。
- 看门狗发现问题时只锁存，不中止当前这一轮，因为中途杀掉机械臂脚本的安全性未知。`stop_picker` 会中止并终止进程。
- 实车、TF、Action、回放、真实机械臂：NOT_RUN。静止阈值和时间戳年龄阈值需要实车标定。

### 13.8 R3.1b 变更文件
- `agt_arm_capability/capability.py`、`work_gate.py`；
- `agt_arm_capability/test/test_work_gate.py`、`test/test_arm_capability_permission.py`；
- `agt_base_control/cmd_vel_guard.py`、`test/test_guard_payload_interlock.py`；
- `agt_mission_runtime/test/test_arm_drive_gate.py`（仅给 feeder 加了时间戳）。

修改前的备份在 `r31/capability.py.r31` 和 `r31/work_gate.py.r31`。

## 14. R3.1c：处理独立复核 INDEPENDENT_REVIEW_R31B_20260925（均未提交）

### 14.1 P1：Action 取消没有参与“是否启动机械臂”的原子判断
**问题：** 门控通过后、提交启动前，如果 Action 取消已被接受，机械臂仍可能被启动。

**修复：**
- `_claim(owner=goal UUID)` 记录这次作业属于哪个目标。
- 取消回调 `_cancel` 会拿锁；只有当前作业的所有者目标发来的取消，才会把 `_op_canceled` 置为 True。
- `_commit` 在同一把锁内**优先**检查 `_op_canceled`。已被接受的取消会让本次启动直接返回 `CANCELED`：不 spawn，不写 trigger，也不锁存故障（机械臂没有动）。
- 门控等待和 `_finish` 也会检查 `_op_canceled`。
- 先后顺序由锁决定，只有两种结果：
  - 取消先拿到锁：机械臂不启动；
  - 提交先拿到锁：机械臂已启动，`_finish` 负责取消、锁存故障并终止进程。
- 取消回调运行时，rclpy 还没把目标切到 CANCELING。`_await_cancel_state()` 会等它切过去，再以 canceled 状态结束目标。

### 14.2 退出日志
**问题：** 测试退出时出现 “Destroyable ... never retrieved”。

**修复：**
- `destroy_node`：先让代次失效、终止进程、join 后台监督线程，然后置 `_closing`，停止发布，最后才销毁 ROS 实体。
- 测试夹具在拆除前等待作业结束，并且等待所有 `cancel_goal_async` 返回的 future 完成。

**结果：** 最终全量 arm 日志 `r31c/arm_capability_final2.log` 中，该异常出现 0 次（修复前为 2 次）。

### 14.3 测试（`experiments/arm_interlock_evidence/r31c/`）
**`r31c_tests.sh`（每组记录真实退出码）：**

| 组 | 结果 |
|---|---|
| arm_capability | rc=0，53 passed |
| mission | rc=0，24 passed |
| base_control | rc=0，9 passed |
| robot_bringup | rc=0，37 passed |
| system_bringup | rc=0，27 passed |

加上 foreign UUID 用例之后，arm 组最终复跑为 rc=0、54 passed，日志是 `arm_capability_final2.log`，其中 Destroyable 异常 0 次。

**新增用例：**
1. 门控通过后、spawn 前取消已被接受 → 从未 spawn，状态为 CANCELED，不锁存；
2. 已有 picker 时，门控通过后、trigger 前取消 → 没有写入 trigger，picker 仍在等待；
3. 取消与提交竞争（取消到达时提交正持有锁）→ 只启动一次，结果为 CANCELED，故障锁存，进程被终止；
4. 被 BUSY 拒绝的目标发来的取消 → 不影响当前作业；
5. 用其他目标的 UUID 直接调用真实取消回调 → 被忽略。

rclpy 不会把已经结束的 BUSY 目标的取消投递给回调，所以用例 4 只证明端到端行为；归属规则本身由用例 5 验证。

**负对照（`r31c_negctl.sh` → `negctl_summary.txt`，改完即恢复，sha256 一致）：**

| 变异 | 结果 |
|---|---|
| `_commit` 忽略取消 | 用例 1、2 失败 |
| 取消回调不校验所有者 | 用例 5 失败（用例 4 仍通过，原因见上） |

### 14.4 仍保留
- 作业中底盘保持条件失效时，当前策略是“锁存故障，但允许本轮结束”，需要现场做风险评估。进程终止不等于机械臂硬件已经安全停止。
- YHS 仍为 BLOCKED；实测外参和档位、真实 TF/Action、回放、agt_arm 的 Git 归属、异机部署都还未完成。
- `test_arm_drive_gate.py` 是重写的，不能证明与丢失的版本等价。
