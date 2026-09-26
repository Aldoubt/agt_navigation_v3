# AGT 工作空间改造收敛与部署启动方案

日期：2026-09-24。状态：实施目标已收敛，软件集成与实机验收尚未完成。

本文件合并用户提出的 **V3.1 Workspace Upgrade** 与现有 **Navigation Runtime V4** 目标，作为本轮工作范围、实施顺序和最终使用形态的统一入口。V3.1/V4 是原有计划名称，本次不据此重新命名包或创建第二套实现。

原 [V4 变更计划](../../V4_CHANGE_PLAN.md) 保留阶段与基线记录；[V4 验收矩阵](../acceptance/V4_ACCEPTANCE_MATRIX.md) 继续管理验收门，[迁移报告](../../V4_MIGRATION_REPORT.md) 保存证据。本文不表示所有目标已实现，也不替代 TF、消息和地图资产的详细契约。

## 1. 本轮目标与实施原则

在已有模块化架构上，形成以 LIO 为运动依据、地图资产可追溯、启动与恢复行为明确、RViz/HMI 共用后端、可迁移到其他电脑的机器人运行系统。

实施顺序固定为：**审计 → 复用 → 收敛 → 补齐 → 测试**。优先解决启动不稳定、健康状态误判、行驶偏斜/摆动和任务中断；目录整理不能代替运行验收。

两份原计划合并后保留：

| 来源 | 保留内容 |
| --- | --- |
| V4 | 独立仓库边界、机器人配置、地图版本发布、Health/Capability、Mission 接口、RViz/HMI 共用后端、分层验收 |
| V3.1 | 全生命周期启动、七种工作模式、完整建图链、定位恢复和低频校正、回放兼容、审计优先 |
| 现场确认 | RViz 为调试终端，HMI 为最终上位控制；轮速只用于诊断；验收前不合并导航 main 与改造分支 |

不新建重复的启动栈、日志框架、点云过滤器或 benchmark。迁移前查调用关系，替代前做同数据对照；现有可用工具保留到替代方案通过验证。

## 2. 最终仓库边界与 src 预期目录

下图是**职责收敛后的预期结构**，只列主要目录；不是本次已执行的搬迁清单。现有可用路径优先保留，新归属需经过依赖审计。每个 ROS 包在工作空间只能有一份参与构建的源码。

```text
ros2_ws/
├── src/
│   ├── agt_navigation_v3/                  # 独立 Git：导航及运行编排
│   │   ├── bringup/                       # 系统 launch，组合外部包
│   │   ├── navigation/
│   │   │   ├── localization/              # LIO adapter、重定位、定位管理
│   │   │   └── nav2/                      # Nav2 配置、RViz 导航调试工具
│   │   ├── cleaning/                      # 现有局部感知；离线处理归属待审计
│   │   ├── runtime/                       # Supervisor、健康状态
│   │   ├── capability/                    # 对外导航 Action
│   │   ├── interfaces/                    # 导航接口
│   │   ├── tools/                         # 保留已有 benchmark/诊断工具
│   │   ├── scripts/                       # 复用 run_field_stack.sh 等
│   │   ├── dependencies/                  # 仓库清单与发布版本锁定
│   │   └── docs/                          # 本方案、契约、验收、操作说明
│   ├── agt_robot_description/              # 独立 Git：URDF、外参、Robot Profile
│   ├── agt_robot_platform/                 # 独立 Git：硬件启动与适配
│   │   ├── agt_robot_bringup/              # 硬件唯一启动入口
│   │   ├── bunker_ros2/                    # 当前底盘驱动
│   │   └── ugv_sdk/                        # 当前底盘 SDK
│   ├── agt_mapping_framework/              # 独立 Git：建图与离线地图生产
│   │   ├── bringup/                        # 建图 launch
│   │   ├── backends/                       # LIO、PGO 后端
│   │   ├── apps/                          # Map Studio
│   │   ├── artifacts/                     # 地图产物契约
│   │   ├── exporters/                     # 点云/栅格导出
│   │   ├── refinement/                    # 离线编辑与加工
│   │   └── scripts/                       # 实机建图、回放、校验
│   ├── agt_mission/                        # 独立 Git：任务语义
│   │   ├── agt_mission_interfaces/
│   │   ├── agt_mission_bt/
│   │   ├── agt_mission_runtime/
│   │   └── agt_mission_bringup/             # 组合任务与导航，不重复启动
│   ├── agt_arm/                            # 机械臂能力与 PickKiwi；源码归属需纳入发布审计
│   ├── agt_robot_hmi/                      # 独立 Git：最终上位控制界面
│   ├── drivers/                           # 外部硬件驱动仓库集合
│   │   ├── agt_ins_driver/                 # RTK/INS 记录与质量信息
│   │   └── Autolabor-C1-ROS2/              # 相机、云台驱动与动作
│   ├── shared/                            # 经审计确有跨系统复用的组件
│   │   └── agt_pointcloud_pipeline/        # 与现有组件对照后确定使用边界
│   └── external/                          # 第三方算法和 SDK，各自保留来源
│       ├── Livox-SDK2/
│       ├── livox_ros_driver2/
│       ├── fast_lio2_mapping/
│       ├── batch_lio/
│       ├── 3d_bbs/
│       ├── small_gicp/
│       └── Sophus/
├── maps/                                  # 独立分发的数据，不混入源码
│   ├── registry.yaml
│   └── <map_id>/<map_version>/             # 不可变正式地图包
├── build/                                 # 新机重新生成
├── install/                               # 新机重新生成
└── log/                                   # 构建日志
```

`drivers/`、`shared/`、`external/` 是组织目录，不表示各自是单一 Git 仓库。运行日志默认在 `~/.ros/agt_field_stack/`，bag 与大型地图单独保存。

硬件差异由 **Robot Profile + driver/adapter** 消化。导航算法不直接依赖 Bunker/YHS CAN、特定相机或 RTK 协议。导航仓库保留 ROS 接口依赖和系统编排；离线地图生产不迁入导航仓库。已有地图管理/转换组件逐项审计，不能仅凭目录名直接删除或搬走。

## 3. 运行数据与状态归属

- `/agt/odometry/local` 是导航运动判据，FAST-LIO2 默认，Batch-LIO 显式切换，两者互斥。
- `/wheel/odom` 可发布、显示、录包、对比，不参与导航就绪、静止判定、路径执行或健康拦截；RTK 同样不接管导航定位。
- `agt_localization_manager` 唯一发布 `map → odom`。
- 选定的 LIO adapter 唯一发布导航 `odom → base_footprint`；机器人模型提供到 `base_link` 及传感器的固定变换。审计整条链，不额外再发布一条重复 `odom → base_link`。
- 原始 Livox 点时序送入 LIO；自车、后方结构、ROI 等过滤在独立局部障碍支路完成。实时局部障碍不写入持久地图。
- 局部实时避障默认保留，关闭开关只用于受控对照实验。

目标系统阶段：

```text
BOOT → LOAD_CONFIG → HARDWARE_BRINGUP → PRECHECK
     → ODOMETRY_READY → LOCALIZATION → NAVIGATION_READY → TASK_EXECUTION
```

以上是统一的阶段语义，不表示目前已经存在全部同名状态枚举。配置和地图先静态检查；硬件起来后再检查消息、时间戳、TF 与单一发布者。

定位恢复单独建模：

```text
LOCALIZED → DEGRADED → LOST → RELOCALIZING → LOCALIZED
                 └─ 条件恢复且验证通过 → LOCALIZED
```

短暂消息延迟不应无条件触发全局重定位；确实失去有效定位时停止导航动作，恢复后由任务策略明确决定是否继续。每次转换记录原因、时间及判据；不得用放宽全部门限来掩盖误判。

全局重定位/GICP 用于初始化和恢复。低频地图校正是待验证目标，复用现有能力，经定位管理器校验后更新变换；当前 `enable_map_tracking=false`，不得宣称已持续纠漂。

## 4. 建图、导航与任务流程

**地图生产：** 传感器验证 → 同步录包 → LIO → PGO（按产物等级选择）→ 原始/优化点云 → Map Studio → 定位图、导航栅格、重定位库 → 质量分析 → candidate → 校验与发布 → 版本化地图包。

复用建图仓库现有实机、回放和导出脚本。现有正式交付要求的 PGO/质量门不因“可选 PGO”被取消；未通过门限的 LIO 原始产物可以诊断使用，但不能冒充已验收正式地图。

**导航：** 选择地图包 → 验证同版本资产及机器人兼容性 → 单一 LIO 就绪 → 全局搜索/GICP → LOCALIZED → Nav2 和健康检查 → NAV_READY → 接受动作。

**连续画线：** RViz 折线 → 重采样/朝向生成 → 预览校验 → `/navigation/follow_route` → 连续跟随。不要把每个采样点变成停车一次的任务点。

**带动作任务：** 路线/任务航点 → Mission → 导航能力 → 到点并通过 LIO 静止判据 → 拍照等 TaskGroup → 后续任务。

RViz 与 HMI 使用同一后端契约。RViz 是调试终端，HMI 是最终操作终端；界面不拥有 TF，也不直接控制驱动。当前 RViz 原生 Nav2 Goal 属于底层调试通道，不能替代 Capability/Health 验收。

## 5. 新电脑部署流程

### 5.1 发布准备：旧电脑需要交付什么

1. 各独立仓库提交并推送，记录准确 SHA；保持导航 `main` 与改造分支分开，验收前不合并。
2. 生成覆盖导航、建图、HMI、Mission、硬件及第三方的完整精确版本 `.repos` 清单，并校验导入后的 SHA。当前 `dependencies/navigation_v4_external.repos` **只覆盖其中三个仓库**，不能当成完整发行包。
3. 分发正式地图包与 registry，核验 hash、机器人配置及外参版本；修正旧机器绝对路径。源码提交不携带 bag、地图、构建产物。
4. 记录 Ubuntu/ROS 版本、原生 SDK 版本与构建步骤、设备网络/CAN/权限配置、验收结果和回退版本。

完整清单与异机验收尚未完成。以下是部署顺序，不能据此标记新机已验证。

### 5.2 新电脑安装与构建

1. 准备 Ubuntu 22.04 + ROS 2 Humble，克隆 `agt_navigation_v3` 并检出交付记录中的 SHA。
2. 按完整发布清单导入到 `<工作空间>/src`，检查重复包与源码 Git 归属。
3. 复用 `scripts/bootstrap_humble.sh --smoke` 安装既有依赖并构建；运行前核对其导入清单与发布清单一致。它不代替尚未补齐的全工作空间锁定。
4. 按建图/HMI 等仓库的现有构建说明补齐依赖并执行干净构建，检查 SDK 头文件、链接库和运行库一致。不要复制旧机 `build/`、`install/`。
5. 复制 `maps/` 数据，设置雷达网络、CAN 与权限，核对选定 Profile 和传感器外参。
6. 所有终端加载同一工作空间环境，使用相同 ROS_DOMAIN_ID、RMW 和发现配置；这些值以本次运行脚本/部署配置为准。

现有检查入口：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/agt_navigation_v3/scripts/check_v4_source_ownership.py --source-root src
ros2 pkg prefix agt_robot_bringup
ros2 pkg prefix agt_mission_bringup
src/agt_navigation_v3/scripts/run_field_stack.sh \
  --mode navigation --map auto --robot bunker_v1 --dry-run
```

随后完成无硬件 Action 测试、bag 回放和受控实机验收。迁移细节见 [跨电脑迁移说明](../v4_migration/portable_workspace.md)。

## 6. 日常实际启动顺序

### 6.1 地图与配置预检

先运行上一节 `--dry-run`，确认解析的地图版本、PCD、栅格和重定位库一致。`auto` 读取 registry 的选择策略，不按文件修改时间找“最新”；稳定部署建议固定已实机验收的版本，或在配置好并验证 `active` 指针后使用它。

### 6.2 后台与 RViz

在现场前置检查通过、车辆静止且可人工接管时：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
src/agt_navigation_v3/scripts/run_field_stack.sh \
  --mode navigation --map auto --robot bunker_v1 --rviz
```

脚本依次启动硬件、FAST-LIO2、全局定位、Nav2、运行状态与调试窗口。等待脚本预检通过、Health 连续 `NAV_READY`，并在 RViz 核对地图与机器人位姿，才执行路线。

自动初始化失败时，可显式使用已有 `--localization-mode auto_then_manual`。人工种子仍需 GICP 和定位管理器校验，不直接作为可信定位。

### 6.3 上位界面与任务

后台就绪后，另开加载相同 ROS 环境的终端：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run agt_robot_hmi agt_robot_hmi
```

HMI 单点导航调用 `/navigation/navigate_to`，任务调用 `/mission/execute_route`。**纯 navigation 模式不启动 Mission Runtime。** 当前 inspection 模式仍含旧巡检兼容链；最终完整系统应复用 Mission Bringup 组合任务与导航，并通过 A5–A7 验收后给出唯一受支持的完整系统命令。不要在已经运行导航时再启动一个同时包含导航的 Mission launch。

目前优先通过 RViz 完成现场调试；HMI 启动成功不等于任务闭环已经通过验收。

### 6.4 停止与问题留证

先取消任务并确认静止，再在后台终端按一次 Ctrl+C，等待有序退出。使用现有 `scripts/record_navigation_run.sh` 保存诊断数据；日志应关联地图版本、Profile、源码版本、Health 原因、LIO、TF、命令速度与任务事件。轮速可以作为辅助记录。

更详细的现有命令和注意事项见 [导航启动文档](../../导航启动文档.md)。

## 7. 七种模式：现有基础与待补齐项

| 目标模式 | 复用基础 | 本轮剩余工作 |
| --- | --- | --- |
| 完整系统 | 系统 Bringup + 外部 Mission Bringup | 收敛任务组合入口，去除对旧兼容链的正式依赖，验收后发布命令 |
| 仅硬件 | `agt_system_bringup/hardware.launch.py` → Robot Bringup | 统一 Profile 与硬件预检；禁止重复 owner |
| 建图 | Mapping 的 `run_mid360_live_mapping.sh` | 共享硬件归属及地图发布流程验证；离线加工仍留在 Mapping |
| 仅定位 | 现有 `localization.launch.py` | 明确硬件前置条件、就绪/恢复与退出语义 |
| 导航 | `run_field_stack.sh --mode navigation` | 启动、Health、定位恢复、路线稳定性验收 |
| 调试/诊断 | `--rviz`、离线画线 launch、现有诊断/录包工具 | 统一配置和运行证据，不另建日志系统 |
| 回放/离线 | Mapping 回放脚本、navigation benchmark | 验证时钟、TF、Livox 类型与版本，隔离实车命令输出 |

**当前 run_field_stack.sh 的 --mode 只有 navigation 和 inspection。** 本表不是已存在七个 CLI 参数的声明。后续扩展现有编排，模式间共享 launch；回放不得与实车驱动共同消费运动命令。

## 8. 实施顺序与完成判据

| 阶段 | 工作 | 完成证据 |
| --- | --- | --- |
| 1 审计 | 仓库/包/入口/状态/TF/参数归属及重复工具清单 | 对照实际源码；已实现、未验收、缺失分别列出 |
| 2 运行收敛 | 启动发现、就绪判据、Health 误判、路线偏斜/摆动排查 | 同场景日志与 bag；解释每次中断及控制响应 |
| 3 定位恢复 | 单一 TF、退化恢复、低频校正 | 回放状态转换、拒绝坏匹配、受控实机恢复证据 |
| 4 任务与界面 | 连续画线、任务点、暂停/取消、RViz/HMI 同一接口 | Action 集成及任务现场记录，无界面假成功 |
| 5 部署与模式 | 完整 SHA 清单、地图发布、七种模式及异机安装 | 新机干净构建、参数/launch/重复 owner 检查、回放与现场记录 |

沿用 agt_navigation_benchmark 和 [V4 验收矩阵](../acceptance/V4_ACCEPTANCE_MATRIX.md)，补齐上述场景的映射；测试失败修复后再推进。至少覆盖：构建、参数加载、机器人配置、launch、节点与发布者唯一性、TF、定位状态转换、导航启动、回放，以及任务拒绝/暂停/取消。

最终交付应让操作者能够：**选机器人与地图 → 启动后端 → 查看就绪原因 → RViz 调试或 HMI 操作 → 稳定执行路线/任务 → 异常可解释且可恢复 → 换电脑可复现。** 未运行的门记为 NOT_RUN，不能用编译成功替代实机验收。


## 9. 双整机配置：Bunker 巡检与 YHS 采摘

### 9.1 用户确认的目标与现状

同一工作空间支持两套整机组合，通过启动时加载的 YAML 选择，切换前停止旧栈，不做运行中热切换：

| 整机组合 | 底盘 | 配套设备 | 任务 |
| --- | --- | --- | --- |
| Bunker 巡检 | Bunker | 云台相机、RTK、导航雷达/IMU | 到点停车、采集视角、记录 RTK 元数据、下一点 |
| YHS 采摘 | YHS | 机械臂、采摘视觉、导航传感器（型号待核实） | 到点停车、执行采摘、确认机械臂可行驶、下一点 |

2026-09-24 源码核对：

- `agt_robot_platform/agt_robot_bringup/launch/robot_hardware.launch.py` 目前只允许 `bunker_v1`，驱动组合仍由显式 launch 参数控制，尚未实现任意整机 YAML 切换。
- `agt_robot_description/config/robot_profiles/bunker_v1.yaml` 已有模型、传感器、能力字段，但 GNSS 字段与硬件 launch 的默认启用值不一致；接入统一配置时必须消除双重默认。
- 已有 `src/agt_arm/agt_arm_interfaces/action/PickKiwi.action`、`agt_arm_capability`、fake/real 配置；Mission 已有 `arm.pick_kiwi` handler、`example_route_kiwi.yaml` 和机械臂启动选项。继续复用这些代码，不另写第二个采摘任务引擎。
- 旧 `机械臂/slam_ws` 中存在 YHS/ROS 1 相关代码与产物；不能据此认定已安装可用的 ROS 2 YHS 驱动。需要核对源码、协议、型号、许可证及适配方式。
- 机械臂 real 配置仍引用本机绝对路径和 Python/SDK 环境；异机部署前须整理依赖与源码归属。已有任务调用不等于停车互锁、机械臂安全收回和实机闭环已通过验收。

### 9.2 单独存放整机组合配置

建议扩展现有 Robot Bringup，在其内部按整机分目录；几何/标定继续由 Robot Description 管理，任务路线继续由 Mission 管理：

```text
src/
├── agt_robot_platform/
│   └── agt_robot_bringup/
│       ├── config/robots/                     # 新增目标目录
│       │   ├── bunker_inspection/
│       │   │   ├── robot.yaml                 # Bunker + 云台相机 + RTK
│       │   │   └── devices.yaml               # 端口、地址等部署配置
│       │   └── yhs_harvesting/
│       │       ├── robot.yaml                 # YHS + 机械臂 + 视觉
│       │       └── devices.yaml
│       └── launch/                            # 共享加载器，按配置选择驱动适配器
├── agt_robot_description/config/robot_profiles/
│   ├── bunker_v1.yaml
│   └── yhs_v1.yaml                            # 待实测型号/几何/外参后补齐
├── agt_arm/                                   # 复用现有机械臂能力
│   ├── agt_arm_interfaces/
│   └── agt_arm_capability/
└── agt_mission/
    └── agt_mission_runtime/
        ├── example_route.yaml
        └── example_route_kiwi.yaml             # 已有到点采摘示例
```

这是一份目录与配置设计，当前尚未创建上述整机配置目录。YHS 驱动查明来源后纳入独立外部依赖或 Platform 内适配包，并固定版本；不把旧 ROS 1 工作空间和 build/devel 直接复制到 ROS 2 src。

整机 YAML 的建议字段如下，**不是当前已支持的参数格式**：

```yaml
schema_version: 1
robot_id: bunker_inspection
robot_profile: bunker_v1
base_adapter: bunker
sensors:
  navigation_lidar: mid360
  navigation_imu: mid360_internal
  rtk: {enabled: true, role: metadata_only}
payloads:
  camera_gimbal: {enabled: true}
  arm: {enabled: false}
devices_file: devices.yaml
```

YHS 配置选择 YHS adapter、其真实 robot_profile 和 arm payload；导航雷达、运动学、外参、footprint 不复制 Bunker 数值。加载器先校验 schema、必填设备、依赖与地图兼容性，再启动；缺少驱动时明确失败，禁止回退成 Bunker。传感器启动只启用设备与服务，不能自动执行采摘。

最终期望在原脚本增加 `--robot-config <robot.yaml>`，统一向硬件、定位、Nav2 和任务传递解析后的配置；这是待实现入口，当前不要执行该参数。现有 `--robot bunker_v1` 保留兼容，若两者同时指定且冲突应拒绝。切换机器人同时切换模型、运动限制、地图兼容配置和设备组合，不能只切 CAN 驱动。

### 9.3 独立任务层：到点后调用设备能力

继续以 `agt_mission` 为独立任务层，两套机器人共享以下流程：

```text
加载整机配置 → 检查任务所需能力 → 导航就绪
→ 导航到站点 → LIO 新鲜且连续满足静止条件 → 保持底盘停止
→ 调用 camera.acquire_view 或 arm.pick_kiwi
→ 等待明确结果 → 确认设备处于允许行驶状态 → 下一站点
```

站点坐标、朝向、执行动作、超时及失败策略放在任务 YAML；机械臂轨迹、识别和夹爪控制保留在 agt_arm/原采摘实现中。导航只负责到点与路线跟随，不包含猕猴桃业务逻辑。

机械臂工作期间任务层持有底盘运动互锁，新导航请求也应被拒绝；仅“不发送下一点”不足以阻止其他客户端控制底盘。复用现有 Capability/命令保护链实现统一仲裁，不另建并行速度发布者。

`PickKiwi` 成功、进程退出或收到取消，不自动等价于机械臂已经收回。必须验证明确的可行驶状态；失败、超时、取消或状态不明时保持底盘停止。现有示例中的 `on_failure: skip` 只能在确认机械臂可行驶后执行。导航到点静止判据继续使用 LIO，轮速不进入导航决策；机械臂状态是作业互锁，与轮速健康检查无关。

### 9.4 接入顺序与验收

1. 整理并验证现有 Bunker 配置，将云台相机/RTK 绑定到整机 YAML，保持已验证参数与设备启动行为。
2. 核实 YHS 型号、驱动来源、CAN 协议、运动学、导航传感器和安装外参；补齐 driver/adapter 与 Robot Profile。
3. 复用 agt_arm 与 Mission，先用 fake 模式验证站点顺序、任务结果、超时/取消和底盘互锁；fake 不代表机械臂实机安全确认。
4. 受控验证 YHS 单独行驶、机械臂原地作业，再验证两站点完整流程；测试作业期间其他导航请求、设备掉线和取消场景。
5. 两套整机分别保留可复现配置、版本清单与验收记录，加入部署说明；Bunker 回归通过后再将配置加载器作为默认入口。

### 9.5 2026-09-24 实施状态（对照源码）
- 已新增 `agt_robot_bringup/config/robots/`：`bunker_inspection`（supported，母板）与 `yhs_harvesting`（blocked，预留）。
- `robot_hardware.launch.py` 支持 `robot_config:=`；`run_field_stack.sh` 支持 `--robot-config`，`--robot bunker_v1` 保持兼容，冲突与阻塞均在启动任何进程前拒绝。
- 机械臂互锁与可行驶状态仍为预留契约（见 agt_robot_bringup/README.md），未实现。
- 证据与剩余缺口见 [IMPLEMENTATION_HANDOFF](IMPLEMENTATION_HANDOFF.md)。上文 9.2 中"尚未创建/待实现入口"的描述以本节为准。
