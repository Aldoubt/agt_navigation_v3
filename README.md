# AGT Navigation V3

AGT Navigation V3 是面向 Bunker 类履带底盘与 Livox MID360 的 ROS 2 Humble
导航栈：默认 FAST-LIO2 连续局部里程计、3D 全局重定位、单一全局校正 TF，以及 Nav2
导航。Batch-LIO 保留为显式可选后端。导航只运行选中的局部 LIO 与适配器，
PGO、建图保存和地图生产流程属于独立的 `agt_mapping_framework`，不随导航启动。

当前里程碑：**v0.3.0 定位合同冻结**。下一阶段为 P3 运行时验收、Nav2 现场验证与
操作员工作流。

> 本 README 是项目入口，不替代算法、TF 或验收标准。开始开发或现场运行前，请先阅读
> [CODEX 上下文](docs/CODEX_CONTEXT.md)、[文档权威矩阵](docs/AUTHORITY_MATRIX.md)
> 和 [P3 运行时验收审计](docs/acceptance/P3_RUNTIME_ACCEPTANCE_AUDIT.md)。

## 冻结架构

```text
MID360 / IMU
  -> FAST-LIO2（默认）-> agt_fastlio_adapter -> /agt/odometry/local, odom -> base_footprint
     或显式 Batch-LIO -> agt_batch_lio_adapter（互斥）
  -> mapping_body 全局重定位 -> /agt/relocalization/pose
  -> agt_localization_manager -> map -> odom
  -> Nav2 -> velocity smoother -> cmd_vel_guard -> 外部 Bunker 驱动
```

- `agt_localization_manager` 是运行时唯一允许发布 `map -> odom` 的组件。
- 正式 PGO 位姿和定位点云使用 `T_map_body` / `mapping_body`；发布边界只转换一次：
  `T_map_base_link = T_map_body * T_body_base_link`。
- Nav2 使用外部定位，刻意不启动 AMCL。
- 原始 Livox 时序直接进入 LIO；PointCloud2 转换是独立的重定位/障碍物支路。

Navigation 顶层入口为 `hardware.launch.py`、`localization.launch.py`、
`navigation.launch.py` 和 `debug.launch.py`；Mission 入口属于外部
`agt_mission_bringup/mission.launch.py`。现场后台由
`run_field_stack.sh` 编排，RViz 用于调试，HMI 在后台就绪后单独启动；
以 [导航启动文档](导航启动文档.md) 为准。

MID360 + Bunker 实机阶段使用只读验收工具链；TF authority、topic 频率、运动中心、
导航录包和报告流程见
[实机验收流程](docs/HARDWARE_ACCEPTANCE_MID360_BUNKER.md)。

完整 frame contract 以
[docs/contracts/TF_CONVENTION.md](docs/contracts/TF_CONVENTION.md) 与
[docs/contracts/backend_contract_matrix.yaml](docs/contracts/backend_contract_matrix.yaml)
为准。

## 快速开始

### 构建工作空间

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

启动全部传感器并连接底盘，终端会每8s查询一次连接情况，rtk默认可以开启也可以不开启
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch agt_system_bringup hardware.launch.py
```

在目标机执行构建与选定软件 smoke 检查：

```bash
cd ~/ros2_ws
bash src/agt_navigation_v3/scripts/field_build_smoke.sh
```

依赖和 bootstrap 说明见
[docs/BOOTSTRAP_AND_ROSBAG_GATE.md](docs/BOOTSTRAP_AND_ROSBAG_GATE.md) 与
[docs/MIGRATION.md](docs/MIGRATION.md)。

### 默认地图与里程计（2026-09-24）

现场脚本和顶层 localization launch 默认 `fastlio2`。默认选图为 `auto`，读取
`<工作空间>/maps/registry.yaml` 的 `latest_validated`；本机当前是
`bunker_mid360/20260924-trav-integration-v1`，不读取未发布实验候选或旧 active 指针。
地图目录、覆盖规则与验证记录见 [默认配置核查](docs/mcp-navigation-defaults-20260924.md)。
显式回退 Batch-LIO 使用 `--lio-backend batch_lio`，先正常退出旧栈再静止重启和重定位。

### 无硬件配置检查

当前顶层保留分阶段入口和独立 Mission 入口，不再提供旧的
`acceptance_offline_replay.launch.py`、`rviz_field_demo.launch.py` 或
`hmi_field_demo.launch.py`。不连接硬件时，先用同一现场脚本检查模式和地图解析：

```bash
cd /home/yangxuan/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

src/agt_navigation_v3/scripts/run_field_stack.sh --mode navigation --dry-run
src/agt_navigation_v3/scripts/run_field_stack.sh --mode inspection --dry-run
```

这只验证参数、地图路径和两种模式的启用关系，不代表定位、底盘运动或相机实拍通过。
完整仿真使用 `agt_simulation_bringup` 的入口；历史回放审计命令只保留在归档文档中，
不能再当作当前启动命令。

### 现场启动导航（默认不接入 HMI）

现场流程固定为：

```text
建图 producer
  -> PGO 优化并确认最终 global_map.pcd
  -> 生成 navigation/map.yaml + map.pgm
  -> 生成 Polar Context / BBS 重定位资产
  -> 创建并校验 Map Package
  -> 选择 map_id/map_version
  -> RViz 启动导航
```

这里的“已经校验过的 Map Package”不是手工约定，而是通过
`create_map_package` 原子生成并自校验的目录。建图完成后，在 mapping producer
输出最终 PGO PCD 和导航地图目录，再执行：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 1. 从同一份最终 PGO PCD 生成重定位资产
ros2 run agt_global_relocalization_native build_relocalization_assets \
  --map /path/to/final/global_map.pcd \
  --output /tmp/site_v1_relocalization \
  --map-leaf 0.5 --bbs-min-level-res 0.5 --bbs-max-level 5

# 2. 从同一次建图的 poses.txt + patches/ 生成 Polar Context 候选库
ros2 run agt_global_relocalization_native build_relocalization_candidates \
  --map-dir /path/to/mapping_package \
  --output /tmp/site_v1_relocalization

# 3. 若 Map Studio 导出了禁行区，先把它烘焙成 Nav2 占用栅格
ros2 run agt_map_manager bake_keepout_zones \
  --map /path/to/confirmed/map.yaml \
  --zones /path/to/confirmed/keepout_zones.yaml \
  --output /tmp/site_v1_navigation --inflate-cells 1

# 4. 创建新的不可变 Map Package；不要覆盖已有版本
ros2 run agt_map_manager create_map_package \
  --map-root /home/yangxuan/ros2_ws/maps \
  --map-id site_name \
  --map-version v001 \
  --source-pcd /path/to/final/global_map.pcd \
  --navigation-dir /tmp/site_v1_navigation \
  --relocalization-assets-dir /tmp/site_v1_relocalization

# 5. 查看有效包；这里能列出的才是可选地图
export AGT_MAP_ROOT=/home/yangxuan/ros2_ws/maps
ros2 run agt_map_manager list_map_packages
```

确认地图时至少检查：PCD 与 PGM 在 RViz 中重合、`map.yaml` 能加载、重定位资产来自
同一份最终 PGO PCD，并完成一次静止全局重定位。确认通过后，选择版本：

```bash
ros2 run agt_map_manager select_map_package \
  --map-root "$AGT_MAP_ROOT" \
  --map-id site_name \
  --map-version v001

ros2 run agt_map_manager validate_active_map \
  --active-state-file "$AGT_MAP_ROOT/active_map.yaml"
```

当前已确认并选择的活动地图是：

```text
/home/yangxuan/ros2_ws/maps/bunker_mid360/
  20260922_143427-fixed-replay-v1
```

它把 `20260922_143427` 实机包修复回放后的 PGO `global_map.pcd`、Nav2 `map.yaml/map.pgm`、
Polar Context 和 BBS 重定位资源作为一个运行包，二维导航范围约为
`124.5 m × 95.2 m`。不要在启动参数中混用其他版本的二维图或重定位资产。

推荐使用单终端受控入口，并显式选择模式。纯导航模式不启动相机或巡检任务：

```bash
cd /home/yangxuan/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
src/agt_navigation_v3/scripts/run_field_stack.sh --mode navigation --rviz
```

到点拍照保存模式会启动 C1 和巡检任务链：

```bash
src/agt_navigation_v3/scripts/run_field_stack.sh --mode inspection --rviz
```

在 RViz 中排好点后启动任务：

```bash
ros2 service call /agt/rviz_patrol/start std_srvs/srv/Trigger '{}'
```

任务按“导航到点 → 实测停止 → 三视角拍照 → 图片与元数据归档 → 下一点 → 返回起点”
运行，记录保存在 `~/.ros/agt_inspection_records/`。详细的三终端等价命令、验收方法和
两种模式差异见 [导航启动文档](导航启动文档.md)。

```text
Map Package
  ├─ navigation/map.yaml + map.pgm       -> Nav2 map_server
  ├─ localization/global_map.pcd         -> 全局重定位 / map tracker
  └─ localization/relocalization/        -> Polar Context + BBS 候选资产
```

地图包目录、文件约束和 frame contract 见
[docs/architecture/V3_NODE_GRAPH_AND_MAP_CONTRACT.md](docs/architecture/V3_NODE_GRAPH_AND_MAP_CONTRACT.md)。

导航和定位资产必须来自同一个已批准 Map Package。现场流程、preflight 与首条路线
顺序见 [导航启动文档](导航启动文档.md)。

### 离线回放状态

旧的确定性回放 launch 已从当前入口架构移除。归档验收文档中的
`acceptance_offline_replay.launch.py` 命令仅用于追溯历史证据，当前版本不要直接执行。
地图和模式的无硬件检查使用上面的 `--dry-run`；闭环能力仍以实车测试为准。

## 常用运行命令

以下命令均假设工作空间已按上文 source。

```bash
# 建图或导航前，进行静止 MID360/IMU preflight。
ros2 run agt_operator_console operator_console sensors

# 仅在车辆静止时请求手动全局重定位。
ros2 service call /agt/localization/relocalize std_srvs/srv/Trigger "{}"

# 查看规范导航位姿链。
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /agt/localization/status

# 发送目标前检查 Nav2 可用性。
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server

# 定位已接受后执行现场 preflight：纯导航不要求相机。
ros2 run agt_navigation_runtime demo_preflight --ros-args -p require_camera:=false

# 到点拍照模式必须要求相机 action 可用。
ros2 run agt_navigation_runtime demo_preflight --ros-args -p require_camera:=true
```

只有在定位状态为 `LOCALIZED`、全局校正有效、RViz 中
`map -> odom -> base_link` 合理且稳定、并且现场 preflight 通过后，才允许发送
导航目标。

## 节点图与数据接口

当前 v3 的完整运行时拓扑、节点职责、topic/service/TF，以及导航和重定位所需的
地图文件格式，统一见
[docs/architecture/V3_NODE_GRAPH_AND_MAP_CONTRACT.md](docs/architecture/V3_NODE_GRAPH_AND_MAP_CONTRACT.md)。

最关键的启动前检查是：

- 传感器提供 Livox `CustomMsg` 和 `sensor_msgs/Imu`，并已启动真实安装位姿的 URDF/static TF。
- 默认 FAST-LIO2 经 `agt_fastlio_adapter` 输出 `/agt/odometry/local`；消息位姿为 `odom -> base_link`，适配器发布 `odom -> base_footprint` TF。Batch-LIO 仅显式可选。
- PointCloud2 bridge 输出 `/agt/livox/points`，只供障碍物、全局重定位和可选 tracker 使用。
- Map Package 同时提供 Nav2 `map.yaml`、最终 PGO `global_map.pcd` 和 Polar/BBS 资产。
- 全局定位成功后，只有 `agt_localization_manager` 发布 `map -> odom`。

## 能力状态

### 当前能力

- 默认通过 `agt_fastlio_adapter` 输出 FAST-LIO2 局部里程计；可显式选择 Batch-LIO，不能同时启动。
- 正式 `mapping_body` 全局重定位：Polar Context 候选检索、候选局部 3D-BBS 与
  `small_gicp` 精配准。
- `LocalizationManager` 全局锚点交接，以及唯一的 `map -> odom` ownership。
- Nav2 地图加载、生命周期、SmacPlanner2D、Regulated Pure Pursuit、速度平滑与
  fail-closed 命令 guard 集成。
- 已存在冻结 mapping-body contract 的确定性回放证据。

现有证据属于软件/回放证据，**不**表示项目已经现场就绪：
`20260922_143427-fixed-replay-v1` 已通过源 PGO 产物、同源资产和运行包完整性检查，但 P3 实车导航、
停车拍照与返回起点 gate 仍需现场测试。

### 实验性能力

- MapTracker 应用 correction 的现场行为。mapping-body open-loop 证据为正向，
  但 correction authority、失败状态转换和现场阈值仍须经 P3 验证。
- 运行中 `LOST` 后的自动全局恢复；当前验收中必须保持关闭。
- 地形感知导航、动态地图处理、RTK/INS 作为定位输入、HMI 调度，以及断电续巡。

实验必须保留默认行为、产生可审计证据，且不得被表述为现场能力。详见
[docs/CODEX_CONTEXT.md](docs/CODEX_CONTEXT.md)。

### 已弃用能力

- `base_link` 全局重定位 query/candidate mode 仅作为 legacy map package 的兼容模式，
  两个参数都必须显式设置：

  ```text
  relocalization_query_frame_mode:=base_link
  bbs_query_frame_mode:=base_link
  ```

  混用 `base_link`/`mapping_body` 会被拒绝。正式 PGO map package 必须使用默认的
  `mapping_body` contract。

- 历史 acceptance report 和 archive design document 只能作为证据，不能作为当前
  权威。请通过文档权威矩阵定位当前规则。

## 文档导航

| 需求 | 阅读 |
| --- | --- |
| 当前里程碑、约束与阅读顺序 | [docs/CODEX_CONTEXT.md](docs/CODEX_CONTEXT.md) |
| 某个问题由哪份文档裁决 | [docs/AUTHORITY_MATRIX.md](docs/AUTHORITY_MATRIX.md) |
| 运行时进程与依赖 | [docs/architecture/RUNTIME_ARCHITECTURE.md](docs/architecture/RUNTIME_ARCHITECTURE.md) |
| TF 与定位 frame contract | [docs/contracts/TF_CONVENTION.md](docs/contracts/TF_CONVENTION.md) 与 [backend matrix](docs/contracts/backend_contract_matrix.yaml) |
| 全局重定位接口 | [docs/contracts/GLOBAL_RELOCALIZATION_SDK_CONTRACT.md](docs/contracts/GLOBAL_RELOCALIZATION_SDK_CONTRACT.md) |
| 当前实现边界 | [docs/CURRENT_NAVIGATION_CAPABILITIES.md](docs/CURRENT_NAVIGATION_CAPABILITIES.md) |
| P3 运行时证据与缺失 gate | [docs/acceptance/P3_RUNTIME_ACCEPTANCE_AUDIT.md](docs/acceptance/P3_RUNTIME_ACCEPTANCE_AUDIT.md) |
| 有序 pre-field 与现场流程 | [docs/acceptance/PRE_ACCEPTANCE_GATE.md](docs/acceptance/PRE_ACCEPTANCE_GATE.md) 与 [docs/RVIZ_FIELD_ACCEPTANCE.md](docs/RVIZ_FIELD_ACCEPTANCE.md) |

## 验收状态

P3 运行时验收进行中。不得依据 build、回放、Gazebo 或单次重定位成功宣称
`FIELD READY`。退出条件是
[docs/acceptance/PRE_ACCEPTANCE_GATE.md](docs/acceptance/PRE_ACCEPTANCE_GATE.md)
中的有序 gate：解决地图 REVIEW、补齐运行时 TF/LIO/定位证据、冻结 tracker policy、
验证 Nav2/guarded motion，并记录物理现场测量结果。

### 初始化定位保底（2026-09-24）

新增 `--localization-mode auto|auto_then_manual|manual`。默认 `auto` 不变；
`auto_then_manual` 两次自动失败后保持后台等待 RViz 初始位姿，经局部 GICP 和 Manager 校验成功才启动 Nav2。
不发布虚假定位、不同时运行两个重定位节点。使用步骤和测试边界见
[人工定位保底说明](docs/mcp-manual-initialization-fallback.md)。
原四个主阶段入口不变，新增 `initialization_view.launch.py` 仅作人工初始化地图显示辅助。


## RViz 绘线工作台（2026-09-24）

已接入原生连续拖画、航点编辑与 Route Workbench 面板。离线必须使用独立 ROS domain；
`/agt/path_tool/preview` 与 `/start` 已分离，在线需预览校验后确认执行。
操作、编译、测试证据与实车验收边界见 [实现说明](docs/mcp-rviz-workbench-implementation.md)。
