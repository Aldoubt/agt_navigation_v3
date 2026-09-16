# AGT Navigation V3

AGT Navigation V3 是面向 Bunker 类履带底盘与 Livox MID360 的 ROS 2 Humble
导航栈：连续 Batch-LIO 局部里程计、3D 全局重定位、单一全局校正 TF，以及 Nav2
导航。FAST-LIO2、PGO、地图保存和重定位资产生成属于独立的
`agt-lio-pgo-mapping` producer，不在本仓库启动。

当前里程碑：**v0.3.0 定位合同冻结**。下一阶段为 P3 运行时验收、Nav2 现场验证与
操作员工作流。

> 本 README 是项目入口，不替代算法、TF 或验收标准。开始开发或现场运行前，请先阅读
> [CODEX 上下文](docs/CODEX_CONTEXT.md)、[文档权威矩阵](docs/AUTHORITY_MATRIX.md)
> 和 [P3 运行时验收审计](docs/acceptance/P3_RUNTIME_ACCEPTANCE_AUDIT.md)。

## 冻结架构

```text
MID360 / IMU
  -> Batch-LIO -> agt_batch_lio_adapter -> /agt/odometry/local, odom -> base_link
  -> mapping_body 全局重定位 -> /agt/relocalization/pose
  -> agt_localization_manager -> map -> odom
  -> Nav2 -> velocity smoother -> cmd_vel_guard -> 外部 Bunker 驱动
```

- `agt_localization_manager` 是运行时唯一允许发布 `map -> odom` 的组件。
- 正式 PGO 位姿和定位点云使用 `T_map_body` / `mapping_body`；发布边界只转换一次：
  `T_map_base_link = T_map_body * T_body_base_link`。
- Nav2 使用外部定位，刻意不启动 AMCL。
- 原始 Livox 时序直接进入 LIO；PointCloud2 转换是独立的重定位/障碍物支路。

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
ros2 launch agt_system_bringup sensor_session.launch.py
```

在目标机执行构建与选定软件 smoke 检查：

```bash
cd ~/ros2_ws
bash src/agt_navigation_v3/scripts/field_build_smoke.sh
```

依赖和 bootstrap 说明见
[docs/BOOTSTRAP_AND_ROSBAG_GATE.md](docs/BOOTSTRAP_AND_ROSBAG_GATE.md) 与
[docs/MIGRATION.md](docs/MIGRATION.md)。

### 离线启动导航与重定位

离线验证不要使用现场入口 `hmi_field_demo.launch.py`：现场入口假定 MID360、URDF、底盘
和外部硬件已经单独启动。离线入口会自己启动 robot description、Batch-LIO、PointCloud2
桥、全局重定位和 Nav2，再只回放 bag 中的原始 LiDAR/IMU：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agt_system_bringup acceptance_offline_replay.launch.py \
  bag:=/home/yangxuan/ros2_ws/experiments/data/rosbag/bunker_mid360_mapping_20260901_205036 \
  navigation_map:=/home/yangxuan/ros2_ws/maps/bunker_mid360_mapping_20260901_205036/v003-indexed/navigation/map.yaml \
  localization_map:=/home/yangxuan/ros2_ws/maps/bunker_mid360_mapping_20260901_205036/v003-indexed/localization/global_map.pcd \
  relocalization_assets:=/home/yangxuan/ros2_ws/maps/bunker_mid360_mapping_20260901_205036/v003-indexed/localization/relocalization \
  relocalization_executable:=global_relocalization \
  auto_relocalize:=true \
  launch_rviz:=true
```

这里的 bag、`global_map.pcd`、Polar/BBS 资产和 `map.yaml/map.pgm` 必须来自同一张地图
的同一版本。启动成功的判据不是 Nav2 进程出现，而是：

1. Map Server 成功加载 `map.yaml`。
2. Batch-LIO 输出 `/agt/odometry/local`。
3. 重定位进入 `LOCALIZED`，不再持续输出 `REJECTED`。
4. `tf2_echo map base_link` 能看到稳定的 `map -> odom -> base_link`。
5. Nav2 costmap 不再持续报等待 `map/base_link` TF。

当前工作区的 `bunker_mid360_mapping_20260901_211105` 只有旧式 PGO/重定位散件，尚未
导出为当前正式 Map Package；不能直接与 `205036/v003-indexed` 交叉拼接。直接把
`211105` bag 和 `205036` 地图包混用时，我实际看到的结果是：Nav2 地图可以加载，
但 BBS/GICP 返回 `small_gicp did not converge`，没有发布 `map -> odom`，所以 Nav2
一直等待 TF。这至少说明问题不在 Nav2 的 PGM/YAML 加载；在 211105 的正式 Map Package
尚未生成前，不能把这次结果当成完整导航验收，也不能把旧式 loose assets 和正式包混用。

要验证 211105，先在 mapping producer 中用对应的 `20260901_211105-pgo-v4` 导出完整
Map Package，再把它的导航地图、最终 PGO PCD 和重定位资产作为一组传入本入口。

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

# 2. 创建新的不可变 Map Package；不要覆盖已有版本
ros2 run agt_map_manager create_map_package \
  --map-root /home/yangxuan/ros2_ws/maps \
  --map-id site_name \
  --map-version v001 \
  --source-pcd /path/to/final/global_map.pcd \
  --navigation-dir /path/to/pgo-consistent-navigation \
  --relocalization-assets-dir /tmp/site_v1_relocalization

# 3. 查看有效包；这里能列出的才是可选地图
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

最后，必须先**单独**启动并验证 MID360 驱动、Bunker 驱动和 URDF/static TF。AGT
导航 launch 不负责启动这些硬件驱动：

```bash
ros2 launch agt_system_bringup sensor_session.launch.py
```

传感器运行后，另一个终端使用选定地图启动 RViz 导航：

```bash
ros2 launch agt_system_bringup rviz_field_demo.launch.py \
  map:=$AGT_MAP_ROOT/site_name/v001/navigation/map.yaml \
  global_map:=$AGT_MAP_ROOT/site_name/v001/localization/global_map.pcd \
  relocalization_assets:=$AGT_MAP_ROOT/site_name/v001/localization/relocalization \
  map_id:=site_name_v001
```

当前默认不启动 `hmi_field_demo.launch.py`。HMI 仅作为后续可选入口；它不参与建图、
地图确认或首次导航验收。`rviz_field_demo.launch.py` 只启动导航和定位消费者，绝不
启动 FAST-LIO2、PGO、mapping session 或 OctoMap。

```text
Map Package
  ├─ navigation/map.yaml + map.pgm       -> Nav2 map_server
  ├─ localization/global_map.pcd         -> 全局重定位 / map tracker
  └─ localization/relocalization/        -> Polar Context + BBS 候选资产
```

地图包目录、文件约束和 frame contract 见
[docs/architecture/V3_NODE_GRAPH_AND_MAP_CONTRACT.md](docs/architecture/V3_NODE_GRAPH_AND_MAP_CONTRACT.md)。

显式三路径只用于当前 RViz 现场入口；三个路径必须来自同一个已校验 Map Package。

```bash
ros2 launch agt_system_bringup rviz_field_demo.launch.py \
  map:=/data/site/navigation/map.yaml \
  global_map:=/data/site/localization/global_map.pcd \
  relocalization_assets:=/data/site/localization/relocalization \
  map_id:=site_v1
```
导航和定位资产必须来自同一个已批准 Map Package。现场流程、preflight 与首条路线
顺序见 [docs/RVIZ_FIELD_ACCEPTANCE.md](docs/RVIZ_FIELD_ACCEPTANCE.md)。

### 运行确定性离线回放

回放是软件 gate：不启动 CAN、底盘、相机或现场命令链，因此不能证明闭环运动。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agt_system_bringup acceptance_offline_replay.launch.py \
  bag:=/path/to/rosbag \
  navigation_map:=/path/to/navigation/map.yaml \
  localization_map:=/path/to/localization/global_map.pcd \
  relocalization_assets:=/path/to/localization/relocalization \
  relocalization_executable:=global_relocalization \
  auto_relocalize:=true \
  global_query_frame_mode:=mapping_body \
  bbs_query_frame_mode:=mapping_body \
  enable_replay_audit:=true \
  report_dir:=/path/to/evidence
```

有序回放 gate 见 [docs/acceptance/PRE_ACCEPTANCE_GATE.md](docs/acceptance/PRE_ACCEPTANCE_GATE.md)，
仍需补齐的证据见 [P3 审计](docs/acceptance/P3_RUNTIME_ACCEPTANCE_AUDIT.md)。

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

# 定位已接受后执行现场 preflight。
ros2 run agt_navigation_runtime demo_preflight
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
- `Batch-LIO` 输出 `/agt/odometry/local`，形成 `odom -> base_link` 连续局部运动。
- PointCloud2 bridge 输出 `/agt/livox/points`，只供障碍物、全局重定位和可选 tracker 使用。
- Map Package 同时提供 Nav2 `map.yaml`、最终 PGO `global_map.pcd` 和 Polar/BBS 资产。
- 全局定位成功后，只有 `agt_localization_manager` 发布 `map -> odom`。

## 能力状态

### 当前能力

- 通过 `agt_batch_lio_adapter` 输出 Batch-LIO 局部里程计。
- 正式 `mapping_body` 全局重定位：Polar Context 候选检索、候选局部 3D-BBS 与
  `small_gicp` 精配准。
- `LocalizationManager` 全局锚点交接，以及唯一的 `map -> odom` ownership。
- Nav2 地图加载、生命周期、SmacPlanner2D、Regulated Pure Pursuit、速度平滑与
  fail-closed 命令 guard 集成。
- 已存在冻结 mapping-body contract 的确定性回放证据。

现有证据属于软件/回放证据，**不**表示项目已经现场就绪：v003 地图仍为 `REVIEW`，
P3 现场 gate 仍未完成。

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
