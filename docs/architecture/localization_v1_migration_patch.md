# AGT Localization v1 兼容迁移 Patch 设计

## 依据、目标与不可变项

本设计以 [localization_architecture.md](localization_architecture.md) 定义的目标架构为唯一依据：

```text
Map Package -> Map Runtime Server
                      |
      Global Relocalization + Local Tracker
                      |
            Localization Manager
                      |
                 map -> odom

            FAST-LIO2 / Batch-LIO
                      |
               odom -> base_link
```

迁移目标是把当前散布在 `agt_navigation_v3/navigation/localization` 的能力组织为 `agt_localization_v1`，而不是替换定位算法。第一阶段只增加 interface、core、runtime facade 与 launch compatibility layer。

以下 baseline 一律冻结：

- `agt_localization_manager` 是当前唯一 `map -> odom` owner；
- Batch-LIO 或 FAST-LIO adapter 是当前唯一 `odom -> base_link` owner；
- 3D-BBS、Polar Context、small_gicp、`map_gicp_tracker` 的算法、参数及 Map Package 格式不变；
- `T_map_odom = T_map_base * inverse(T_odom_base)` 不变；
- Nav2、controller、planner、TF frame 名称不变；
- 已有 `/agt/relocalization/*`、`/agt/map_tracking/*`、`/agt/localization/*` 接口继续可用。

## 当前实现到目标架构的映射

| 目标架构模块 | 当前实现 | v1 初期动作 |
| --- | --- | --- |
| Global Relocalization | `agt_global_relocalization` + `agt_global_relocalization_native` | 保持算法与 executable；通过 facade 纳入统一 launch |
| Local Tracker | `agt_map_tracker` + native `map_gicp_tracker` | 保持算法；新增 `agt_local_tracker` 只做 topic/参数 compatibility |
| Localization Manager | `agt_localization_manager` | 先抽取纯状态/校正逻辑；旧 manager 保留为 legacy owner |
| Map Runtime Server | launch 中的绝对 `global_map` / `relocalization_assets` 参数 | 新增只读 `agt_map_runtime`，先解析 Map Package，再逐步取代路径散落 |
| Keyframe Manager | 当前没有正式 ROS keyframe contract | 在 ROS layer 增加 adapter，默认关闭；不改 LIO 前端 |
| LIO odometry | Batch-LIO/FAST-LIO adapter | 不移动、不重命名、不改变 TF |

## 1. 新增 package 列表

新 package 放在 `navigation/localization/`。`agt_localization_v1` 是这组 package 的 release/profile 名，不创建第二套同名算法实现。

| 新 package | 责任 | 初版行为 | 不负责 |
| --- | --- | --- | --- |
| `agt_localization_interfaces` | Keyframe、Map metadata、local-map service 与 v1 状态的 typed ROS 接口 | 只生成 rosidl interface | 不修改现有 JSON/status topic |
| `agt_localization_core` | 无 ROS 依赖的状态机、odom 匹配、correction gate、SE(3) math | 与 legacy manager 做 shadow 相同计算 | 不发布 TF、不调用 BBS/GICP |
| `agt_localization_ros` | manager node、legacy bridge、keyframe adapter、v1 compat launch | `shadow` 模式只输出诊断 | 不拥有 LIO odom |
| `agt_map_runtime` | Map Package manifest 校验、asset path resolver、`GetLocalMap` server skeleton | 首期只校验并返回 metadata/path；local cloud 查询暂返回明确 unsupported | 不生成或修改 PCD/BBS/Polar Context |
| `agt_local_tracker` | local tracker facade/plugin boundary | 首期转换 legacy `agt_map_tracker` observation/status | 不复制或改写 native GICP |

`agt_global_relocalization` 已经是目标架构中 Global Relocalization 的 package 名，因此不创建平行的 `agt_global_relocalization_v1`。第一阶段只由 `agt_localization_ros` 的 launch facade 包装它；后续若内部整理，也在原 package 内以兼容方式演进。

## 2. 文件迁移列表

### 2.1 Patch 1：冻结与抽取，不改变运行图

| 源文件 | 新文件 | 迁移动作 | legacy 文件 |
| --- | --- | --- | --- |
| `agt_localization_manager/.../localization_manager.py` 中状态、插值、innovation、`T_map_odom` 纯逻辑 | `agt_localization_core/.../{state_machine,correction_math,odom_buffer}.py` | 复制/抽取为纯函数，逐例比较结果 | 保留且继续运行 |
| `agt_localization_manager/test/test_correction_math.py` | `agt_localization_core/test/test_legacy_parity.py` | 添加 legacy-v1 parity case | 原测试保留 |
| `agt_robot_interfaces/msg/LocalizationMetrics.msg` | 不迁移 | `agt_localization_interfaces` 依赖并复用 | 保持原 package/type |
| `agt_global_relocalization/config/global_relocalization.yaml` | 不迁移 | v1 launch 通过参数 overlay include | 仍是 BBS/GICP 唯一默认参数 |
| `agt_map_tracker/config/map_tracker.yaml` | 不迁移 | `agt_local_tracker` 显式透传对应参数 | 保持原文件与默认值 |

### 2.2 Patch 2：Map Runtime 与 interface 接入

| 源位置 | 新位置 | 动作 | 兼容行为 |
| --- | --- | --- | --- |
| `rviz_field_demo.launch.py` 的 `map`、`global_map`、`relocalization_assets` 参数 | `agt_map_runtime/asset_resolver.py` | 校验 manifest、sha256、required files 并解析绝对路径 | 仍接受旧三路径参数；两种输入互斥 |
| `agt_global_relocalization` 的 direct path 参数 | `agt_localization_ros/global_relocalization_bridge.py` | 从 resolver 注入完全相同的 path | backend command 与 asset 格式不变 |
| `agt_map_tracker` 的 `global_map` 参数 | `agt_local_tracker/legacy_tracker_bridge.py` | 从 resolver 注入相同 path | native tracker 不变 |
| LIO `/agt/odometry/local` + cloud topic | `agt_localization_ros/keyframe_adapter.py` | 距离/转角/时间门限生成可选 keyframe | 原 LIO 输入和输出不变 |

### 2.3 Patch 3：manager 切换（仅 shadow 验收后）

| 旧文件 | v1 目标 | 切换条件 | 旧代码处置 |
| --- | --- | --- | --- |
| `agt_localization_manager/localization_manager.py` | `agt_localization_ros/localization_manager_node.py` | shadow parity 和现场验收通过 | 保留，作为 `legacy` profile |
| `agt_map_tracker/map_tracker.py` | `agt_local_tracker/local_tracker_node.py` 或 facade | 只在 Keyframe contract 稳定后 | 保留 native path 与旧 launch |
| `agt_global_relocalization/*.py` | 原 package 内部按需整理 | 不早于 Map Runtime 上线 | 不改 BBS/Polar/small_gicp |

禁止在这三个 patch 中删除任何旧 source、launch、YAML、native executable 或 topic alias。

## 3. interface 定义

### 3.1 `Keyframe.msg`

`navigation/localization/agt_localization_interfaces/msg/Keyframe.msg`：

```text
std_msgs/Header header
geometry_msgs/PoseWithCovariance pose
sensor_msgs/PointCloud2 cloud
float64 quality
```

语义：`header.stamp` 是选中 LIO keyframe 的时间；`pose` 为同一时刻的 local odom pose；cloud 使用原始 PointCloud2 frame，不在 message 内重标 frame。初期发布 topic：`/agt/lio/keyframe`。

默认 keyframe 规则实现为可配置但关闭：距离、yaw、时间三者任一门限触发。启用后只为 local tracker 提供低频候选帧，绝不改变 FAST-LIO2/Batch-LIO 输入路径。

### 3.2 `MapMetadata.msg` 与 `GetLocalMap.srv`

`MapMetadata.msg`：

```text
string map_id
string map_version
string package_root
string global_map_path
string relocalization_assets_path
string manifest_sha256
```

`GetLocalMap.srv`：

```text
geometry_msgs/PoseStamped pose
float64 radius
string layer
---
sensor_msgs/PointCloud2 cloud
agt_localization_interfaces/MapMetadata metadata
bool success
string message
```

第一阶段只保证 `MapMetadata` 和 path validation；`layer` 值预留 `geometry`、`dynamic`、`elevation`。当 tiled map 尚未实施时，service 必须返回 `success=false` 和明确说明，不能伪造局部点云。

### 3.3 `LocalizationState.msg`

```text
builtin_interfaces/Time stamp
string state
string source
string reason
bool has_global_correction
bool tracking_enabled
bool recovery_requested
string map_id
string map_version
```

v1 state 采用架构文档的外部语义：`UNINITIALIZED`、`SEARCHING`、`LOCALIZED`、`TRACKING`、`LOST`。它与 legacy 的 `BOOT`、`WAIT_GLOBAL`、`DEGRADED`、`RECOVERY_REQUESTED`、`RELOCALIZING` 通过 bridge 映射，而非改变 legacy topic 的 JSON 值。

### 3.4 既有接口兼容表

| 既有 topic/service | 类型 | v1 处理 |
| --- | --- | --- |
| `/agt/relocalization/request` | `std_msgs/msg/Empty` | 原样转发给 legacy global relocalizer |
| `/agt/relocalization/pose` | `PoseWithCovarianceStamped` | 原样保留；bridge 可转换成内部 observation |
| `/agt/map_tracking/pose` | `PoseWithCovarianceStamped` | 原样保留；local tracker facade 订阅/发布它 |
| `/agt/map_tracking/status` | `std_msgs/msg/String` | 原样保留 JSON payload |
| `/agt/localization/status` | `std_msgs/msg/String` | active v1 必须维持同类型、同字段语义 |
| `/agt/relocalization/status` | `std_msgs/msg/String` | 保留 debug JSON |
| `/agt/localization/metrics` | `agt_robot_interfaces/msg/LocalizationMetrics` | 继续发布；不改 message package |
| `/agt/localization/relocalize` | `std_srvs/srv/Trigger` | 保留服务名与行为 |

绝不允许在同一 ROS graph 内对上述同名 topic 使用不同 message 类型。

## 4. launch 修改计划

### 新增 launch

新增 `agt_localization_ros/launch/localization_v1_compat.launch.py`：

```text
mode:=legacy                  # legacy | shadow | v1
map_package:=                 # <site>/<version> 或绝对 package root
global_map:=                  # legacy fallback
relocalization_assets:=       # legacy fallback
enable_keyframes:=false
enable_local_tracking:=false
use_sim_time:=false
```

### 既有 launch 的最小改动

在 `rviz_field_demo.launch.py`、`navigation_debug.launch.py`、offline replay launch、`fastlio_anchor_nav.launch.py` 中只增加：

```text
localization_stack:=legacy
map_package:=
```

默认值固定为 `legacy`，因此现有现场命令的 node graph、参数、TF 与导航行为不变。

| profile | 启动内容 | `map -> odom` owner | Nav2 是否消费 |
| --- | --- | --- | --- |
| `legacy` | 现有 global relocalizer、legacy manager、可选 legacy tracker | `agt_localization_manager` | 是 |
| `shadow` | 完整 legacy 链 + v1 core/bridge/runtime | `agt_localization_manager` | 是；v1 不写 `/tf` |
| `v1` | global backend、local tracker facade、v1 manager | `agt_localization_ros` | 是；legacy manager 不启动 |

`shadow` 中 v1 的预览 transform 发布到 `/agt/localization/v1/shadow/map_odom`（普通 `TransformStamped`），不能通过 TF broadcaster 发布。启动文件必须使用互斥条件保证 `legacy` 与 `v1` 不会同时发布 `map -> odom`。

FAST-LIO anchor profile 继续使用其既有的“global anchor 后不启用 tracking recovery”策略；第一阶段只能选 `legacy` 或 `shadow`，不能借此迁移改变锚定逻辑。

## 5. 参数文件设计

```text
navigation/localization/
├── agt_localization_core/config/localization_policy.yaml
├── agt_localization_ros/config/localization_v1.yaml
├── agt_localization_ros/config/keyframe_adapter.yaml
├── agt_map_runtime/config/map_runtime.yaml
└── agt_local_tracker/config/local_tracker_compat.yaml
```

### `localization_policy.yaml`

只放与 ROS graph 无关的 legacy-equivalent policy；所有初值必须从 `agt_localization_manager/config/localization_manager.yaml` 复制：

```yaml
state_machine:
  local_odom_timeout_sec: 0.30
  local_odom_lost_sec: 1.00
  global_match_max_skew_sec: 0.10
  recovery_cooldown_sec: 5.0
tracking_gate:
  max_translation_innovation_m: 0.50
  max_yaw_innovation_deg: 5.0
  tracking_consecutive_accepts: 2
correction:
  smoothing_enabled: true
  tau_sec: 3.0
  max_linear_rate_mps: 0.10
  max_yaw_rate_degps: 2.0
```

### `localization_v1.yaml`

只放 frame/topic/owner 绑定：

```yaml
agt_localization_ros:
  ros__parameters:
    tf_owner: shadow
    map_frame: map
    odom_frame: odom
    base_frame: base_link
    local_odom_topic: /agt/odometry/local
    global_pose_topic: /agt/relocalization/pose
    tracking_pose_topic: /agt/map_tracking/pose
    tracking_status_topic: /agt/map_tracking/status
    shadow_transform_topic: /agt/localization/v1/shadow/map_odom
    state_topic: /agt/localization/v1/state
    metrics_topic: /agt/localization/v1/shadow/metrics
    tf_publish_rate_hz: 30.0
```

### `map_runtime.yaml`

```yaml
agt_map_runtime:
  ros__parameters:
    map_root_env: AGT_MAP_ROOT
    require_manifest: true
    verify_sha256: true
    required_files:
      - navigation/map.yaml
      - localization/global_map.pcd
      - localization/relocalization
```

任何 BBS、GICP、Polar Context 参数仍只保留在现有 `global_relocalization.yaml` 与 backend command；不能在 v1 YAML 再复制一份。

## 6. 测试方案

### Unit / interface

- 对 global pose、tracking pose、stale local odom、recovery cooldown 与 correction smoothing，v1 core 结果逐项等于 legacy manager；
- `Keyframe.msg` 的 stamp/frame/covariance/cloud 保真；阈值未满足不发布；
- Map Runtime 覆盖有效 package、缺 manifest、sha256 失配、缺 `map.yaml`、缺 `global_map.pcd`、缺 relocalization assets；
- bridge 覆盖 `PoseWithCovarianceStamped`、JSON status、`LocalizationMetrics` 的 legacy type 兼容。

### Launch / TF ownership

- `localization_stack:=legacy`：只存在 legacy `map -> odom` publisher；
- `shadow`：`/tf` 的 `map -> odom` 仍只来自 legacy manager；v1 preview 不出现在 `/tf`；
- `v1`：legacy manager 不启动；`map -> odom` 仅 v1 manager 发布；
- 三种模式：LIO adapter 仍是唯一 `odom -> base_link` owner；
- `/agt/relocalization/pose` 在 RViz、publisher、subscriber 中始终为同一 `PoseWithCovarianceStamped` 类型。

### Offline differential replay

冻结同一 Map Package、`bunker_mid360_mapping_20260901_211105` bag 和 launch 参数：

1. 跑 `legacy`，记录 global/tracking observation、local odom、`/tf`、status、metrics；
2. 跑 `shadow`，按同一 odom timestamp 比较 shadow `T_map_odom` 与 legacy TF 的 translation/yaw、状态转移、recovery request；
3. 若超出定义的浮点/时间容差，不允许启用 `v1`；
4. `v1` 仅在 shadow parity 通过后测试冷启动、人工重定位、LOST recovery 与 Nav2 lifecycle；
5. 记录 TF gap、extrapolation error、relocalization success 与 map package version，禁止出现新增回归。

### 现场验收

验收场景：静止冷启动、断电恢复、30 分钟连续导航、动态障碍环境。任一出现 TF 双发布、`odom -> base_link` blackout、Nav2 crash、错误 Map Package 或错误 global correction，立即用 `localization_stack:=legacy` 回退。

## 7. 实施顺序与回退

1. **冻结 baseline**：为现有 launch、topic type、TF owner 与关键参数补 static contract tests。
2. **interfaces + core**：不接入正式 launch，不改变 runtime。
3. **Map Runtime + compatibility launch**：增加 `shadow`，默认依旧 `legacy`。
4. **Keyframe / local tracker facade**：默认关闭，在离线 replay 单独验收。
5. **v1 manager active**：仅在 shadow parity 与车辆 shadow 结果通过后，显式 `localization_stack:=v1`。
6. **Map Runtime 成为默认**：只在 manifest/hash 验证稳定后，将 Map Package 输入设为 field default。

每一步可用 `localization_stack:=legacy` 无损回退。旧 package、旧 launch、旧 YAML 和 native algorithm 的删除必须是另一个 deprecation 任务，且至少经过一个发布周期的 replay、车辆验收和下游 consumer 迁移；本迁移 patch 不包含删除授权。
