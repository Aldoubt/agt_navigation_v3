# V3 节点图与 Map Package 接口

本文档描述 `agt_navigation_v3` 当前实际运行链路。建图生产端不在本仓库：
FAST-LIO2、PGO、最终 PCD、poses/patches 和重定位资产由
`agt-lio-pgo-mapping` 导出；v3 只校验并消费 Map Package。

## 1. 运行节点图

```text
                         external hardware
   Livox MID360 driver ───────────────┐
   Bunker CAN driver                  │
   robot_state_publisher / URDF       │
                                      │
       CustomMsg + IMU                │ static TF / chassis TF
          │                           │
          ▼                           ▼
   batch_lio (external) ──► agt_batch_lio_adapter
                                  │
                                  └── /agt/odometry/local
                                      odom -> base_link

   CustomMsg ──► agt_livox_tools ──► /agt/livox/points (PointCloud2)
                                      │              │
                                      │              ├── agt_pointcloud_preprocessor
                                      │              │       └── /agt/navigation/points_obstacles -> Nav2 costmap
                                      │              │
                                      │              └── agt_global_relocalization
                                      │                      └── /agt/relocalization/pose
                                      │
   Map Manager ──► /agt/map/status       │
   Map Package ─────────────────────────┘
      │             │                    │
      │             └── global_map.pcd + relocalization assets
      └── navigation/map.yaml ──► Nav2 map_server

   /agt/relocalization/pose ──► agt_localization_manager ──► map -> odom
   /agt/odometry/local ────────────────────────────────────► odom -> base_link

   Nav2 ──► velocity smoother ──► agt_cmd_vel_guard ──► external Bunker driver
     │
     └── /navigate_to_pose <── agt_navigation_runtime / HMI / patrol task
```

### 启动入口与职责

| 入口 | 用途 | 是否启动建图 |
|---|---|---:|
| `agt_system_bringup/rviz_field_demo.launch.py` | 默认现场入口：显式指定三份资产并启动 RViz | 否 |
| `agt_system_bringup/hmi_field_demo.launch.py` | 可选入口：从 `active_map.yaml` 启动现场链和 HMI | 否 |
| `agt_system_bringup/acceptance_field.launch.py` | 手动种子定位的现场验收入口 | 否 |
| `agt_system_bringup/acceptance_offline_replay.launch.py` | rosbag 离线回放验收 | 否 |
| `agt_navigation_runtime/navigation_lio.launch.py` | Batch-LIO + adapter 局部里程计 | 否 |
| `agt_global_relocalization/global_relocalization.launch.py` | BBS + GICP 全局定位 | 否 |

`agt_system_bringup` 不负责启动 Livox 驱动、底盘驱动或 URDF 发布器；这些必须在
AGT launch 之前单独启动并检查。

默认验收不接入 HMI。先由 mapping producer 生成最终 PGO 地图，再由
`agt_map_manager create_map_package` 创建并自校验正式包，确认后用
`select_map_package` 选择版本，最后通过 `rviz_field_demo.launch.py` 启动导航。

## 2. ROS 数据接口

### 输入和输出 topic

| 数据 | 接口 | 消费者/发布者 | 说明 |
|---|---|---|---|
| Livox 原始点云 | `/livox/lidar`，Livox `CustomMsg` | 驱动 -> Batch-LIO | 保留逐点时间，不能先转成普通 PointCloud2 |
| IMU | `/livox/imu`，`sensor_msgs/Imu` | 驱动 -> Batch-LIO | 必须与实际 MID360 IMU 和时间戳一致 |
| 局部里程计 | `/agt/odometry/local` | adapter -> 定位/任务运行时 | 连续 `odom -> base_link` 运动 |
| 查询点云 | `/agt/livox/points`，`sensor_msgs/PointCloud2` | bridge -> 重定位/障碍物/tracker | 二级支路，不替代 Batch-LIO 原始输入 |
| 障碍物点云 | `/agt/navigation/points_obstacles`，`sensor_msgs/PointCloud2` | preprocessor -> Nav2 VoxelLayer | 导航局部代价地图输入 |
| 全局定位结果 | `/agt/relocalization/pose` | relocalizer -> manager | `PoseWithCovarianceStamped`，结果须通过质量门限 |
| 定位状态 | `/agt/localization/status` | manager -> guard/任务 | `LOCALIZED` 后才允许运动 |
| 地图状态 | `/agt/map/status` | map manager -> 定位/HMI | active map 的 ID、版本和资产路径 |
| Nav2 地图 | `/map` | `nav2_map_server` -> Nav2/RViz | 来自 Map Package 的 `map.yaml` |

### service/action

| 接口 | 作用 |
|---|---|
| `/agt/localization/relocalize` (`std_srvs/Trigger`) | 请求一次静止全局重定位 |
| `/agt/map/list`、`/agt/map/load` | 查询/加载 Map Package |
| `/agt/map/validate`、`/agt/map/activate` | 校验或激活地图版本 |
| `/agt/map/edit/*` | HMI staging 编辑生命周期，不直接覆盖正式包 |
| `/agt/map/generate` (`GenerateMapPackage`) | 地图管理器的包生成接口；建图算法本身仍属于 producer |
| `/navigate_to_pose` | Nav2 导航 action |

## 3. Map Package 格式

默认根目录为 `AGT_MAP_ROOT`，未设置时为 `~/ros2_ws/maps`。当前 v3 消费的
正式包结构如下：

```text
${AGT_MAP_ROOT}/<map_id>/<map_version>/
├── metadata.yaml                         # 必须，schema_version: 1
├── localization/
│   ├── global_map.pcd                    # 必须，最终优化后的 PGO 全局点云
│   └── relocalization/                   # 正式重定位资产目录
│       ├── relocalization_assets.yaml
│       ├── global_map_downsampled.pcd
│       ├── polar_context.db
│       ├── polar_context.yaml
│       └── voxelmaps_coords/
│           ├── voxel_params.txt
│           └── *.pcd
└── navigation/
    ├── map.yaml                          # 必须，Nav2 map_server 输入
    ├── map.pgm                           # 必须，由 map.yaml 的 image 引用
    ├── map.topology                       # 可选导航拓扑
    ├── elevation.pgm                     # 可选
    ├── slope.pgm                         # 可选
    └── obstacle.pgm                      # 可选
```

`metadata.yaml` 至少要声明：

```yaml
schema_version: 1
map_id: bunker_mid360_mapping_20260901_205036
map_version: v003-indexed
frame_id: map
assets:
  localization_map:
    path: localization/global_map.pcd
    sha256: <64-hex-digest>
  navigation_map:
    path: navigation/map.yaml
    sha256: <64-hex-digest>
  relocalization_assets:
    path: localization/relocalization
    sha256: <directory-digest>
```

校验要求：`localization_map` 和 `navigation_map` 必须存在；正式运行还必须有
`relocalization_assets`，且包含 Polar Context 数据库、配置、降采样地图和至少一个
voxel PCD。`map.yaml` 的 `image` 必须指向包内 `map.pgm`，并包含有效的
`resolution`、三元素 `origin`、`negate`、`occupied_thresh` 和 `free_thresh`。

重定位资产必须由同一份最终 PGO `global_map.pcd` 生成，不能把不同版本的 PCD、PGM
或 `polar_context.db` 混装。`select_map_package` 和 Map Manager 会校验路径、包内
边界、SHA-256 和资产内容；仅用于调试的显式 `rviz_field_demo` 则要求调用者自行保证
三份路径来自同一包。

## 4. Frame contract

```text
map
 └── odom                  # agt_localization_manager 唯一发布 map -> odom
      └── base_link        # Batch-LIO adapter 的连续局部运动边界
           └── livox_frame / lidar_link  # 实际安装 TF
```

- 正式 PGO 地图和定位查询使用 `mapping_body` 语义，内部位姿为 `T_map_body`。
- 发布到机器人边界时只做一次 `T_body_base_link` 转换。
- `base_link` 兼容查询模式已弃用，不能与 `mapping_body` 混用。
- 不允许第二个节点发布 `map -> odom`；定位失败时 manager 保持安全状态。
- 不要把 MID360 安装倾角在点云里二次“拉平”；以 URDF/static TF 为准。

## 5. 冷启动顺序

```text
1. source ROS + workspace
2. 启动 MID360、底盘、URDF/static TF，并确认 CustomMsg/IMU/TF
3. select_map_package 校验并写入 active_map.yaml
4. 启动 hmi_field_demo 或 rviz_field_demo
5. Map Manager 发布 active package 状态
6. Batch-LIO 输出局部里程计
7. bridge 输出 PointCloud2，重定位加载 global_map.pcd + Polar/BBS 资产
8. 重定位成功 -> Localization Manager 发布 map -> odom
9. Nav2 map_server 加载 map.yaml，确认 LOCALIZED 后再发送目标
```

导航启动不会自动完成建图，也不会从一个 PCD 临时生成 PGM 或 Polar Context。
如果资产不存在，应回到 mapping producer 导出并校验新版本 Map Package。

离线回放使用 `acceptance_offline_replay.launch.py`，它只从 bag 回放原始
`CustomMsg + IMU`，不会复用 bag 中的 `/tf`、里程计或定位结果。现场入口
`hmi_field_demo.launch.py` / `rviz_field_demo.launch.py` 则假定传感器驱动和真实
机器人 TF 已经由外部进程启动。
