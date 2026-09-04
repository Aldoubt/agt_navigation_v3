# agt_navigation_v3

AGT 户外履带式巡检机器人 ROS 2 Humble 导航与巡检集成仓库。

目标平台：**Bunker v1 + 倾斜 MID360 + Fast-LIO2 建图 + Batch-LIO 导航里程计 + 3D-BBS/small_gicp 无初值全局重定位 + Nav2 + RTK/INS 记录 + Autolabor C1 云台相机**。

> **开发策略**：本轮 `runtime-v1` 合并后，`main` 作为唯一日常开发主线。当前仍处于初步设计/实车验证阶段，优先保持思路清晰、问题可定位、代码可快速修改，不长期维护多套功能分支。只有高风险或破坏性实验才临时开分支。
>
> **状态说明**：代码已经形成可测试主链，但尚未在目标 Ubuntu 22.04 / ROS 2 Humble + 实际 MID360/Bunker/C1 上完成整仓 build、rosbag 和硬件验收，因此“已落地”不等于 hardware PASS。

## 1. 当前 V1 主链

### Mapping Mode

```text
MID360 CustomMsg + built-in IMU
          ↓
robotics-laboratory/fast-lio2
          ↓
loop closure + GTSAM PGO
          ↓
可选 HBA refinement
          ↓
final global_map.pcd
          ├───────────────┐
          ↓               ↓
agt_map_converter     build_relocalization_assets
          ↓               ↓
Nav2/terrain maps     3D-BBS voxelmap + downsampled PCD
          └───────┬───────┘
                  ↓
             Map Package
```

### Navigation Mode

```text
MID360 CustomMsg + built-in IMU
          ↓
Functionhx/Batch-LIO
          ↓
/aft_mapped_to_init (camera_init -> body)
          ↓
agt_batch_lio_adapter
          ↓
/agt/odometry/local (odom -> base_link)
          ↓
Nav2 / stop gate / Localization Manager
```

### Global Relocalization

```text
/agt/localization/relocalize
          ↓
agt_localization_manager
          ↓
/agt/relocalization/request
          ↓
静止后的 /agt/livox/points 缓存
          ↓
lidar frame -> base_link
去掉已知 MID360 安装倾角
          ↓
3D-BBS 无初值全局粗搜索
          ↓
命中位置附近局部地图裁剪
          ↓
small_gicp GICP 精配准
          ↓
score / fitness / overlap gate
          ↓
/agt/relocalization/pose (T_map_base)
          ↓
agt_localization_manager
          ↓
T_map_odom = T_map_base × inverse(T_odom_base)
```

`agt_localization_manager` 是唯一 `map -> odom` owner。

### RTK V1

```text
RTK/INS
  ├─ 建图地理信息记录
  ├─ 巡检照片/云台位置记录
  ├─ health / quality
  └─ 未来 map <-> ENU 工具

当前禁止：
  RTK -> 自动重定位 seed
  RTK -> Batch-LIO correction
  RTK -> map->odom correction
```

树荫、fixed/float 变化或 RTK 标定偏差不会直接推动导航坐标系跳变。

---

## 2. 设计原则

- **建图和导航里程计解耦**：建图追求 PGO/HBA 后的全局一致性；导航里程计追求高频、连续、不跳变。
- **导航局部里程计使用 Batch-LIO**，不把回环后的建图轨迹直接当 Nav2 odom。
- **LiDAR map matching 是全局定位主链**，V1 不依赖 RTK 自动重定位。
- **一个 TF edge 只有一个 owner**：`agt_localization_manager` 唯一发布 `map -> odom`。
- MID360 实际安装倾角保留在 URDF/TF；Fast-LIO/Batch-LIO 原始时序链不经过 generic voxel/self-filter。
- 导航障碍/重定位使用独立 PointCloud2 分支 `/agt/livox/points`。
- 检测到机器人运动时清空重定位 scan cache；BBS query 只允许使用重新积累的静止扫描。
- Nav2 SUCCESS 不等于适合拍照；必须再通过 `/agt/odometry/local` measured-stop gate。
- 每张照片以 `image_stamp` 为同步锚点，关联 map pose、RTK 和云台实际角度。
- 50 Hz 是 Nav2 controller、velocity smoother、guard、CAN command refresh 的端到端要求。

---

## 3. 当前代码结构

```text
src/
├── agt_robot_interfaces/              typed msg/action/service
├── agt_livox_tools/                   Livox CustomMsg <-> PointCloud2
├── agt_pointcloud_preprocessor/       navigation-only self/range/rear/voxel filter
├── agt_mapping_bringup/               Fast-LIO2 + PGO/HBA wrappers + IMU preflight
├── agt_fastlio_adapter/               legacy/mapping odom adapter
├── agt_batch_lio_adapter/             Batch-LIO -> canonical odom/base_link
├── agt_global_relocalization/         scan cache / TF / map-follow / gates
├── agt_global_relocalization_native/  3D-BBS + local-submap small_gicp backend
├── agt_localization_manager/          sole map->odom owner
├── agt_rtk_manager/                   RTK quality / record-only manager
├── agt_map_manager/                   map package / version / hash / active map
├── agt_map_converter/                 PCD -> Nav2 + terrain products
├── agt_nav2_bringup/                  no-AMCL Nav2 baseline
├── agt_base_control/                  Bunker 50 Hz cmd_vel guard
├── agt_navigation_runtime/            Navigate -> stop -> C1 -> synchronized record
├── agt_rviz_patrol/                   RViz queue -> mission -> RETURN_HOME
└── agt_system_bringup/                staged system / RViz demo composition
```

关键文档：

```text
docs/GLOBAL_RELOCALIZATION_BBS_GICP.md
docs/FIELD_SENSOR_BASELINE.md
docs/MAPPING_AND_LIO_POLICY.md
docs/VIBRATION_AND_LI_INIT.md
docs/CURRENT_NAVIGATION_CAPABILITIES.md
```

---

## 4. 依赖与构建

### 基础仓库

```bash
mkdir -p ~/agt_ws/src
cd ~/agt_ws/src

git clone https://github.com/Aldoubt/agt_navigation_v3.git

git clone https://github.com/Aldoubt/agt_ins_driver.git
git clone https://github.com/Aldoubt/agt_bunker_base.git
git clone https://github.com/Aldoubt/agt_chassis_description.git
git clone https://github.com/Aldoubt/Autolabor-C1-ROS2.git
```

算法依赖版本固定在：

```text
dependencies/field_demo.repos
```

包括：

- `robotics-laboratory/fast-lio2`：建图；
- `Functionhx/Batch-LIO`：导航局部里程计；
- `KOKIAOKI/3d_bbs`：无初值全局粗定位；
- `koide3/small_gicp`：GICP 精配准；
- `morte2025/LiDAR_IMU_Init_ROS2`：可选标定验证。

```bash
cd ~/agt_ws
vcs import src < src/agt_navigation_v3/dependencies/field_demo.repos
```

### 安装 CPU 3D-BBS

```bash
cd ~/agt_ws/src/external/3d_bbs
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_CUDA=OFF
make -j
sudo make install
```

V1 先固定 CPU 路线，避免现场强依赖 CUDA。性能不足再切 GPU。

### 安装 small_gicp

```bash
cd ~/agt_ws/src/external/small_gicp
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_HELPER=ON
make -j
sudo make install
```

### 构建 workspace

```bash
cd ~/agt_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

---

## 5. 上车 P0：MID360 IMU / 外参

Batch-LIO V1 baseline：

```yaml
extrinsic_est_en: false
extrinsic_T: [0.011, 0.02329, -0.04412]
extrinsic_R: [1,0,0, 0,1,0, 0,0,1]
time_diff_lidar_to_imu: 0.0
```

配置：

```text
src/agt_mapping_bringup/config/batch_lio_mid360.yaml
```

先使用 MID360 内置 IMU。只有 vibration bag / map distortion / time alignment 的证据表明不够，才增加外置 IMU 或覆盖默认标定。

### 必做 `acc_norm` 测试

```bash
ros2 run agt_mapping_bringup mid360_imu_preflight.py --ros-args \
  -p duration_sec:=10.0
```

判断：

```text
mean_acc_norm ~ 1.0   -> Batch-LIO acc_norm = 1.0
mean_acc_norm ~ 9.81  -> Batch-LIO acc_norm = 9.81
其它                  -> FAIL，先查单位/驱动/传感器
静止 gyro 过大        -> FAIL，查振动/安装/状态
```

不要直接复制别人的 `mean_acc_norm / acc_norm`。

### LI-Init

`morte2025/LiDAR_IMU_Init_ROS2` 已固定版本，但不是开机依赖。只在以下证据出现后使用：

- 地图畸变持续指向 LiDAR/IMU 外参；
- 加减速/旋转存在明显 time offset；
- Batch-LIO 问题无法由振动/clipping 解释；
- LiDAR/IMU 刚性安装发生变化。

---

## 6. 建图与地图发布

### 6.1 建图

```bash
ros2 launch agt_mapping_bringup mapping_mode.launch.py
```

大地图建议保存 patches，必要时再 HBA refinement。**Nav2 和全局重定位最终必须使用同一份优化后的 `global_map.pcd`。**

### 6.2 生成 Nav2 / terrain 地图

```bash
ros2 run agt_map_converter pcd_to_nav_map \
  /data/site_A/global_map.pcd \
  --output /data/site_A/navigation \
  --resolution 0.10 \
  --max-step 0.22 \
  --max-slope-deg 20.0
```

```bash
ros2 run agt_map_converter validate_nav_map /data/site_A/navigation
```

必须看到：

```text
MAP VALIDATION PASS
```

### 6.3 预构建重定位资产

每个最终优化 PCD 只需要执行一次：

```bash
ros2 run agt_global_relocalization_native build_relocalization_assets \
  --map /data/site_A/global_map.pcd \
  --output /data/site_A/relocalization
```

输出：

```text
relocalization/
├── global_map_downsampled.pcd
├── relocalization_assets.yaml
└── voxelmaps_coords/
    ├── voxel_params.txt
    ├── 0.pcd
    ├── 1.pcd
    └── ...
```

运行时优先直接加载这个 BBS hierarchy；没有该目录时仍可 fallback 到现场重建，便于早期调试。

### 6.4 创建版本化 Map Package

```bash
ros2 run agt_map_manager create_map_package \
  --map-root ~/.ros/agt_maps \
  --map-id site_A \
  --map-version v1 \
  --source-pcd /data/site_A/global_map.pcd \
  --navigation-dir /data/site_A/navigation \
  --relocalization-assets-dir /data/site_A/relocalization
```

包内建议结构：

```text
~/.ros/agt_maps/site_A/v1/
├── metadata.yaml
├── localization/
│   ├── global_map.pcd
│   └── relocalization/
│       ├── global_map_downsampled.pcd
│       ├── relocalization_assets.yaml
│       └── voxelmaps_coords/...
├── navigation/
│   ├── map.yaml
│   ├── map.pgm
│   ├── elevation.pgm
│   ├── slope.pgm
│   └── obstacle.pgm
└── rtk/origin.yaml              # 可选
```

Map Manager 会对普通文件做 SHA256，对 `relocalization_assets` 目录做确定性 tree hash；加载时发现文件被修改会拒绝激活。

### 6.5 激活地图

```bash
ros2 launch agt_map_manager map_manager.launch.py
```

```bash
ros2 service call /agt/map/load \
  agt_robot_interfaces/srv/LoadMapPackage \
  "{map_id: 'site_A', map_version: 'v1'}"
```

`/agt/map/status` 发布：

```text
map_id / map_version / generation
navigation_map_yaml
localization_map_pcd
relocalization_assets_path
```

`agt_global_relocalization` 默认跟随该 active Map Package，因此不用再单独维护 BBS PCD 路径和 asset 路径。

**当前限制**：Nav2 在线热切图的两阶段 apply/rollback 还没完成。RViz Demo 启动时仍显式传同一 Map Package 的 `navigation/map.yaml`。不要把“Map Manager 已激活”理解成 Nav2 已自动 reload。

---

## 7. Global Relocalization V1

`Ikunio/Lidar_nav2_ws` 作为工程参考，不整包迁移。其 small_gicp relocalization 代码依赖 previous/initial pose，并自行广播 `map -> odom`，不符合我们的“真正无初值 + 单 TF owner”。

当前固定：

```text
KOKIAOKI/3d_bbs   -> global coarse
koide3/small_gicp -> local fine GICP
```

重定位前要求：

```text
Batch-LIO 正常
/agt/odometry/local 新鲜
/agt/livox/points 正常
lidar -> base_link TF 正确
机器人静止
active Map Package 正确
```

启动：

```bash
ros2 launch agt_global_relocalization global_relocalization.launch.py
ros2 launch agt_localization_manager localization_manager.launch.py
```

触发：

```bash
ros2 service call /agt/localization/relocalize std_srvs/srv/Trigger "{}"
```

在线性能路径：

```text
prebuilt BBS voxelmap
        ↓
全局 BBS 搜索
        ↓
coarse XYZ/RPY
        ↓
默认裁 35 m XY / ±8 m Z 局部地图
        ↓
small_gicp
```

局部地图点过少时自动 fallback 到降采样全图，优先保证可诊断而不是静默输出错误 pose。

---

## 8. Navigation / RViz Inspection Demo

硬件、Batch-LIO、Localization Manager、C1 和 Bunker driver 确认后：

```bash
ros2 launch agt_system_bringup rviz_demo.launch.py \
  map:=~/.ros/agt_maps/site_A/v1/navigation/map.yaml \
  map_id:=site_A_v1
```

人工 preflight：

```bash
ros2 run agt_navigation_runtime demo_preflight
```

RViz 用 **2D Goal Pose** 顺序点击巡检点，然后：

```bash
ros2 service call /agt/rviz_patrol/start std_srvs/srv/Trigger "{}"
```

每点：

```text
NavigateToPose
 -> SUCCESS
 -> measured stationary gate
 -> settle
 -> front_left_sky
 -> front_center_sky
 -> front_right_sky
 -> record image_stamp/map pose/RTK/gimbal
 -> next point
```

最后自动：

```text
RETURN_HOME -> measured stop -> COMPLETED
```

默认停稳门限：

```text
linear <= 0.03 m/s
angular <= 0.05 rad/s
continuous 0.8 s
odom freshness <= 0.5 s
max wait 8.0 s
```

履带振动导致静止 twist 抖动时，用 vibration bag 调阈值，不删除 stop gate。

记录目录：

```text
~/.ros/agt_inspection_records/
```

---

## 9. 当前验收顺序

1. `mid360_imu_preflight` PASS，确认 `acc_norm`；
2. 录制 vibration bag；
3. Fast-LIO2 + PGO 建图；
4. 大地图需要时做 HBA；
5. `agt_map_converter` + `validate_nav_map` PASS；
6. `build_relocalization_assets` 成功；
7. `create_map_package` + `/agt/map/load` 成功；
8. Batch-LIO + adapter 连续输出 `/agt/odometry/local`；
9. 多个不同启动点无 `/initialpose` 完成 3D-BBS + GICP；
10. 检查唯一 `map -> odom -> base_link` TF；
11. Nav2 单点到达和停车；
12. C1 三视角；
13. RViz 1 点 -> 3 图 -> HOME；
14. RViz 3 点 -> 9 图 -> HOME；
15. 稳定后再接 Qt HMI；
16. 再做断电续巡、自动 readiness 和最终参数冻结。

---

## 10. 当前改造进度

| 模块 | 状态 | 当前情况 |
| --- | --- | --- |
| Mapping Fast-LIO2 + PGO/HBA | 🟡 | 依赖/launch 骨架已固定，待实车建图 |
| Batch-LIO navigation odom | 🟡 | adapter 已落地，待实车参数验证 |
| MID360 factory baseline | 🟡 | 默认外参 + `acc_norm` preflight 已落地 |
| LI-Init | 🟡 optional | 固定版本，按证据启用 |
| 3D-BBS global coarse | 🟡 | native backend 已落地，待 target benchmark |
| BBS prebuilt assets | 🟡 | 离线 builder + runtime direct-load 已落地 |
| local-submap small_gicp | 🟡 | BBS 命中后局部 GICP 已落地 |
| Map Package | 🟡 | PCD/Nav2/BBS assets/version/tree hash 已落地 |
| Map Manager -> Relocalization | 🟡 | active map 路径自动跟随已落地 |
| Nav2 active-map hot apply | 🔴 | 两阶段 reload/rollback 尚未落地 |
| Localization Manager | 🟡 | 唯一 map->odom + 时间对齐/gate 已落地 |
| RTK manager | 🟡 | record/quality；不参与自动定位修正 |
| Map converter | 🟡 | PCD -> Nav2/elevation/slope/obstacle |
| Local obstacle perception | 🟡 | range/self/rear/voxel -> VoxelLayer |
| Nav2 | 🟡 | SmacPlanner2D + RPP + 50 Hz baseline |
| Bunker guard | 🟡 | `/cmd_vel -> /mux/cmd_vel` 50 Hz |
| Inspection runtime | 🟡 | measured-stop + C1 + synchronized record |
| RViz patrol | 🟡 | queue + 3 views + RETURN_HOME |
| HMI | ⏸️ | RViz 稳定后再接 |
| Auto readiness | ⏸️ | 后续 |
| Power-cycle mission resume | 🔴 | 后续实车阶段 |
| Full hardware acceptance | 🔴 | 需要目标 Humble + MID360 + Bunker + C1 |

---

## 11. Git / 分支策略

当前工程仍是快速设计验证期：

- `main` 保持最新可理解、可编译目标状态；
- 默认直接在 `main` 继续修问题和落地功能；
- 不为每个小模块维护长期功能分支；
- 只有大规模重构、上游替换或可能破坏现场版本的实验才开短期分支；
- 实车稳定以后，再考虑 release/tag 和冻结依赖版本。

核心目标不是保持很多“漂亮分支”，而是保证任何时候都能回答：**现在为什么这样设计、怎么启动、哪里还没验证、出现问题从哪里查。**
