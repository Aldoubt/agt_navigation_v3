# agt_navigation_v3

AGT 户外履带式巡检机器人 ROS 2 Humble 导航与巡检集成仓库。

目标平台：**Bunker v1 + 倾斜 MID360 + robotics-laboratory/fast-lio2 建图 + Functionhx/Batch-LIO 导航里程计 + 3D-BBS/small_gicp 全局重定位 + Nav2 + RTK/INS 记录 + Autolabor C1 云台相机**。

> 当前开发分支：`runtime-v1`
>
> 当前阶段只追求一条可重复、可在 RViz 中人工操作的最小闭环。Qt HMI 和纯自动 readiness 状态机暂缓，等 RViz 闭环稳定后再接入。
>
> 当前分支仍未在目标 Ubuntu 22.04 / ROS 2 Humble + 实际 MID360/Bunker/C1 上完成整仓 build 和硬件验收，因此“代码已落地”不等于 hardware PASS。

## 当前 V1 主链

### Mapping Mode

```text
MID360 CustomMsg + built-in IMU
          ↓
robotics-laboratory/fast-lio2
          ↓
FAST-LIO2 + loop closure + GTSAM PGO
          ↓
/pgo/save_maps (大地图建议 save_patches=true)
          ↓
可选 HBA refinement
          ↓
final global_map.pcd
          ↓
agt_map_converter
          ↓
map.yaml + map.pgm
+ elevation/slope/obstacle
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
/agt/relocalization/request
          ↓
最近若干帧 /agt/livox/points
          ↓
按 URDF TF 转到 base_link，移除 MID360 安装倾角
          ↓
3D-BBS：无初值全局粗搜索
          ↓
small_gicp：GICP 精配准
          ↓
/agt/relocalization/pose (T_map_base)
          ↓
agt_localization_manager
          ↓
T_map_odom = T_map_base × inverse(T_odom_base)
```

`agt_localization_manager` 是唯一 `map -> odom` owner。

### Inspection Demo

```text
RViz 点选 P001...PN
       ↓
逐点 NavigateToPose
       ↓
实测底盘持续静止
       ↓
front-left / front-center / front-right 三视角
       ↓
按 image_stamp 记录 map pose + RTK + actual gimbal angle
       ↓
最后自动 RETURN_HOME
       ↓
待机
```

## 设计原则

- **建图和导航里程计解耦**：建图需要 PGO/HBA 全局一致性；导航里程计需要高频、连续、不跳变。
- **导航局部里程计固定 Batch-LIO**，不把 loop closure 优化结果直接作为 Nav2 odom。
- **全局重定位不依赖 RTK**：V1 使用 LiDAR map matching；RTK 标定/树荫漂移不会移动导航坐标系。
- RTK/INS 当前只做建图地理记录、巡检照片/云台位置记录、质量显示和未来 map↔ENU 工具。
- `agt_localization_manager` 唯一发布 `map -> odom`；任何外部重定位包都只提供 pose，不允许自己抢 TF。
- MID360 实际安装倾角保留在 URDF；FAST-LIO/Batch-LIO 原始时序链不经过 generic voxel/self-filter。
- 导航/重定位 PointCloud2 使用独立 `/agt/livox/points` 分支。
- Nav2 SUCCESS 不等于可拍照；必须再通过 `/agt/odometry/local` 的 measured-stop gate。
- 50 Hz 是 Nav2 controller、velocity smoother、guard、CAN command refresh 的端到端要求。

## 当前代码结构

```text
src/
├── agt_robot_interfaces/              typed msg/action/service
├── agt_livox_tools/                   Livox CustomMsg <-> PointCloud2
├── agt_pointcloud_preprocessor/       navigation-only self/range/voxel filter
├── agt_mapping_bringup/               fast-lio2 + PGO/HBA mapping wrappers + IMU preflight
├── agt_fastlio_adapter/               legacy/mapping odom adapter
├── agt_batch_lio_adapter/             Batch-LIO camera_init/body -> canonical odom/base_link
├── agt_global_relocalization/         scan cache / TF alignment / gates / pose publisher
├── agt_global_relocalization_native/  3D-BBS coarse + small_gicp GICP fine backend
├── agt_localization_manager/          sole map->odom owner
├── agt_rtk_manager/                   RTK quality/record-only manager
├── agt_map_manager/                   map package/version/checksum
├── agt_map_converter/                 PCD -> Nav2 + terrain-derived products
├── agt_nav2_bringup/                  no-AMCL Nav2 baseline
├── agt_base_control/                  Bunker 50 Hz cmd_vel guard
├── agt_navigation_runtime/            Navigate -> stop -> C1 -> synchronized record
├── agt_rviz_patrol/                   RViz queue -> mission -> RETURN_HOME
└── agt_system_bringup/                staged system / RViz demo composition
```

关键设计文档：

```text
docs/GLOBAL_RELOCALIZATION_BBS_GICP.md
docs/FIELD_SENSOR_BASELINE.md
docs/MAPPING_AND_LIO_POLICY.md
docs/VIBRATION_AND_LI_INIT.md
docs/CURRENT_NAVIGATION_CAPABILITIES.md
```

# 依赖与构建

## 1. 基础仓库

```bash
mkdir -p ~/agt_ws/src
cd ~/agt_ws/src

git clone https://github.com/Aldoubt/agt_navigation_v3.git
git -C agt_navigation_v3 checkout runtime-v1

git clone https://github.com/Aldoubt/agt_ins_driver.git
git clone https://github.com/Aldoubt/agt_bunker_base.git
git clone https://github.com/Aldoubt/agt_chassis_description.git
git clone https://github.com/Aldoubt/Autolabor-C1-ROS2.git
```

## 2. 固定算法依赖

`dependencies/field_demo.repos` 已固定当前测试版本：

- `robotics-laboratory/fast-lio2`：建图；
- `Functionhx/Batch-LIO`：导航局部里程计；
- `KOKIAOKI/3d_bbs`：无初值全局粗定位；
- `koide3/small_gicp`：GICP 精配准；
- `morte2025/LiDAR_IMU_Init_ROS2`：可选离线标定验证。

如果机器安装了 `vcstool`：

```bash
cd ~/agt_ws
vcs import src < src/agt_navigation_v3/dependencies/field_demo.repos
```

3D-BBS 当前上游不提供常规 CMake package config，因此第一版先单独安装 CPU library：

```bash
cd ~/agt_ws/src/external/3d_bbs
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_CUDA=OFF
make -j
sudo make install
```

small_gicp helper：

```bash
cd ~/agt_ws/src/external/small_gicp
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_HELPER=ON
make -j
sudo make install
```

随后：

```bash
cd ~/agt_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

# 上车 P0：MID360 IMU / 外参基线

## MID360 V1 外参

Batch-LIO 第一版固定使用上游给出的 MID360 factory-consistency baseline：

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

先用 MID360 内置 IMU。只有真实 vibration bag / map distortion / time alignment 证据表明不够，才增加外置 IMU 或重新标定。

## 必做 acc_norm 测试

每台车第一次上车、MID360 firmware/driver 变化后必须静止测试：

```bash
ros2 run agt_mapping_bringup mid360_imu_preflight.py --ros-args \
  -p duration_sec:=10.0
```

工具自动统计静止加速度模长：

- 接近 `1.0`：推荐 Batch-LIO `acc_norm=1.0`；
- 接近 `9.81`：推荐 `acc_norm=9.81`；
- 不在两个合理区间：FAIL，先排查驱动单位/传感器状态；
- 静止角速度均值过大：FAIL，重新静止测试或检查振动。

不要直接复制别人的 `mean_acc_norm / acc_norm`。

## Bunker 振动 bag

```bash
bash $(ros2 pkg prefix agt_mapping_bringup)/share/agt_mapping_bringup/scripts/record_vibration_bag.sh
```

至少覆盖：静止、powered-static、低/中速直行、原地左右转、草地、碎石/颠簸、再次静止。

用该 bag 冻结：

```text
satu_acc / satu_gyro
IMU covariance/noise
stationary thresholds
是否发生 clipping
Batch-LIO pose/velocity jump
```

## LI-Init

固定可选工具：`morte2025/LiDAR_IMU_Init_ROS2`，版本见 `dependencies/field_demo.repos`。

它不是开机依赖。只有以下情况再跑：

- 地图畸变持续指向 LiDAR/IMU 外参问题；
- 加减速/旋转时表现出明显 time offset；
- Batch-LIO 问题无法用振动/clipping 解释；
- LiDAR/IMU 刚性安装发生变化。

标定 bag 要有充分 roll/pitch/yaw + 平移激励，并重复多次验证结果一致性，再覆盖 factory baseline。

# Mapping Mode

## 1. 建图

```bash
ros2 launch agt_mapping_bringup mapping_mode.launch.py
```

默认组合：

```text
fastlio2/lio_launch.py
pgo/pgo_launch.py
```

## 2. 保存地图

```bash
ros2 service call /pgo/save_maps \
  interface/srv/SaveMaps \
  "{file_path: '/data/site_A', save_patches: true}"
```

大地图计划 HBA 时保持 `save_patches=true`。

## 3. 可选 HBA

```bash
ros2 launch agt_mapping_bringup hba_refine.launch.py
```

再按上游 `/hba/refine_map` 服务进行一致性优化。最终导航与重定位共同使用**同一份最终优化 PCD**。

# PCD -> Nav2 地图

```bash
ros2 run agt_map_converter pcd_to_nav_map \
  /data/site_A/global_map.pcd \
  --output /data/site_A/navigation \
  --resolution 0.10 \
  --max-step 0.22 \
  --max-slope-deg 20.0
```

输出：

```text
map.yaml
map.pgm
elevation.pgm
slope.pgm
obstacle.pgm
converter_metadata.yaml
```

自检：

```bash
ros2 run agt_map_converter validate_nav_map /data/site_A/navigation
```

必须看到：

```text
MAP VALIDATION PASS
```

`0.10m / 0.22m / 20°` 只是初值，必须用真实场地 PCD 调整。

# Global Relocalization V1

## 为什么没有直接迁 Ikunio 整包

`Ikunio/Lidar_nav2_ws` 的 registration 目录是很好的工程参考，但其 `global_small_gicp_relocalization` / `global_relocalization` 实现本身从 `previous_result_t_` 或 RViz `initialpose` 起步，并直接广播 `map -> odom`。这不满足我们的“真正无初值 + 单一 TF owner”要求。

V1 只迁移其有价值的模式，并直接依赖更明确的上游：

```text
KOKIAOKI/3d_bbs      -> global coarse search
koide3/small_gicp    -> fine GICP
```

Scan Context 算法本身适合 outdoor place recognition，但官方公开实现是 CC BY-NC-SA；当前产品仓库不直接复制/内嵌该代码。

## 重定位操作

先保证：

```text
Batch-LIO 正常
/agt/odometry/local 正常
/agt/livox/points 正常
lidar -> base_link 静态 TF 正确
机器人静止
```

配置 `global_map`：

```text
src/agt_global_relocalization/config/global_relocalization.yaml
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

链路：

```text
Localization Manager
  -> /agt/relocalization/request
  -> scan cache
  -> base_link TF transform
  -> CPU 3D-BBS
  -> small_gicp
  -> score/fitness/overlap gates
  -> /agt/relocalization/pose
  -> Localization Manager
  -> map -> odom
```

V1 在重定位请求时检查机器人是否静止。缓存扫描逐帧变换到 `base_link` 以移除 MID360 安装倾角；当前尚未实现运动中多帧 odom deskew，所以不要边走边触发全局重定位。

### 大地图性能说明

当前 native V1 每次请求仍会从完整 PCD 建 BBS target 并让 GICP处理完整 map，是**功能基线**而不是最终大地图性能版本。

下一阶段优化固定为：

```text
Map Package 离线预构建 3D-BBS voxelmap
       +
运行时直接加载 voxelmap
       +
BBS 命中后裁剪局部 submap
       +
small_gicp 只精配准局部地图
```

不改变 `/agt/relocalization/pose` ROS 契约。

# Navigation Mode

Batch-LIO 原生：

```text
/livox/lidar CustomMsg
/livox/imu sensor_msgs/Imu
      ↓
/aft_mapped_to_init
camera_init -> body
```

我们的适配：

```text
agt_batch_lio_adapter
      ↓
使用 body <- base_link 静态 TF
转换 pose + twist
      ↓
/agt/odometry/local
odom -> base_link
```

不要简单把 `camera_init/body` 字符串改名成 `odom/base_link`。

# RTK/INS V1 Policy

启动 `agt_ins_driver` / `agt_rtk_manager` 仍然有价值，但当前只允许：

```text
mapping geographic record
inspection image/gimbal position record
RTK health/status
future surveyed map <-> ENU tooling
```

当前禁止：

```text
RTK -> automatic relocalization seed
RTK -> Batch-LIO correction
RTK -> map->odom correction/publisher
```

因此 RTK fixed/float 变化不会让实车导航坐标系跳动。

# RViz Inspection Demo

硬件/定位手动确认后：

```bash
ros2 launch agt_system_bringup rviz_demo.launch.py \
  map:=/data/site_A/navigation/map.yaml \
  map_id:=site_A_v1
```

人工 preflight：

```bash
ros2 run agt_navigation_runtime demo_preflight
```

RTK 目前不是导航放行条件；只有专门验证记录链时才加 `require_rtk:=true`。

RViz：

1. `Fixed Frame = map`；
2. 用 **2D Goal Pose** 顺序点 P001/P002/P003；
3. `/agt/rviz_patrol/markers` 显示队列；
4. 启动：

```bash
ros2 service call /agt/rviz_patrol/start std_srvs/srv/Trigger "{}"
```

默认每个巡检点：

```text
NavigateToPose
 -> Nav2 SUCCESS
 -> measured stationary gate
 -> extra settle
 -> front_left_sky
 -> front_center_sky
 -> front_right_sky
 -> next point
```

默认三视角：

```text
front_left_sky    heading -45°  pitch +35°
front_center_sky  heading   0°  pitch +45°
front_right_sky   heading +45°  pitch +35°
```

第一次 C1 实机必须确认 pitch 正负方向；若相反，只改 YAML 符号。

最后一个点完成后自动：

```text
RETURN_HOME -> measured stop -> COMPLETED -> standby
```

## 停稳门禁

默认：

```text
linear speed  <= 0.03 m/s
angular speed <= 0.05 rad/s
连续满足       0.8 s
odom freshness <= 0.5 s
maximum wait    8.0 s
```

履带振动导致静止 twist 抖动时，用 vibration bag 调阈值，不直接删除门禁。

## 数据记录

默认：

```text
~/.ros/agt_inspection_records/
```

记录 image path、image_stamp、map pose、RTK 经纬高/status/time delta、actual gimbal heading/roll/pitch、camera error code。

生成汇总：

```bash
ros2 run agt_navigation_runtime generate_demo_report /path/to/mission_dir
```

# 验收

## 单点

```text
P001 -> 3 images -> RETURN_HOME
```

```bash
ros2 run agt_navigation_runtime validate_records \
  /path/to/mission_dir \
  --expected-points 1
```

## 三点

```text
P001 -> 3
P002 -> 3
P003 -> 3
RETURN_HOME
```

```bash
ros2 run agt_navigation_runtime validate_records \
  /path/to/mission_dir \
  --expected-points 3
```

RTK 当前是记录项而不是导航条件，因此默认不要求 `--require-rtk`；只有测试 RTK 记录完整性时再开启。

## 当前优先验收顺序

1. `mid360_imu_preflight` PASS，确认 `acc_norm`；
2. vibration rosbag 录制完成；
3. fast-lio2 + PGO 建图；
4. 大地图可选 HBA；
5. `global_map.pcd -> agt_map_converter -> validate_nav_map PASS`；
6. Batch-LIO + adapter 输出连续 `/agt/odometry/local`；
7. 3D-BBS + small_gicp 在多个不同启动点无 `/initialpose` 重定位成功；
8. `map -> odom -> base_link` TF 只有唯一 owner 且连续；
9. 单独 Nav2 一点到达/停稳；
10. C1 三视角；
11. RViz 1点 -> 3图 -> HOME；
12. RViz 3点 -> 9图 -> HOME；
13. 以上稳定后再接 `agt_robot_hmi`。

# 当前改造进度

| 模块 | 状态 | 当前情况 |
| --- | --- | --- |
| Mapping fast-lio2 + PGO/HBA | 🟡 | 固定上游与 launch 骨架，待真实 MID360 build/建图 |
| Batch-LIO navigation odom | 🟡 | 固定上游，adapter 已落地，待 MID360 实车参数验证 |
| MID360 factory baseline | 🟡 | 默认外参已固定，`acc_norm` 强制 preflight 已落地 |
| LI-Init | 🟡 optional | 固定 ROS2 仓库/版本，只有证据表明默认标定有问题再启用 |
| 3D-BBS global coarse | 🟡 | native backend 已落地，待 target build + rosbag benchmark |
| small_gicp fine registration | 🟡 | 已接 BBS coarse result，门限待实测 |
| Localization Manager | 🟡 | 唯一 map->odom + 时间对齐/covariance gate 已落地 |
| RTK manager | 🟡 | quality/record chain 已落地；明确不参与自动定位修正 |
| Map converter | 🟡 | PCD -> Nav2/elevation/slope/obstacle + validator 已落地 |
| Local obstacle perception | 🟡 | range/self/rear-sector/voxel -> Nav2 VoxelLayer 已落地 |
| Nav2 | 🟡 | SmacPlanner2D + RPP + 50Hz baseline |
| Bunker guard | 🟡 | `/cmd_vel -> /mux/cmd_vel` 50Hz |
| Inspection runtime | 🟡 | measured-stop + C1 + synchronized records |
| RViz patrol | 🟡 | queue + 3 views + RETURN_HOME |
| HMI | ⏸️ | RViz 稳定后再接 |
| Auto readiness | ⏸️ | 当前阶段不做 |
| BBS prebuilt voxelmap/local submap | 🔴 | 下一轮大地图性能优化 |
| Power-cycle mission resume | 🔴 | RViz Demo 稳定后 |
| Full hardware acceptance | 🔴 | 需要目标 Humble + MID360 + Bunker + C1 |
