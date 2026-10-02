# Bunker 双入口启动与离线验证（2026-09-27）

> 本轮只改代码和跑隔离测试；**没有启动真实驱动、发送运动目标或完成实机验收**。正式图 `bunker_mid360/20260924-trav-integration-v1` **没有已核准起点位姿**，因此近场初始化状态为 `SKIPPED`，不是定位成功或失败。不要用栅格 `origin`、零位姿或实验 `poses.txt` 首帧冒充建图起点。

## 两条独立生命周期

先在同一台机器人、同一 ROS_DOMAIN_ID 的**终端 H**启动唯一硬件所有者（只有取得现场授权、完成 CAN/MID360/C1/SDK 检查后才执行无 `--dry-run` 的命令）：

```bash
cd /home/yangxuan/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
bash src/agt_navigation_v3/scripts/run_bunker_hardware.sh --dry-run
# 实机、纯导航需要底盘＋MID360/IMU＋robot_state_publisher：
bash src/agt_navigation_v3/scripts/run_bunker_hardware.sh
# 实机、点选巡检则改用（不要同时执行以上两条实机命令）：
bash src/agt_navigation_v3/scripts/run_bunker_hardware.sh --camera
```

`--rtk` 仅在硬件入口选择 RTK 数据源（只用于记录，不参与定位），`--robot-config ID|PATH` 指向一份完整 Bunker 配置。硬件命令的 `--dry-run` 只解析配置／展示将要运行的 launch，不检查设备，也不启动任何驱动。硬件运行终端应保持打开；禁止另起一份 `hardware.launch.py`、Bunker、MID360 或 C1。Ctrl+C 停止硬件，应先停止导航、任务并确认底盘已停。

**终端 N**只接入外部硬件，按模式选一种：

```bash
cd /home/yangxuan/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
# 离线核对：不用硬件，显示所选地图、硬件所有权和起点状态。
bash src/agt_navigation_v3/scripts/run_field_stack.sh --map auto --mode navigation --dry-run
bash src/agt_navigation_v3/scripts/run_field_stack.sh --map auto --mode inspection --dry-run
# 取得现场许可、硬件已在 H 终端运行时（二选一）：
bash src/agt_navigation_v3/scripts/run_field_stack.sh --map auto --mode navigation --rviz
bash src/agt_navigation_v3/scripts/run_field_stack.sh --map auto --mode inspection --rviz
```

本轮的软件构建只写入隔离目录 `~/ros2_ws/experiments/bunker_offline_refactor_20260927/colcon_install`，**没有覆盖正式 `~/ros2_ws/install`**。因此若尚未正式重建三个改动的包，在终端 N 的导航命令前显式执行：

```bash
export AGT_FIELD_OVERLAY_SETUP=/home/yangxuan/ros2_ws/experiments/bunker_offline_refactor_20260927/colcon_install/setup.bash
```

导航脚本会在正常工作空间之后重新 source 该 overlay，并拒绝使用缺少起点校验／C1 健康模块的旧安装。正式现场发布时，可先离线审查并重建 `agt_global_relocalization agt_navigation_runtime agt_system_bringup`，重新 source 工作空间安装，然后不再设置这个实验 overlay。此配置只选择软件版本，不代替真实硬件验收。

导航／巡检启动前检查图上恰好一个 `/robot_state_publisher`、`/bunker`、`/livox_lidar_publisher`，所选 CAN 端口一致、Bunker `publish_odom_tf=false`，且 MID360 点云和 IMU 正在发布；巡检还要求唯一 C1 节点与新鲜 `/camera_gimbal/health`。它们**不会**启动或清理硬件；Ctrl+C 只关闭自己的 LIO／定位／Nav2／RViz。`/wheel/odom` 仅为诊断，不代替 LIO 静止判据。若硬件因掉线而退出，应停止任务和导航，重新检查后由 H 终端恢复，不能由 N 终端自动补启动。

## 定位顺序（Nav2 始终最后）

默认 `--localization-mode auto_then_manual`：

1. **可选近场**：仅对当前地图 ID／版本、当前 `localization/global_map.pcd` SHA-256、`T_map_body` 定义均匹配且有人核准的起点文件，调用一次局部 GICP。先等 LIO 静止、完整查询与新鲜扫描；服务只提出匹配位姿，Localization Manager 报 `LOCALIZED` 且全局修正有效、局部里程计新鲜并稳定后才算成功。没有文件则打印 `SKIPPED` 并直接进入第二步；文件错配或中途变化**停止启动**，不能静默使用错误先验。
2. **全图自动**：使用现有 Polar Context 候选＋3D-BBS 粗配准＋GICP 精配准，最多两次；近场失败后使用**新采集**的静止查询点云，不能直接重用失败扫描。
3. **人工保底**：自动均失败时，保留 LIO、切换初始化视图，等待 RViz `2D Pose Estimate` 在 `/initialpose` 指示粗略位置与朝向，经局部 GICP 和 Localization Manager 接受后继续。人工匹配失败继续等待，不放行 Nav2。

仅 Manager 确认定位且启动后的 TF／预检／Navigation Health 再次通过才进入可操作状态。显式 `--localization-mode auto` 仍可选“仅全图自动、失败退出”；`manual` 可选“直接等待操作员”。如果要复测，先关闭旧栈，不要同时运行第二套定位／TF。

### 如何准备起点先验（目前不要创建伪数据）

在受控建图记录与 PGO 后确认 **`T_map_body` 的真实起点**，人工审核坐标系、四元数与机器人姿态，并从该地图包的 `localization/global_map.pcd` 计算 SHA-256、对照地图元数据；然后在**地图包外部**保存为：

```text
/home/yangxuan/ros2_ws/map_start_hints/bunker_mid360/20260924-trav-integration-v1.yaml
```

格式如下（**示意字段，不是可用先验；占位内容不可直接上线**）：

```yaml
schema_version: 1
map_id: bunker_mid360
map_version: 20260924-trav-integration-v1
frame_id: map
pose_semantics: T_map_body
localization_map_sha256: '<由实际正式 PCD 得到的 64 位 SHA-256>'
source: '<可追溯的建图起点证据>'
reviewed_by: '<实际审核人>'
approved_for_near_search: false  # 未审核前保持 false，必须审核通过后明确改为 true
pose:
  x: '<确认值>'
  y: '<确认值>'
  z: '<确认值>'
  qx: '<确认值>'
  qy: '<确认值>'
  qz: '<确认值>'
  qw: '<确认值>'
```

可通过 `--start-hint /绝对路径/起点.yaml` 显式指定，或使用上述按地图 ID／版本自动查找的位置。缺省文件**不需要**为完成离线改造而填写。不要改写已发布地图包或将实验轨迹搬进去；位姿必须由现场资料核准，且文件校验只证明数据匹配，**不代表 GICP 已在机器人上成功**。

## RViz 两种交互不要混用

- **纯导航画线**：保留 RViz Path Tool 的连续绘制／预览／确认，经 `/navigation/follow_route` 执行。它不是巡检拍照航点，不能因为画过线就触发相机。
- **点选巡检**：在巡检模式中用 RViz 单独点选航点，再显式调用 `ros2 service call /agt/rviz_patrol/start std_srvs/srv/Trigger '{}'`。每个航点 NavigateToPose 到达后，使用 `/agt/odometry/local` **实测停车并稳定**，检查新鲜 C1 `CapabilityHealth.STATE_READY`（画面、串口、反馈及 Action 完好）才逐视角拍照；C1 恢复 READY 有短暂等待，超时／必需拍照失败则终止任务，**不再去下一点**。健康失败返回错误码 `1403`。巡检启动预检也检查 C1 的新鲜健康状态，Action 名称存在不等于相机可用。

## 验证边界与故障提示

本轮 `bash -n`、地图 `--dry-run`、纯单元测试、隔离 ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY 的假 Action／假 C1 话题测试或独立输出目录的构建，只算离线软件证据。真实传感器频率、CAN／LiDAR 连接、唯一 TF 发布者、近场／全图自动重定位、真实停车拍照及 `NAV_READY` 均是 **NOT_RUN**，必须按现场验收矩阵重新取证。当前设备不在线不阻止这些离线工作。

旧 Livox SDK/驱动版本组合在历史审计中有潜在不一致；实机硬件命令之前要核查 `ldd`／SDK 版本与驱动构建配置，不把 `--dry-run` 成功当设备已连接。不要在本轮测试运行真实驱动、底盘命令或实车航点。
