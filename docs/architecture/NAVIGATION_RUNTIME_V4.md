# Navigation Runtime V4：工作树架构与边界

状态：2026-09-23 的源码快照；**不是实机验收或发布许可**。阶段依据为仓库根目录的 [V4_CHANGE_PLAN.md](../../V4_CHANGE_PLAN.md)。本文件描述当前接线和仍保留的兼容入口；正式 TF、传感器和定位数学契约仍以 [TF_CONVENTION.md](../contracts/TF_CONVENTION.md)、[POINTCLOUD_PIPELINE.md](../POINTCLOUD_PIPELINE.md) 等原有合同为准。

## 职责与启动入口

```text
agt_system_bringup/hardware.launch.py (兼容转发)
  └─ agt_robot_bringup/robot_hardware.launch.py
       ├─ agt_robot_description: bunker_v1 + 已核实标定 + robot_state_publisher
       ├─ MID360 / Bunker / RTK / camera-gimbal（按开关启用）
       └─ 底盘 publish_odom_tf=false

Mapping: mapping_source → experiments/ candidate → 校验/晋升 → maps/registry.yaml
Navigation: resolve_map(robot, map_spec) → Nav2 + localization + Supervisor + Capability
Mission: /navigation/navigate_to → 可选 /camera_gimbal/acquire_view → 下一航点
UI: RViz 画线 → /navigation/follow_route；HMI ROS2 → Navigation / Mission Action
```

| 边界 | 当前所有者 | 不变量与证据 |
| --- | --- | --- |
| 驱动与 `robot_state_publisher` | `src/agt_robot_platform/agt_robot_bringup/launch/robot_hardware.launch.py`；System Bringup 只转发 | 硬件不得由 Mapping 和 Navigation 各自重复实例化。Mapping live 通过 include 复用同一入口，但需要现场 graph 核验。 |
| 机器人结构 | `src/agt_robot_description` 的 `bunker_v1` Profile 与拆分后的 Xacro | 不推断未知尺寸、坡度或安全参数；标定和 TF 数值以现场基线为准。 |
| 全局 TF | `agt_localization_manager` 独占 `map → odom` | 互斥 LIO adapter 提供 `odom → base_footprint`，描述包提供静态车体/传感器帧；底盘不再发布竞争的 odom TF。参考正式 TF 合同。 |
| 导航障碍云 | `agt_pointcloud_preprocessor` | 生产 launch 不引入共享实验 plugin chain；见 [pointcloud_pipeline_audit.md](../v4_migration/pointcloud_pipeline_audit.md)。 |
| 不可变地图版本 | `agt_map_manager` 的 candidate、promotion、registry、resolver | `auto` 读取 `latest_validated`，`active` 是独立的生产游标；校验整包 SHA、资产 hash、Robot Profile 兼容性后才交给 Nav2。已有版本不得覆盖。 |
| 健康与导航动作 | `agt_navigation_supervisor`、`agt_navigation_capability` | `/navigation/health` 汇总传感器、odom、定位、TF、Nav2；`/navigation/navigate_to` 和 `/navigation/follow_route` 在未就绪、状态过期或结果非成功时 fail closed。 |
| 任务执行 | `agt_mission_runtime` | `/mission/execute_route` 执行 Route → Waypoint → 可选 TaskGroup；到点不等于拍照任务完成。当前只接受 `camera.acquire_view` handler；暂停、恢复、停止和取消须做完整集成验收。 |

## 地图与导航的版本契约

- Mapping 的新候选产物放在 `experiments/`；只有通过校验与晋升的版本才能进入 `maps/<map_id>/<map_version>/`。现有已发布地图不因迁移而原地改写。
- `map_catalog.resolve_map` 对 `auto`、`active`、`latest`、`map_id`、`map_id/version` 解析 registry；地图与 `bunker_v1` 不兼容、未注册、校验值不符时拒绝启动，不回退到硬编码的旧图。
- 2026-09-23 的 registry 中 `bunker_mid360` 有 `latest_validated: 20260922_143427-v4-001`，但 `active: ''`；同时目录内仍有历史 `active_map.yaml`。两种状态不能混为一谈，候选登记**不代表 V4 生产激活**；现场切换需单独审批和验证。
- 导航图、定位 PCD 与重定位资产必须同源。保留两套离线投影器直到相同 PCD/参数的 parity 证据齐备，见 [map_generation_audit.md](../v4_migration/map_generation_audit.md)。

## 入口关系和未收口边界

- `agt_system_bringup/navigation.launch.py` 启动 Nav2、Supervisor、Capability；外部 `agt_mission_bringup/mission.launch.py` 在其上增加 Mission Runtime，并可显式启动旧巡检运行时。
- RViz 点画线路径工具已提交 `FollowRoute` 到 Navigation Capability；**旧 `rviz_patrol.py` 仍调用 `/agt/mission/execute`**。这是兼容路径，不代表旧巡检 UI 已改用 V4 Mission。
- HMI 的 ROS2 通道使用 `/navigation/navigate_to`、`/mission/execute_route` 并订阅 Health/State；ROSBridge 导航控制仍明确拒绝。必须验证界面层不会绕开新门控。
- Benchmark 已迁入 V3 工具树；共享点云仓库保持独立 Git 历史和三个 ROS 包，未改变生产导航点云源。见 [benchmark_audit.md](../v4_migration/benchmark_audit.md) 与 [pointcloud_pipeline_audit.md](../v4_migration/pointcloud_pipeline_audit.md)。
- Map promotion 在临时目录完成 7 项包括激活前/后异常与 registry 写入失败的故障注入：已登记版本在激活失败时保持 `VALIDATED`/`PROMOTED`，上一个 active/pointer 得以恢复。跨文件断电窗口、旧 `select_map_package` 的双指针协同和正式环境恢复仍未验收，不能据此视为最终发布通过。

## 验证界线

此文档依据源码和已有日志，不会把单元测试、mock 或软件构建推论成唯一 TF 发布者、自动重定位成功或实机 `NAV_READY`。可执行的验收门、证据格式和 `NOT_RUN` 项目见 [V4_ACCEPTANCE_MATRIX.md](../acceptance/V4_ACCEPTANCE_MATRIX.md)；当前结果见仓库根目录的 [V4_MIGRATION_REPORT.md](../../V4_MIGRATION_REPORT.md)。
