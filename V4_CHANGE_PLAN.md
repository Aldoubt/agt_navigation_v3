# AGT Navigation Runtime V4 变更计划

> 本轮后续目标已统一到 [工作空间改造收敛与部署启动方案](docs/upgrade/WORKSPACE_CONSOLIDATION_PLAN.md)。本文保留原 V4 基线和阶段记录；新增范围与使用流程以统一方案为入口，验收仍沿用 V4 验收矩阵。

审计日期：2026-09-23。基线分支为 `refactor/navigation-runtime-v4`，HEAD 为
`e27abeb3fd63280ee8e48a47966e06588f8eebd8`。本文件先记录现状和实施边界；
本轮开始前已有的 8 项导航工作区修改属于用户现场工作，逐项见
`/home/yangxuan/ros2_ws_audit/navigation_v4/baseline.md`。

## 审计结论

- `colcon list --names-only` 发现 74 个包；现有 `agt_robot_bringup` 只是骨架，
  `agt_system_bringup/hardware.launch.py` 仍实际启动 robot_state_publisher、
  MID360、Bunker、RTK 和相机。
- 当前 `scripts/run_field_stack.sh` 和离线画线 launch 均硬编码地图路径；
  `navigation.launch.py` 接收绝对 `map.yaml`，尚无 `map:=auto` resolver。
- `agt_map_manager` 已能原子创建、校验、选择地图包，但没有正式的 candidate →
  promote、Robot Profile compatibility 和 registry 中 active/latest 两个游标。
- `map→odom` 由 `agt_localization_manager` 拥有；`odom→base_footprint` 由互斥
  LIO adapter 拥有；底盘 driver 配置 `publish_odom_tf=false`。
- `agt_navigation_benchmark` 与 V3 的 `tools/navigation_benchmark` 并存；
  `agt_pointcloud_pipeline` 与 V3 `agt_pointcloud_preprocessor` 并存，迁移前保留。
- `agt_robot_hmi/install` 的 3 个已跟踪文件在基线时已被删除；绝不清理、恢复或提交它们。
- 仓库根的 `.gitignore` 已覆盖常见生成目录；源码中当前没有这些历史目录。

## 分阶段变更

| 阶段 | Current → Target | 主要文件/包 | 风险 | 验证 |
|---|---|---|---|---|
| R0 | 未冻结现场 → 保存分支、SHA、状态、包和 launch 清单 | `ros2_ws_audit/navigation_v4/baseline.md` | 遗漏用户改动 | 比对 `git status --short` 原样记录；后续不覆盖 |
| R1 | 源码树混入生成物的历史风险 → 只清理未跟踪生成物并补 ignore | 各仓库 `.gitignore`、workspace inventory | `agt_robot_hmi/install` 有跟踪文件 | `git ls-files` 逐路径先查；`colcon list` 仍为 74 包 |
| R2 | `tracked_chassis_description` 现场 URDF + Robot Bringup 骨架 → `agt_robot_description` 和 `bunker_v1` Profile；由 Robot Bringup 唯一启动硬件 | `tracked_chassis_description`、`agt_robot_bringup`、`agt_system_bringup/hardware.launch.py` | TF 标定、driver 重复、Git 历史 | xacro/frames、launch owner 静态检查、受控构建；保持原标定值 |
| R3 | 地图路径散落且 active/latest 混合 → candidate 生命周期、promotion、registry/resolver、兼容检查 | `agt_map_manager`、`agt_mapping_artifacts`、`run_field_stack.sh`、相关 launch | 错图、同源资产混用、覆盖已发布地图 | auto/active/latest/精确版本/非法与不兼容用例；hash 与原子发布 |
| R4 | shell readiness + Nav2 原生入口 → Supervisor 和 Navigation Capability/Health | 新 `agt_navigation_interfaces`、`agt_navigation_capability`、`agt_navigation_supervisor`；复用现有 localization | 错误成功语义、失定位继续运动 | mock 传感器、定位、Nav2 action；失败时 fail closed |
| R5 | 导航包含拍照任务语义 → Mission Route/Waypoint/TaskGroup 与 Camera capability | 独立 `src/agt_mission` 仓库；迁移 `agt_rviz_patrol`/inspection runtime 调用 | 到点即跳到下一点、无限 retry | P01→Task A→P02→Task B→P03 状态机及失败/超时/取消测试 |
| R6 | RViz 直连旧 Nav2/服务 → 调用 Mission/Navigation Capability | `agt_rviz_patrol` | UI 与任务状态不一致 | 路线预览、开始/暂停/恢复/停止，模拟后端 |
| R7 | HMI 自有编排 → 与 RViz 共用后端 contract | `agt_robot_hmi`、接口文档 | 现有 HMI 未提交删除；跨仓库兼容 | mock API contract；不碰基线删除文件 |
| R8 | 两套 benchmark/pointcloud 职责 → 审计后按生产依赖迁移或保留 | `docs/v4_migration/*_audit.md`、V3 `tools/`、Mapping perception | 移走生产代码造成运行时缺包 | 依赖图、引用扫描、parity test、构建 |
| R9 | 分散验收 → V4 架构、约束、TF、地图、Mission、启动和报告 | `docs/*V4*.md`、`V4_MIGRATION_REPORT.md` | 将未跑实机误写 PASS | 分包 build/test、clean build、PASS/FAIL/NOT_RUN |

## 顺序和约束

每个阶段完成后先构建并运行与该阶段相关的测试；失败则停留在当前阶段修复。
不改变控制器、costmap、定位算法的数学参数和实测标定。现有运行入口在新入口验证
之前保留；避免把目录整理当成运行功能验收。正式地图为不可变版本，`map:=auto`
只从 registry 的 `latest_validated` 读取，`active` 单独保留为稳定生产指针。

实机 Cold Start、自动重定位和 NAV_READY 必须有真实现场证据；软件 mock 或回放
结果不能替代硬件验收。任一阶段涉及不明的真实几何或安全限制时保持现值并记录
`NOT_RUN`，不推测参数。
