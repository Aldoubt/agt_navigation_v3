# MCP 审查：v3 导航能力边界与荔枝果园升级路线

日期：2026-09-20。仓库：`agt_navigation_v3`，HEAD `1f517c7`（main，工作树干净）。
本文只做审查与规划，不修改代码、参数、地图资产或运行结果。

## 0. 前置结论：建图仓库尚未同步云端

`agt-lio-pgo-mapping` 仓库本地提交与远端一致（本地与 `origin/main` 均为
`7d4ff8e`，实时 `ls-remote` 已核对），但此前的 0.2.0 编排升级共 **28 个新增/修改文件
仍在工作树中未提交**，因此没有进入云端。需要用户确认后执行提交/推送；
`origin` 的只读查询无需交互凭据即可完成，推送预计可行但未经用户批准不做。

## 1. v3 系统已具备的能力（有证据支撑）

| 能力 | 现状 | 主要证据 |
| --- | --- | --- |
| 局部里程计 | Batch-LIO + adapter 发布 `/agt/odometry/local`，版本化外参，速度由位姿差分导出 | `docs/CURRENT_NAVIGATION_CAPABILITIES.md` §3-4 |
| 全局重定位 | Polar Context → Top-K 候选 → 3D-BBS 粗配准 → small_gicp 精化，无 `/initialpose`、无 RTK 种子；离线回放与 Gazebo 冷启动均达到 `LOCALIZED`（2026-09-05） | 同上 §5；`docs/GLOBAL_RELOCALIZATION_BBS_GICP.md` |
| TF 权限 | `agt_localization_manager` 是唯一 `map->odom` 发布者；`mapping_body` 合同冻结（v0.3.0） | `docs/acceptance/P3_RUNTIME_ACCEPTANCE_AUDIT.md` §1 |
| Nav2 | SmacPlanner2D + Regulated Pure Pursuit（50 Hz）+ velocity smoother；Gazebo 闭环 `NavigateToPose=SUCCEEDED` | capabilities §4 |
| 运动安全 | `cmd_vel_guard` fail-closed：定位丢失/陈旧即清零 `/mux/cmd_vel`，LOST→0 延迟 1.4 ms；速度上限前向 0.55 m/s、角速度 0.65 rad/s | `config/safety.yaml`；bench 验收 2026-09-05 |
| 巡检任务 | 点位队列 → NavigateToPose → 实测停止门（twist+位姿差分双证据、0.8 s 保持）→ C1 三视角拍摄 → 图片/位姿/RTK/云台角度归档 → RETURN_HOME；任务 YAML 支持逐点视角 | `docs/RVIZ_FIELD_ACCEPTANCE.md`；`云台相机任务与导航点设置说明.md` |
| 局部感知 | CustomMsg→PointCloud2 桥；range 0.5–120 m、自车盒过滤、radial-slope 地面滤波（坡度上限 35°、障碍高度 0.10–2.00 m）、0.10 m 体素；8×8 m 局部代价图 | `config/perception.yaml` |
| 地图转换 | 3D PCD → 2D 占据栅格：垂直跨度 `max_step`（例 0.22 m）或坡度 `max_slope_deg`（例 20°）判障；含冠层/悬空证据处理与轨迹 QA（PASS/REVIEW） | `cleaning/agt_map_converter`；`docs/acceptance/PRE_ACCEPTANCE_GATE.md` |
| 地图管理 | Map Package 合同：`navigation/map.yaml` + `localization/global_map.pcd` + `relocalization/`；`agt_map_manager` 校验激活地图 | `docs/MAP_MANAGEMENT.md` |
| 验收工具 | `field_build_smoke.sh`、硬件五项检查脚本、5/7/10 m 精度任务生成器、离线回放审计器 | `scripts/`、`tools/hardware_acceptance/` |

## 2. 能力边界（当前做不到或未验证）

### 2.1 实机闭环尚未发生

`acceptance_report.md` 五项硬件就绪检查全部 **NOT_RUN**。所有既有通过证据来自
台架、离线回放和 Gazebo；回放不能证明控制器到底盘的闭环行为。P3 审计列出 9 个
未闭合闸门（MAP 人工决策、实机拓扑、LIO 运行时健康、重定位重复性、TF 交接、
MapTracker 权限、Nav2 现场行为、安全与操作员流程、物理指标）。

### 2.2 感知边界

- 无地面分割之外的地形理解：无聚类/跟踪、无语义类别、无动态目标速度预测、
  无地形置信融合（capabilities §3 明示）。
- 动态行人/车辆只会被体素层"标记-清除"，行为依赖 Nav2 恢复策略，尚无果园
  动态场景实测。
- 高草/灌木可能被 0.10 m 障碍下限当作障碍；冠层回波依赖地图转换阶段的证据
  清理，运行时局部代价图不区分"可穿过的枝叶"与硬障碍。

### 2.3 定位边界

- 重定位明确要求补测：≥5 个差异起始位姿、树影与重复树形误报、履带振动、
  断电恢复。荔枝园的成排树干属于典型重复几何，风险在
  `RELOCALIZATION_DEBUGGING_RETROSPECTIVE_2026-09-05.md` 中已被点名。
- 运行中 `LOST` 后自动恢复重定位未启用；第一次现场测试遇到 LOST 只允许
  停车+人工重定位。
- RTK 始终只是元数据；冠层下 GNSS 退化不影响导航链路，但也不能用于救援。

### 2.4 规划与控制边界

- 只做点对点导航：无行间跟踪、无覆盖规划、无地头转弯策略；不匹配"自主巡航
  换行"目标（用户本次目标为定点巡检，边界吻合）。
- 2D 占据栅格 + 2D footprint，不含横滚/俯仰感知；本目标果园较平坦，属低优先。
- 代价图膨胀半径 0.55 m、footprint 1.04×0.80 m：行间净宽必须显著大于
  车宽+双侧膨胀，否则需要果园专属参数组。

### 2.5 任务与运维边界

- 无断电任务续跑、无逐点相机可选、无 HMI 集成（当前阶段明确排除）。
- 采集归档完整（图片时间戳 + map 位姿 + RTK + 云台角），但识别/统计在仓库外。

## 3. 荔枝果园升级路线（目标：较平坦规则树行的定点巡检）

升级顺序遵循既有闸门 `MAP -> LIO -> LOCALIZATION -> TRACKER -> NAV2 -> 闭环 -> 现场`，
上游未过不调下游。

### 阶段 0：决策与同步（0.5 天）

1. 用户确认后提交并推送建图 0.2.0 升级（28 个文件），或按模块拆分提交。
2. 冻结果园任务定义：巡检点清单、每树视角模板、返航点、最大允许速度。

### 阶段 1：在既有试验场完成 P3 实机闸门（先于果园）

不在 Bunker 上跑通一次短路线就进果园，会把定位、控制、机械问题混在一起无法归因：

1. `scripts/run_hardware_acceptance.sh` 五项检查（唯一节点、唯一 TF 权限、
   频率、运动中心、导航录包）全部 PASS。
2. 已知地图上的受控短路线 + 单点三视角拍摄，验证实测停止门在履带振动下的阈值。
3. 补做重定位 5 起始位姿测试（含重复几何点位）。

### 阶段 2：果园地图生产

1. 用建图 0.2.0 入口手动驾驶采集覆盖全部巡检行间的 rosbag（含回环）。
2. PGO 导出校验产物 → `pcd_to_nav_map` 转换：
   - 果园专属 `--max-step/--max-slope-deg`（较平坦场地建议保持 0.22 m / 20°，
     按实测沟坎调整）；
   - 重点审查 `trajectory_qa_status` 的 REVIEW 区域：冠层投影、树干膨胀、自车回波；
   - 检查行间净空 ≥ 车宽 + 2×膨胀 + 安全余量，不满足则改膨胀参数或改点。
3. 生成重定位资产（patches + BBS/Polar），Map Package 校验。

### 阶段 3：果园离线回放与定位加固

1. `acceptance_offline_replay.launch.py` 回放果园 bag：LIO 延迟不随时间增长、
   `map->odom->base_link` 连续、Nav2 能规划。
2. 在果园实测 ≥5 个起始位姿重定位，记录误报率；重复树行误报优先调
   候选半径/Top-K 与静止查询质量，不靠放宽门限掩盖。
3. 用果园 bag 调感知：地面滤波对草地的表现、`min_obstacle_height`、
   树干在膨胀后的可通行性；改动必须附回放对比证据。

### 阶段 4：果园现场分级测试

| 级别 | 内容 | 通过判据 |
| --- | --- | --- |
| L1 | 单条行、遥控优先、3 棵树 | 到位精度、停止门、三视角归档完整 |
| L2 | 单行往返 + 行人穿越干扰 | 停车-等待-重规划行为符合预期，不误闯 |
| L3 | 多行连续巡检 + RETURN_HOME | 全程无人工干预，`session`/归档完整 |
| L4 | 任务文件批量化（按树生成点位与视角） | 任务生成器输出可直接执行 |

### 阶段 5：任务产品化（L4 之后）

- 扩展点位生成器：从行几何自动生成每树侧向拍摄点与朝向。
- 果园三视角模板（仰视冠层）：复制 `front_sky_three_views.yaml` 定制 pitch，
  实机先确认 pitch 正负。
- 振动场景增大 `base_settle_time`/`settle_time`，不放宽到位容差。

### 明确不做（本轮）

- 行间自主跟踪/覆盖规划、语义可通行性、断电续跑、RTK 参与定位、
  HMI 集成。它们依赖阶段 1-4 的证据，先不引入复杂度。

## 4. 风险与对策

| 风险 | 对策 |
| --- | --- |
| 重复树行导致重定位误报 | 阶段 3 专项测试；静态查询质量优先；保留人工重定位兜底 |
| 高草/落叶造成虚假障碍 | 果园 bag 调参；保留 `patch_nav_map` 审计式修图，不手改源图 |
| 行间净宽不足 | 地图阶段量化净空；膨胀参数按果园配置文件单独管理 |
| 冠层遮挡与光照变化 | 依赖 LiDAR 链路，RTK 仅记录；拍摄点选择避开强逆光时段 |
| 行人与作业机械 | 速度上限维持 0.55 m/s；验证恢复策略；遥控随时接管优先 |
| 未提交代码丢失 | 阶段 0 立即提交推送建图修改 |

## 5. 本审查的证据与边界

- 依据：`AGENTS.md`、`README.md`、`CURRENT_NAVIGATION_CAPABILITIES.md`、
  `P3_RUNTIME_ACCEPTANCE_AUDIT.md`、`RVIZ_FIELD_ACCEPTANCE.md`、
  `PRE_ACCEPTANCE_GATE.md`、`acceptance_report.md`、`config/*.yaml`、
  `云台相机任务与导航点设置说明.md` 与 `agt_map_converter` 测试。
- 本文为静态审查，未运行导航节点、未连接实机、未改动任何参数。
- 现场执行顺序与安全边界仍以 `docs/HARDWARE_ACCEPTANCE_MID360_BUNKER.md`
  与 `AUTHORITY_MATRIX.md` 为准。
