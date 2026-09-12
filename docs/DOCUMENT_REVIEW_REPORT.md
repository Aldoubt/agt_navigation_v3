# Documentation Content Review Report

## Scope and Authority Rules

This report audits all 57 Markdown/YAML documents currently below `docs/`.
It does not move, delete, or modify any existing document. `KEEP` means the
document remains an active authority or required navigation/evidence document;
it does not mean every sentence is current. `MERGE` means retain the source
for useful content but consolidate its authoritative claims elsewhere.
`ARCHIVE` means retain as historical evidence, not as current specification.
`REVIEW` means an owner must resolve status before the document can be treated
as authoritative.

## 1. Authority Matrix

| 文档 | 类型 | 当前作用 | 是否保留 | 推荐动作 |
|---|---|---|---|---|
| [CODEX_CONTEXT.md](CODEX_CONTEXT.md) | 当前上下文/里程碑 | v0.3.0 localization contract freeze、冻结决策和下一里程碑 | 是，当前权威 | KEEP |
| [DOCUMENT_INDEX.md](DOCUMENT_INDEX.md) | 索引 | docs 全量清单与分类入口 | 是，导航权威 | KEEP |
| [DOCUMENT_MIGRATION_PLAN.md](DOCUMENT_MIGRATION_PLAN.md) | 流程 | 文档分类与迁移门槛 | 是，流程权威 | KEEP |
| [ARCHITECTURE_BASELINE.md](ARCHITECTURE_BASELINE.md) | 架构基线 | v0.1 系统架构早期基线 | 是，保留依据 | MERGE |
| [ADVANCED_DESIGN_V2.md](ADVANCED_DESIGN_V2.md) | 设计决策 | terrain、map manager、controller、RTK 等设计决策 | 是，需区分规划与实现 | MERGE |
| [architecture/TARGET_FUNCTIONAL_ARCHITECTURE.md](architecture/TARGET_FUNCTIONAL_ARCHITECTURE.md) | 目标架构 | 当前功能域和包拓扑目标 | 是，结构权威 | KEEP |
| [architecture/RUNTIME_ARCHITECTURE.md](architecture/RUNTIME_ARCHITECTURE.md) | 运行时架构 | 进程、DDS 边界和高带宽链路 | 是，运行时权威 | KEEP |
| [architecture/map_pipeline_design.md](architecture/map_pipeline_design.md) | 架构设计 | 通用地图生成流水线 | 是，作为通用模型 | MERGE |
| [design/map_manager_architecture.md](design/map_manager_architecture.md) | 组件设计 | Map Manager 生命周期和接口基线 | 是，Map Manager 设计权威 | KEEP |
| [design/operator_mode_upgrade.md](design/operator_mode_upgrade.md) | 操作流程设计 | operator mode 设计及任务记录 | 是，当前操作流程记录 | KEEP |
| [FIELD_MAP_FINALIZATION.md](FIELD_MAP_FINALIZATION.md) | 现场流程 | PGO 结果到正式地图包的流程 | 是，现场操作权威 | KEEP |
| [HMI_MAP_WORKFLOW.md](HMI_MAP_WORKFLOW.md) | HMI/地图契约 | EditSession、地图编辑和导航目标 | 是，HMI/map-edit 权威 | KEEP |
| [IMPLEMENTATION_ROADMAP_v1.md](IMPLEMENTATION_ROADMAP_v1.md) | 路线图 | 产品能力阶段和冻结规则 | 是，但不是运行时权威 | REVIEW |
| [MAP_CONVERTER_DESIGN.md](MAP_CONVERTER_DESIGN.md) | 转换器设计 | PCD 到 Nav2 地图的基础转换方案 | 是，作为 fallback 设计 | MERGE |
| [MAP_MANAGEMENT.md](MAP_MANAGEMENT.md) | 地图设计 | 早期 Map Package 生命周期概念 | 是，保留有效概念 | MERGE |
| [MIGRATION.md](MIGRATION.md) | 环境/恢复指南 | workspace 恢复和迁移步骤 | 是，当前恢复指南 | KEEP |
| [NAV2_MAP_POLICY.md](NAV2_MAP_POLICY.md) | 地图策略 | localization map 与 Nav2 map 的所有权边界 | 是，活动地图契约 | KEEP |
| [ODOMETRY_MAPPING_SELECTION.md](ODOMETRY_MAPPING_SELECTION.md) | 选型依据 | FAST-LIO2/Batch-LIO 选型理由 | 是，保留决策依据 | MERGE |
| [TERRAIN_MAP_DESIGN.md](TERRAIN_MAP_DESIGN.md) | 地形架构 | patch-based terrain 与地图分离设计 | 是，地形设计权威 | KEEP |
| [contracts/TF_CONVENTION.md](contracts/TF_CONVENTION.md) | TF 契约 | TF 所有权和坐标变换约定 | 是，TF 权威 | KEEP |
| [contracts/GLOBAL_RELOCALIZATION_SDK_CONTRACT.md](contracts/GLOBAL_RELOCALIZATION_SDK_CONTRACT.md) | SDK 契约 | ROS/backend relocalization 边界 | 是，适配器契约 | KEEP |
| [contracts/backend_contract_matrix.yaml](contracts/backend_contract_matrix.yaml) | 机器可读契约 | `mapping_body`、`T_map_body`、发布边界和兼容模式 | 是，最高精度 frame 权威 | KEEP |
| [design/candidate_bbs_mapping_body_experiment.md](design/candidate_bbs_mapping_body_experiment.md) | 实验/契约依据 | BBS frame mode 候选方案和实验依据 | 是，保留理由 | MERGE |
| [GLOBAL_RELOCALIZATION_BBS_GICP.md](GLOBAL_RELOCALIZATION_BBS_GICP.md) | relocalization 架构 | BBS coarse + small_gicp fine 的当前方案 | 是，当前 relocalization 设计权威 | KEEP |
| [LIVOX_DATA_AND_RELOCALIZATION_PRIMER.md](LIVOX_DATA_AND_RELOCALIZATION_PRIMER.md) | 数据策略 | Livox 格式和 BBS/GICP 数据规则 | 是，数据契约/说明 | KEEP |
| [MAPPING_AND_LIO_POLICY.md](MAPPING_AND_LIO_POLICY.md) | LIO 策略 | mapping/navigation LIO 分工和地图派生 | 是，LIO 策略权威 | KEEP |
| [MAINLINE_POLICY.md](MAINLINE_POLICY.md) | 开发策略 | mainline 和验证策略 | 是，开发策略权威 | KEEP |
| [POINTCLOUD_PIPELINE.md](POINTCLOUD_PIPELINE.md) | 点云契约 | 过滤顺序和不可破坏的处理约束 | 是，点云契约 | KEEP |
| [ACCEPTANCE.md](ACCEPTANCE.md) | 软件验收 | offline、Gazebo、V1 acceptance gates | 是，软件验收权威 | KEEP |
| [acceptance/PRE_ACCEPTANCE_GATE.md](acceptance/PRE_ACCEPTANCE_GATE.md) | 现场前置验收 | MAP/LIO/localization/Nav2/field 的有序 gate | 是，前置验收权威 | KEEP |
| [acceptance/FIELD_ACCEPTANCE_V1.md](acceptance/FIELD_ACCEPTANCE_V1.md) | 现场验收 | V1 现场模式和目标 | 是，但需确认分支状态 | REVIEW |
| [BOOTSTRAP_AND_ROSBAG_GATE.md](BOOTSTRAP_AND_ROSBAG_GATE.md) | 可复现性验收 | bootstrap 与 rosbag-to-field gate | 是，现场准备流程 | KEEP |
| [CURRENT_NAVIGATION_CAPABILITIES.md](CURRENT_NAVIGATION_CAPABILITIES.md) | 能力快照 | runtime-v1 已实现能力 | 是，能力权威 | KEEP |
| [FIELD_SENSOR_BASELINE.md](FIELD_SENSOR_BASELINE.md) | 硬件基线 | MID360/IMU 和标定 preflight | 是，传感器权威 | KEEP |
| [INTEGRATION_RUNTIME_V1.md](INTEGRATION_RUNTIME_V1.md) | 集成验收 | 跨仓库运行时边界和已知缺口 | 是，集成边界 | KEEP |
| [RVIZ_FIELD_ACCEPTANCE.md](RVIZ_FIELD_ACCEPTANCE.md) | 现场操作验收 | 当前阶段 RViz 现场验收程序 | 是，当前现场路径 | KEEP |
| [VIBRATION_AND_LI_INIT.md](VIBRATION_AND_LI_INIT.md) | 诊断流程 | 振动诊断与 LI-Init | 是，当前诊断流程 | KEEP |
| [refactor/DEFAULT_TEST_WORKFLOW.md](refactor/DEFAULT_TEST_WORKFLOW.md) | 测试流程 | mapping/debug-navigation 固定流程 | 是，内容并入验收入口 | MERGE |
| [refactor/R1_1_RUNTIME_VALIDATION.md](refactor/R1_1_RUNTIME_VALIDATION.md) | 历史验证 | R1.1 debug runtime 结果 | 是，证据 | ARCHIVE |
| [refactor/R1_1_SIM_INTERFACE_AUDIT.md](refactor/R1_1_SIM_INTERFACE_AUDIT.md) | 历史审计 | R1.1 simulation interface 边界 | 是，证据 | ARCHIVE |
| [refactor/R1_1_SIMULATION_AUDIT.md](refactor/R1_1_SIMULATION_AUDIT.md) | 历史审计 | R1.1 simulation 能力和缺口 | 是，证据 | ARCHIVE |
| [refactor/R1_1_SIMULATION_VALIDATION.md](refactor/R1_1_SIMULATION_VALIDATION.md) | 历史验证 | R1.1 simulation 验证结果 | 是，证据 | ARCHIVE |
| [refactor/TEST_REPORT.md](refactor/TEST_REPORT.md) | 历史测试 | refactor 构建/测试证据 | 是，证据 | ARCHIVE |
| [archive/CURRENT_ARCHITECTURE_AUDIT.md](archive/CURRENT_ARCHITECTURE_AUDIT.md) | 历史审计 | 2026-09-10 包和功能域审计 | 是，追溯证据 | ARCHIVE |
| [archive/P2_20_MAPPING_BODY_ACCEPTANCE_CHECKLIST.md](archive/P2_20_MAPPING_BODY_ACCEPTANCE_CHECKLIST.md) | 历史验收 | P2.20 frame-contract checklist | 是，历史记录 | ARCHIVE |
| [archive/p2_20_mapping_body_frame_contract.md](archive/p2_20_mapping_body_frame_contract.md) | 历史契约 | P2.20 `mapping_body` 契约 | 是，已被 P2.21 替代 | ARCHIVE |
| [archive/P2_21_GLOBAL_LOCALIZATION_FRAME_CONTRACT_CHECKLIST.md](archive/P2_21_GLOBAL_LOCALIZATION_FRAME_CONTRACT_CHECKLIST.md) | 历史验收 | P2.21 frame-contract checklist | 是，历史记录 | ARCHIVE |
| [archive/P2_21_GLOBAL_LOCALIZATION_FRAME_CONTRACT_FREEZE.md](archive/P2_21_GLOBAL_LOCALIZATION_FRAME_CONTRACT_FREEZE.md) | 历史冻结 | P2.21 frame-contract freeze | 是，冻结证据 | ARCHIVE |
| [archive/ROSBAG_ANALYSIS.md](archive/ROSBAG_ANALYSIS.md) | 历史分析 | rosbag 清单和格式分析 | 是，追溯证据 | ARCHIVE |
| [archive/ROSBAG_RELOCALIZATION_BENCHMARK.md](archive/ROSBAG_RELOCALIZATION_BENCHMARK.md) | 历史基准 | relocalization benchmark 方法 | 是，可复用证据 | ARCHIVE |
| [design/map_manager_audit.md](design/map_manager_audit.md) | 历史审计 | design 分支的 Map Manager 审计 | 是，审计证据 | ARCHIVE |
| [map_manager_audit.md](map_manager_audit.md) | 历史审计 | 较新的本地树 Map Manager 实现缺口快照 | 是，审计证据 | ARCHIVE |
| [RELOCALIZATION_DEBUGGING_RETROSPECTIVE_2026-09-05.md](RELOCALIZATION_DEBUGGING_RETROSPECTIVE_2026-09-05.md) | 历史复盘 | relocalization 调试经验和 replay 证据 | 是，复盘证据 | ARCHIVE |
| [refactor/build_baseline_report.md](refactor/build_baseline_report.md) | 历史构建 | R1.1 构建结果和已知失败 | 是，证据 | ARCHIVE |
| [refactor/FUNCTIONAL_DOMAIN_COMPLETION.md](refactor/FUNCTIONAL_DOMAIN_COMPLETION.md) | 历史迁移 | 功能域迁移完成记录 | 是，迁移证据 | ARCHIVE |
| [refactor/MIGRATION_PLAN.md](refactor/MIGRATION_PLAN.md) | 历史迁移 | 早期源码树迁移计划 | 是，历史计划 | ARCHIVE |
| [refactor/MIGRATION_REPORT.md](refactor/MIGRATION_REPORT.md) | 历史迁移 | 早期迁移状态报告 | 是，历史报告 | ARCHIVE |

## 2. 重复内容与替代关系

### 2.1 Frame contract 演进链

内容关系为：

`archive/p2_20_mapping_body_frame_contract.md`
→ `archive/P2_20_MAPPING_BODY_ACCEPTANCE_CHECKLIST.md`
→ `archive/P2_21_GLOBAL_LOCALIZATION_FRAME_CONTRACT_FREEZE.md`
→ `archive/P2_21_GLOBAL_LOCALIZATION_FRAME_CONTRACT_CHECKLIST.md`
→ `contracts/backend_contract_matrix.yaml` + `contracts/TF_CONVENTION.md`。

P2.20/P2.21 文档应作为冻结过程和验收证据，不应与当前 YAML 矩阵并列解释为
当前规范。当前 frame authority 应以 YAML 矩阵、TF convention，以及当前
relocalization 文档共同解释；其中 YAML 矩阵是最精确的机器可读来源。

### 2.2 Map Manager 与地图包

`ARCHITECTURE_BASELINE.md`、`ADVANCED_DESIGN_V2.md`、`MAP_MANAGEMENT.md`、
`design/map_manager_architecture.md`、`FIELD_MAP_FINALIZATION.md`、
`HMI_MAP_WORKFLOW.md` 和两个 `map_manager_audit.md` 都涉及地图包、生命周期
或 Map Manager，但职责不同：

- 生命周期和接口：`design/map_manager_architecture.md`；
- 现场 finalize：`FIELD_MAP_FINALIZATION.md`；
- HMI 编辑：`HMI_MAP_WORKFLOW.md`；
- 实现状态和缺口：两个 audit，仅作证据；
- 早期概念：`MAP_MANAGEMENT.md`，建议并入当前 Map Manager 设计。

两个 `map_manager_audit.md` 不是字节级重复。`docs/map_manager_audit.md` 是
较新的本地实现快照，信息更细；但两者都不是规范性架构文档。

### 2.3 Map generation pipeline

`architecture/map_pipeline_design.md`、`MAP_CONVERTER_DESIGN.md`、
`TERRAIN_MAP_DESIGN.md`、`NAV2_MAP_POLICY.md`、`MAPPING_AND_LIO_POLICY.md`
和 `acceptance/PRE_ACCEPTANCE_GATE.md` 有交叠，但应分工：

- terrain-aware 首选设计：`TERRAIN_MAP_DESIGN.md`；
- 基础/fallback 转换器：`MAP_CONVERTER_DESIGN.md`；
- localization map 与 Nav2 map 所有权：`NAV2_MAP_POLICY.md`；
- LIO 和地图来源策略：`MAPPING_AND_LIO_POLICY.md`；
- gate 顺序：`acceptance/PRE_ACCEPTANCE_GATE.md`；
- 通用流水线说明：`architecture/map_pipeline_design.md`，适合合并为背景章节。

### 2.4 LIO、点云和传感器策略

`ODOMETRY_MAPPING_SELECTION.md`、`MAPPING_AND_LIO_POLICY.md`、
`LIVOX_DATA_AND_RELOCALIZATION_PRIMER.md`、`CURRENT_NAVIGATION_CAPABILITIES.md`、
`FIELD_SENSOR_BASELINE.md` 和 `VIBRATION_AND_LI_INIT.md` 都涉及 LIO 或传感器。
推荐权威分工为：

- 选型策略：`MAPPING_AND_LIO_POLICY.md`；
- Livox 消息格式和转换：`LIVOX_DATA_AND_RELOCALIZATION_PRIMER.md`；
- 已实现能力：`CURRENT_NAVIGATION_CAPABILITIES.md`；
- 硬件 preflight：`FIELD_SENSOR_BASELINE.md`；
- 振动/初始化诊断：`VIBRATION_AND_LI_INIT.md`。

`ODOMETRY_MAPPING_SELECTION.md` 保留决策理由，但不再单独作为策略权威。

### 2.5 Acceptance 与 test evidence

`ACCEPTANCE.md`、`acceptance/PRE_ACCEPTANCE_GATE.md`、
`acceptance/FIELD_ACCEPTANCE_V1.md`、`RVIZ_FIELD_ACCEPTANCE.md`、
`refactor/DEFAULT_TEST_WORKFLOW.md` 以及 R1.1 记录互相引用相同的验证阶段，
但时间和范围不同：

- 软件/offline/Gazebo gate：`ACCEPTANCE.md`；
- 进入现场前的有序 gate：`acceptance/PRE_ACCEPTANCE_GATE.md`；
- 当前 RViz 实车路径：`RVIZ_FIELD_ACCEPTANCE.md`；
- V1 正式现场目标：`acceptance/FIELD_ACCEPTANCE_V1.md`，需确认其旧 commit 基线；
- R1.1 文件和 `TEST_REPORT.md`：归档证据，不作为当前通过条件。

### 2.6 Architecture migration records

`architecture/TARGET_FUNCTIONAL_ARCHITECTURE.md` 描述目标结构；
`archive/CURRENT_ARCHITECTURE_AUDIT.md`、
`refactor/FUNCTIONAL_DOMAIN_COMPLETION.md`、
`refactor/MIGRATION_PLAN.md` 和 `refactor/MIGRATION_REPORT.md` 描述迁移过程。
后四者不应被 AI 当作当前运行时架构来源。

## 3. 权威冲突和被替代文档

- `CODEX_CONTEXT.md` 将当前里程碑定为 v0.3.0 localization contract freeze，
  下一阶段是 runtime acceptance、Nav2 field test、operator workflow。
- `ACCEPTANCE.md` 的 offline/Gazebo gate 已通过，但
  `PRE_ACCEPTANCE_GATE.md`、`RVIZ_FIELD_ACCEPTANCE.md` 和
  `FIELD_ACCEPTANCE_V1.md` 表明真实 Bunker field acceptance 仍是开放阶段。
- `FIELD_ACCEPTANCE_V1.md` 固定在较早 commit `8d0ab16`，不能不加核验地定义
  当前分支状态；因此标记 `REVIEW`。
- `ADVANCED_DESIGN_V2.md` 中部分 RTK、terrain、map lifecycle 和 recovery 内容
  是规划性描述；必须由 `CURRENT_NAVIGATION_CAPABILITIES.md` 区分已实现能力。
- `map_manager_audit.md` 指出 Map Manager 结构存在但生成/编排仍不完整，
  因此限制了 `ADVANCED_DESIGN_V2.md` 中“进入 V1”的乐观表述。
- P2.20 frame contract 已被 P2.21 冻结链替代；P2.21 的最终机器可读表达由
  `contracts/backend_contract_matrix.yaml` 承接。
- `refactor/MIGRATION_PLAN.md` 和 `refactor/MIGRATION_REPORT.md` 是源码树迁移
  的历史记录，不应替代当前文档分类计划 `DOCUMENT_MIGRATION_PLAN.md`。

## 4. 当前架构推荐阅读路径

AI 应按“上下文 → 结构 → 契约 → 数据/定位 → 地图 → 验收”的顺序阅读：

1. [DOCUMENT_INDEX.md](DOCUMENT_INDEX.md) → [DOCUMENT_MIGRATION_PLAN.md](DOCUMENT_MIGRATION_PLAN.md)
2. [CODEX_CONTEXT.md](CODEX_CONTEXT.md)
3. [architecture/TARGET_FUNCTIONAL_ARCHITECTURE.md](architecture/TARGET_FUNCTIONAL_ARCHITECTURE.md)
   → [architecture/RUNTIME_ARCHITECTURE.md](architecture/RUNTIME_ARCHITECTURE.md)
4. [contracts/TF_CONVENTION.md](contracts/TF_CONVENTION.md)
   → [contracts/backend_contract_matrix.yaml](contracts/backend_contract_matrix.yaml)
5. [MAPPING_AND_LIO_POLICY.md](MAPPING_AND_LIO_POLICY.md)
   → [LIVOX_DATA_AND_RELOCALIZATION_PRIMER.md](LIVOX_DATA_AND_RELOCALIZATION_PRIMER.md)
   → [CURRENT_NAVIGATION_CAPABILITIES.md](CURRENT_NAVIGATION_CAPABILITIES.md)
6. [GLOBAL_RELOCALIZATION_BBS_GICP.md](GLOBAL_RELOCALIZATION_BBS_GICP.md)
   → [contracts/GLOBAL_RELOCALIZATION_SDK_CONTRACT.md](contracts/GLOBAL_RELOCALIZATION_SDK_CONTRACT.md)
7. [NAV2_MAP_POLICY.md](NAV2_MAP_POLICY.md)
   → [TERRAIN_MAP_DESIGN.md](TERRAIN_MAP_DESIGN.md)
   → [design/map_manager_architecture.md](design/map_manager_architecture.md)
8. [FIELD_MAP_FINALIZATION.md](FIELD_MAP_FINALIZATION.md)
   → [HMI_MAP_WORKFLOW.md](HMI_MAP_WORKFLOW.md)
9. [ACCEPTANCE.md](ACCEPTANCE.md)
   → [acceptance/PRE_ACCEPTANCE_GATE.md](acceptance/PRE_ACCEPTANCE_GATE.md)
   → [RVIZ_FIELD_ACCEPTANCE.md](RVIZ_FIELD_ACCEPTANCE.md)
10. [FIELD_SENSOR_BASELINE.md](FIELD_SENSOR_BASELINE.md)
    → [BOOTSTRAP_AND_ROSBAG_GATE.md](BOOTSTRAP_AND_ROSBAG_GATE.md)
    → [VIBRATION_AND_LI_INIT.md](VIBRATION_AND_LI_INIT.md)
11. [INTEGRATION_RUNTIME_V1.md](INTEGRATION_RUNTIME_V1.md)
    → [design/operator_mode_upgrade.md](design/operator_mode_upgrade.md)
    → [MAINLINE_POLICY.md](MAINLINE_POLICY.md)
12. 仅在追查历史、证据或缺口时阅读 `archive/`、R1.1 报告、P2.20/P2.21
    记录和两个 Map Manager audit。

此顺序避免把历史审计、目标设计或早期验收基线误读为当前运行时契约。