# Documentation Index

This index covers Markdown and YAML files below `docs/`. It describes the
current working tree and the proposed destination category. It does not imply
that files have been moved or deleted.

## Categories

- **architecture**: system structure, runtime topology, design decisions, and
  operational architecture.
- **contracts**: stable interfaces, frame/topic semantics, policies, and
  compatibility rules.
- **acceptance**: gates, test workflows, validation reports, and field
  procedures.
- **archive**: historical audits, superseded experiments, and completed phase
  records retained for traceability.

## Current Context

| Document | Proposed category | Current role |
|---|---|---|
| [CODEX_CONTEXT.md](CODEX_CONTEXT.md) | contracts | Current milestone, frozen decisions, and next milestone |

## Index Documents

| Document | Proposed category | Current role |
|---|---|---|
| [DOCUMENT_INDEX.md](DOCUMENT_INDEX.md) | contracts | This documentation inventory |
| [DOCUMENT_MIGRATION_PLAN.md](DOCUMENT_MIGRATION_PLAN.md) | contracts | Classification and staged migration procedure |

## Architecture

| Document | Proposed category | Current role |
|---|---|---|
| [ARCHITECTURE_BASELINE.md](ARCHITECTURE_BASELINE.md) | architecture | System architecture baseline |
| [ADVANCED_DESIGN_V2.md](ADVANCED_DESIGN_V2.md) | architecture | Terrain and map-manager design decisions |
| [architecture/TARGET_FUNCTIONAL_ARCHITECTURE.md](architecture/TARGET_FUNCTIONAL_ARCHITECTURE.md) | architecture | Target functional topology |
| [architecture/RUNTIME_ARCHITECTURE.md](architecture/RUNTIME_ARCHITECTURE.md) | architecture | Process-level runtime topology |
| [architecture/map_pipeline_design.md](architecture/map_pipeline_design.md) | architecture | Map generation pipeline |
| [design/map_manager_architecture.md](design/map_manager_architecture.md) | architecture | Map manager architecture baseline |
| [design/operator_mode_upgrade.md](design/operator_mode_upgrade.md) | architecture | Operator-mode workflow design and task log |
| [FIELD_MAP_FINALIZATION.md](FIELD_MAP_FINALIZATION.md) | architecture | Map finalization and HMI workflow |
| [HMI_MAP_WORKFLOW.md](HMI_MAP_WORKFLOW.md) | architecture | HMI map editing and navigation workflow |
| [IMPLEMENTATION_ROADMAP_v1.md](IMPLEMENTATION_ROADMAP_v1.md) | architecture | Implementation phases and frozen rules |
| [MAP_CONVERTER_DESIGN.md](MAP_CONVERTER_DESIGN.md) | architecture | 3D-to-navigation map converter |
| [MAP_MANAGEMENT.md](MAP_MANAGEMENT.md) | architecture | Map package and lifecycle design |
| [MIGRATION.md](MIGRATION.md) | architecture | Source-tree migration and restoration guide |
| [NAV2_MAP_POLICY.md](NAV2_MAP_POLICY.md) | architecture | Nav2 map and costmap design |
| [ODOMETRY_MAPPING_SELECTION.md](ODOMETRY_MAPPING_SELECTION.md) | architecture | Mapping and odometry selection |
| [TERRAIN_MAP_DESIGN.md](TERRAIN_MAP_DESIGN.md) | architecture | Terrain-aware map generation |

## Contracts

| Document | Proposed category | Current role |
|---|---|---|
| [contracts/TF_CONVENTION.md](contracts/TF_CONVENTION.md) | contracts | TF ownership and frame convention |
| [contracts/GLOBAL_RELOCALIZATION_SDK_CONTRACT.md](contracts/GLOBAL_RELOCALIZATION_SDK_CONTRACT.md) | contracts | Relocalization SDK boundary |
| [contracts/backend_contract_matrix.yaml](contracts/backend_contract_matrix.yaml) | contracts | Machine-readable backend frame contract |
| [design/candidate_bbs_mapping_body_experiment.md](design/candidate_bbs_mapping_body_experiment.md) | contracts | Candidate `mapping_body` contract experiment |
| [GLOBAL_RELOCALIZATION_BBS_GICP.md](GLOBAL_RELOCALIZATION_BBS_GICP.md) | contracts | Global relocalization backend contract and rationale |
| [LIVOX_DATA_AND_RELOCALIZATION_PRIMER.md](LIVOX_DATA_AND_RELOCALIZATION_PRIMER.md) | contracts | Livox format and relocalization data policy |
| [MAPPING_AND_LIO_POLICY.md](MAPPING_AND_LIO_POLICY.md) | contracts | Mapping and navigation LIO policy |
| [MAINLINE_POLICY.md](MAINLINE_POLICY.md) | contracts | Mainline development policy |
| [POINTCLOUD_PIPELINE.md](POINTCLOUD_PIPELINE.md) | contracts | Point-cloud processing order and invariants |

## Acceptance

| Document | Proposed category | Current role |
|---|---|---|
| [ACCEPTANCE.md](ACCEPTANCE.md) | acceptance | Offline, simulation, and V1 acceptance gates |
| [acceptance/PRE_ACCEPTANCE_GATE.md](acceptance/PRE_ACCEPTANCE_GATE.md) | acceptance | Pre-acceptance gate checklist |
| [acceptance/FIELD_ACCEPTANCE_V1.md](acceptance/FIELD_ACCEPTANCE_V1.md) | acceptance | Field acceptance workflow |
| [BOOTSTRAP_AND_ROSBAG_GATE.md](BOOTSTRAP_AND_ROSBAG_GATE.md) | acceptance | Bootstrap and rosbag-to-field gate |
| [CURRENT_NAVIGATION_CAPABILITIES.md](CURRENT_NAVIGATION_CAPABILITIES.md) | acceptance | Validated runtime capability snapshot |
| [FIELD_SENSOR_BASELINE.md](FIELD_SENSOR_BASELINE.md) | acceptance | Hardware and sensor baseline |
| [INTEGRATION_RUNTIME_V1.md](INTEGRATION_RUNTIME_V1.md) | acceptance | Integration runtime validation boundary |
| [RVIZ_FIELD_ACCEPTANCE.md](RVIZ_FIELD_ACCEPTANCE.md) | acceptance | Current RViz field acceptance procedure |
| [VIBRATION_AND_LI_INIT.md](VIBRATION_AND_LI_INIT.md) | acceptance | Vibration and LI-Init diagnostic procedure |
| [refactor/DEFAULT_TEST_WORKFLOW.md](refactor/DEFAULT_TEST_WORKFLOW.md) | acceptance | Default test workflow |
| [refactor/R1_1_RUNTIME_VALIDATION.md](refactor/R1_1_RUNTIME_VALIDATION.md) | acceptance | Runtime validation record |
| [refactor/R1_1_SIM_INTERFACE_AUDIT.md](refactor/R1_1_SIM_INTERFACE_AUDIT.md) | acceptance | Simulation interface audit |
| [refactor/R1_1_SIMULATION_AUDIT.md](refactor/R1_1_SIMULATION_AUDIT.md) | acceptance | Simulation runtime audit |
| [refactor/R1_1_SIMULATION_VALIDATION.md](refactor/R1_1_SIMULATION_VALIDATION.md) | acceptance | Simulation validation record |
| [refactor/TEST_REPORT.md](refactor/TEST_REPORT.md) | acceptance | Refactor test report |

## Archive

| Document | Proposed category | Current role |
|---|---|---|
| [archive/CURRENT_ARCHITECTURE_AUDIT.md](archive/CURRENT_ARCHITECTURE_AUDIT.md) | archive | Completed architecture audit |
| [archive/P2_20_MAPPING_BODY_ACCEPTANCE_CHECKLIST.md](archive/P2_20_MAPPING_BODY_ACCEPTANCE_CHECKLIST.md) | archive | Completed P2.20 acceptance checklist |
| [archive/p2_20_mapping_body_frame_contract.md](archive/p2_20_mapping_body_frame_contract.md) | archive | Superseded P2.20 frame contract |
| [archive/P2_21_GLOBAL_LOCALIZATION_FRAME_CONTRACT_CHECKLIST.md](archive/P2_21_GLOBAL_LOCALIZATION_FRAME_CONTRACT_CHECKLIST.md) | archive | Completed P2.21 checklist |
| [archive/P2_21_GLOBAL_LOCALIZATION_FRAME_CONTRACT_FREEZE.md](archive/P2_21_GLOBAL_LOCALIZATION_FRAME_CONTRACT_FREEZE.md) | archive | Historical P2.21 freeze record |
| [archive/ROSBAG_ANALYSIS.md](archive/ROSBAG_ANALYSIS.md) | archive | Historical rosbag analysis |
| [archive/ROSBAG_RELOCALIZATION_BENCHMARK.md](archive/ROSBAG_RELOCALIZATION_BENCHMARK.md) | archive | Historical relocalization benchmark |
| [design/map_manager_audit.md](design/map_manager_audit.md) | archive | Design-stage map manager audit |
| [map_manager_audit.md](map_manager_audit.md) | archive | Earlier map manager audit snapshot |
| [RELOCALIZATION_DEBUGGING_RETROSPECTIVE_2026-09-05.md](RELOCALIZATION_DEBUGGING_RETROSPECTIVE_2026-09-05.md) | archive | Dated debugging retrospective |
| [refactor/build_baseline_report.md](refactor/build_baseline_report.md) | archive | Refactor baseline record |
| [refactor/FUNCTIONAL_DOMAIN_COMPLETION.md](refactor/FUNCTIONAL_DOMAIN_COMPLETION.md) | archive | Refactor completion record |
| [refactor/MIGRATION_PLAN.md](refactor/MIGRATION_PLAN.md) | archive | Earlier source-tree migration plan |
| [refactor/MIGRATION_REPORT.md](refactor/MIGRATION_REPORT.md) | archive | Earlier source-tree migration report |

## Migration Notes

- The four category directories already present in the working tree are the
  preferred final locations.
- Root-level files and files under `design/` or `refactor/` remain in place
  until the staged migration is reviewed and links are updated.
- The two `map_manager_audit.md` files are intentionally indexed by full path.
- `docs/CODEX_CONTEXT.md` is a current context document and is indexed as a
  contract/context artifact rather than an architecture specification.
- This index itself is the entry point for the documentation set.