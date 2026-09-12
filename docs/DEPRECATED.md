# Deprecated and Archived Documentation

The documents below are retained for traceability but are not current
architecture or runtime authorities. The replacement column identifies where a
new developer or AI agent should look first.

| ARCHIVE document | Replacement | Reason |
|---|---|---|
| [archive/CURRENT_ARCHITECTURE_AUDIT.md](archive/CURRENT_ARCHITECTURE_AUDIT.md) | [architecture/TARGET_FUNCTIONAL_ARCHITECTURE.md](architecture/TARGET_FUNCTIONAL_ARCHITECTURE.md), [architecture/RUNTIME_ARCHITECTURE.md](architecture/RUNTIME_ARCHITECTURE.md) | Historical package/domain audit; current structure is described by the architecture baselines |
| [archive/p2_20_mapping_body_frame_contract.md](archive/p2_20_mapping_body_frame_contract.md) | [contracts/backend_contract_matrix.yaml](contracts/backend_contract_matrix.yaml) | P2.20 frame contract was superseded by the P2.21 freeze and current matrix |
| [archive/P2_20_MAPPING_BODY_ACCEPTANCE_CHECKLIST.md](archive/P2_20_MAPPING_BODY_ACCEPTANCE_CHECKLIST.md) | [contracts/backend_contract_matrix.yaml](contracts/backend_contract_matrix.yaml), [acceptance/PRE_ACCEPTANCE_GATE.md](acceptance/PRE_ACCEPTANCE_GATE.md) | Completed P2.20 checklist; current contract and gate are maintained separately |
| [archive/P2_21_GLOBAL_LOCALIZATION_FRAME_CONTRACT_FREEZE.md](archive/P2_21_GLOBAL_LOCALIZATION_FRAME_CONTRACT_FREEZE.md) | [contracts/backend_contract_matrix.yaml](contracts/backend_contract_matrix.yaml), [contracts/TF_CONVENTION.md](contracts/TF_CONVENTION.md) | Freeze record is evidence; the machine-readable and TF contracts are current |
| [archive/P2_21_GLOBAL_LOCALIZATION_FRAME_CONTRACT_CHECKLIST.md](archive/P2_21_GLOBAL_LOCALIZATION_FRAME_CONTRACT_CHECKLIST.md) | [contracts/backend_contract_matrix.yaml](contracts/backend_contract_matrix.yaml), [acceptance/PRE_ACCEPTANCE_GATE.md](acceptance/PRE_ACCEPTANCE_GATE.md) | Completed phase checklist, not a live specification |
| [archive/ROSBAG_ANALYSIS.md](archive/ROSBAG_ANALYSIS.md) | [BOOTSTRAP_AND_ROSBAG_GATE.md](BOOTSTRAP_AND_ROSBAG_GATE.md) | Historical bag analysis replaced by the current reproducibility gate |
| [archive/ROSBAG_RELOCALIZATION_BENCHMARK.md](archive/ROSBAG_RELOCALIZATION_BENCHMARK.md) | [GLOBAL_RELOCALIZATION_BBS_GICP.md](GLOBAL_RELOCALIZATION_BBS_GICP.md), [ACCEPTANCE.md](ACCEPTANCE.md) | Historical benchmark method; current backend design and acceptance gates are authoritative |
| [design/map_manager_audit.md](design/map_manager_audit.md) | [design/map_manager_architecture.md](design/map_manager_architecture.md), [map_manager_audit.md](map_manager_audit.md) | Design-stage audit; later audit and architecture document carry the active context |
| [map_manager_audit.md](map_manager_audit.md) | [design/map_manager_architecture.md](design/map_manager_architecture.md), [CURRENT_NAVIGATION_CAPABILITIES.md](CURRENT_NAVIGATION_CAPABILITIES.md) | Implementation snapshot and gap evidence; not a normative MapManager contract |
| [RELOCALIZATION_DEBUGGING_RETROSPECTIVE_2026-09-05.md](RELOCALIZATION_DEBUGGING_RETROSPECTIVE_2026-09-05.md) | [GLOBAL_RELOCALIZATION_BBS_GICP.md](GLOBAL_RELOCALIZATION_BBS_GICP.md), [acceptance/PRE_ACCEPTANCE_GATE.md](acceptance/PRE_ACCEPTANCE_GATE.md) | Dated debugging retrospective; current design and gate documents supersede its operational guidance |
| [refactor/R1_1_RUNTIME_VALIDATION.md](refactor/R1_1_RUNTIME_VALIDATION.md) | [ACCEPTANCE.md](ACCEPTANCE.md), [INTEGRATION_RUNTIME_V1.md](INTEGRATION_RUNTIME_V1.md) | R1.1 validation record is historical evidence |
| [refactor/R1_1_SIM_INTERFACE_AUDIT.md](refactor/R1_1_SIM_INTERFACE_AUDIT.md) | [architecture/RUNTIME_ARCHITECTURE.md](architecture/RUNTIME_ARCHITECTURE.md), [INTEGRATION_RUNTIME_V1.md](INTEGRATION_RUNTIME_V1.md) | Historical simulation interface audit |
| [refactor/R1_1_SIMULATION_AUDIT.md](refactor/R1_1_SIMULATION_AUDIT.md) | [ACCEPTANCE.md](ACCEPTANCE.md), [CURRENT_NAVIGATION_CAPABILITIES.md](CURRENT_NAVIGATION_CAPABILITIES.md) | Historical simulation capability/gap audit |
| [refactor/R1_1_SIMULATION_VALIDATION.md](refactor/R1_1_SIMULATION_VALIDATION.md) | [ACCEPTANCE.md](ACCEPTANCE.md) | Historical validation result; current gates define acceptance |
| [refactor/TEST_REPORT.md](refactor/TEST_REPORT.md) | [ACCEPTANCE.md](ACCEPTANCE.md), [MAINLINE_POLICY.md](MAINLINE_POLICY.md) | Refactor test evidence, not a current pass/fail authority |
| [refactor/build_baseline_report.md](refactor/build_baseline_report.md) | [MAINLINE_POLICY.md](MAINLINE_POLICY.md), [ACCEPTANCE.md](ACCEPTANCE.md) | Historical R1.1 build baseline |
| [refactor/FUNCTIONAL_DOMAIN_COMPLETION.md](refactor/FUNCTIONAL_DOMAIN_COMPLETION.md) | [architecture/TARGET_FUNCTIONAL_ARCHITECTURE.md](architecture/TARGET_FUNCTIONAL_ARCHITECTURE.md), [DOCUMENT_MIGRATION_PLAN.md](DOCUMENT_MIGRATION_PLAN.md) | Completed migration record; current target and documentation plan are separate |
| [refactor/MIGRATION_PLAN.md](refactor/MIGRATION_PLAN.md) | [DOCUMENT_MIGRATION_PLAN.md](DOCUMENT_MIGRATION_PLAN.md) | Earlier source-tree migration plan; current task concerns documentation only |
| [refactor/MIGRATION_REPORT.md](refactor/MIGRATION_REPORT.md) | [DOCUMENT_REVIEW_REPORT.md](DOCUMENT_REVIEW_REPORT.md), [DOCUMENT_MIGRATION_PLAN.md](DOCUMENT_MIGRATION_PLAN.md) | Earlier migration status report; current document authority is recorded by the new audit |

Archived documents may still be read when investigating historical decisions,
regressions, or validation evidence. They must not be used alone to infer
current runtime behavior.