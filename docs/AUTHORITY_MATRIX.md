# Documentation Authority Matrix

Use this table to route a question to its primary source. Supporting
documents may explain rationale or procedure, but they do not override the
primary authority listed here.

| Question | Primary authority | Supporting documents | Authority note |
|---|---|---|---|
| What is the current milestone and what decisions are frozen? | [CODEX_CONTEXT.md](CODEX_CONTEXT.md) | [MAINLINE_POLICY.md](MAINLINE_POLICY.md) | Current context; not a detailed runtime contract |
| What is the target package and functional topology? | [architecture/TARGET_FUNCTIONAL_ARCHITECTURE.md](architecture/TARGET_FUNCTIONAL_ARCHITECTURE.md) | [architecture/RUNTIME_ARCHITECTURE.md](architecture/RUNTIME_ARCHITECTURE.md) | Target structure is distinct from historical audits |
| What processes and high-bandwidth runtime edges exist? | [architecture/RUNTIME_ARCHITECTURE.md](architecture/RUNTIME_ARCHITECTURE.md) | [INTEGRATION_RUNTIME_V1.md](INTEGRATION_RUNTIME_V1.md) | Runtime topology and executor/DDS boundary |
| Who owns `map -> odom` and how are frames related? | [contracts/TF_CONVENTION.md](contracts/TF_CONVENTION.md) | [contracts/backend_contract_matrix.yaml](contracts/backend_contract_matrix.yaml) | LocalizationManager is the sole correction-TF owner |
| What is the formal relocalization frame contract? | [contracts/backend_contract_matrix.yaml](contracts/backend_contract_matrix.yaml) | [contracts/TF_CONVENTION.md](contracts/TF_CONVENTION.md) | Machine-readable `mapping_body` and `T_map_body` authority |
| What ROS/backend relocalization interface is supported? | [contracts/GLOBAL_RELOCALIZATION_SDK_CONTRACT.md](contracts/GLOBAL_RELOCALIZATION_SDK_CONTRACT.md) | [GLOBAL_RELOCALIZATION_BBS_GICP.md](GLOBAL_RELOCALIZATION_BBS_GICP.md) | Adapter and SDK boundary |
| How does global localization work? | [GLOBAL_RELOCALIZATION_BBS_GICP.md](GLOBAL_RELOCALIZATION_BBS_GICP.md) | [contracts/GLOBAL_RELOCALIZATION_SDK_CONTRACT.md](contracts/GLOBAL_RELOCALIZATION_SDK_CONTRACT.md), [LIVOX_DATA_AND_RELOCALIZATION_PRIMER.md](LIVOX_DATA_AND_RELOCALIZATION_PRIMER.md) | Current design is 3D-BBS coarse plus small_gicp fine |
| What Livox message format and conversion rules apply? | [LIVOX_DATA_AND_RELOCALIZATION_PRIMER.md](LIVOX_DATA_AND_RELOCALIZATION_PRIMER.md) | [POINTCLOUD_PIPELINE.md](POINTCLOUD_PIPELINE.md) | Preserve raw Livox timing for the LIO path |
| Which LIO is used for mapping and navigation? | [MAPPING_AND_LIO_POLICY.md](MAPPING_AND_LIO_POLICY.md) | [ODOMETRY_MAPPING_SELECTION.md](ODOMETRY_MAPPING_SELECTION.md) | Policy authority; selection document is rationale |
| What point-cloud filtering order is allowed? | [POINTCLOUD_PIPELINE.md](POINTCLOUD_PIPELINE.md) | [MAPPING_AND_LIO_POLICY.md](MAPPING_AND_LIO_POLICY.md) | Do not insert generic filtering before the LIO front-end without a contract change |
| What is the current implemented navigation capability? | [CURRENT_NAVIGATION_CAPABILITIES.md](CURRENT_NAVIGATION_CAPABILITIES.md) | [INTEGRATION_RUNTIME_V1.md](INTEGRATION_RUNTIME_V1.md) | Prefer this over aspirational design documents |
| What is the MapManager lifecycle and interface model? | [design/map_manager_architecture.md](design/map_manager_architecture.md) | [HMI_MAP_WORKFLOW.md](HMI_MAP_WORKFLOW.md), [MAP_MANAGEMENT.md](MAP_MANAGEMENT.md) | Map Manager design authority; audits are evidence only |
| What is the current MapManager implementation status? | [map_manager_audit.md](map_manager_audit.md) | [design/map_manager_audit.md](design/map_manager_audit.md), [CURRENT_NAVIGATION_CAPABILITIES.md](CURRENT_NAVIGATION_CAPABILITIES.md) | Audit snapshot, not a normative contract |
| How is a mapping run finalized into a field map package? | [FIELD_MAP_FINALIZATION.md](FIELD_MAP_FINALIZATION.md) | [HMI_MAP_WORKFLOW.md](HMI_MAP_WORKFLOW.md) | Field finalization procedure |
| How are maps edited and navigation targets managed in HMI? | [HMI_MAP_WORKFLOW.md](HMI_MAP_WORKFLOW.md) | [FIELD_MAP_FINALIZATION.md](FIELD_MAP_FINALIZATION.md) | EditSession and active-map workflow |
| How is the 3D map converted to a Nav2 map? | [MAP_CONVERTER_DESIGN.md](MAP_CONVERTER_DESIGN.md) | [architecture/map_pipeline_design.md](architecture/map_pipeline_design.md), [TERRAIN_MAP_DESIGN.md](TERRAIN_MAP_DESIGN.md) | Converter is the baseline/fallback design |
| What is the preferred terrain-aware map architecture? | [TERRAIN_MAP_DESIGN.md](TERRAIN_MAP_DESIGN.md) | [MAP_CONVERTER_DESIGN.md](MAP_CONVERTER_DESIGN.md) | Terrain design is preferred; converter remains fallback |
| Which map is authoritative for Nav2 and localization? | [NAV2_MAP_POLICY.md](NAV2_MAP_POLICY.md) | [FIELD_MAP_FINALIZATION.md](FIELD_MAP_FINALIZATION.md) | Separates localization assets from Nav2 map ownership |
| What is the generic map-generation pipeline? | [architecture/map_pipeline_design.md](architecture/map_pipeline_design.md) | [TERRAIN_MAP_DESIGN.md](TERRAIN_MAP_DESIGN.md), [MAP_CONVERTER_DESIGN.md](MAP_CONVERTER_DESIGN.md) | Background pipeline model; detailed policy is elsewhere |
| What are the software/offline/Gazebo acceptance gates? | [ACCEPTANCE.md](ACCEPTANCE.md) | [refactor/DEFAULT_TEST_WORKFLOW.md](refactor/DEFAULT_TEST_WORKFLOW.md) | Current software acceptance authority |
| What gates must pass before field acceptance? | [acceptance/PRE_ACCEPTANCE_GATE.md](acceptance/PRE_ACCEPTANCE_GATE.md) | [BOOTSTRAP_AND_ROSBAG_GATE.md](BOOTSTRAP_AND_ROSBAG_GATE.md) | Ordered pre-field gate |
| What is the current RViz field procedure? | [RVIZ_FIELD_ACCEPTANCE.md](RVIZ_FIELD_ACCEPTANCE.md) | [acceptance/FIELD_ACCEPTANCE_V1.md](acceptance/FIELD_ACCEPTANCE_V1.md) | Current staged field path |
| What are the formal V1 field targets? | [acceptance/FIELD_ACCEPTANCE_V1.md](acceptance/FIELD_ACCEPTANCE_V1.md) | [RVIZ_FIELD_ACCEPTANCE.md](RVIZ_FIELD_ACCEPTANCE.md) | Verify its older commit baseline before treating it as current |
| What hardware and sensor preflight is required? | [FIELD_SENSOR_BASELINE.md](FIELD_SENSOR_BASELINE.md) | [VIBRATION_AND_LI_INIT.md](VIBRATION_AND_LI_INIT.md) | MID360/IMU and calibration baseline |
| How are bootstrap and rosbag-to-field reproducibility gates run? | [BOOTSTRAP_AND_ROSBAG_GATE.md](BOOTSTRAP_AND_ROSBAG_GATE.md) | [MIGRATION.md](MIGRATION.md) | Field-readiness and restore procedure |
| How are vibration and LI-Init issues diagnosed? | [VIBRATION_AND_LI_INIT.md](VIBRATION_AND_LI_INIT.md) | [FIELD_SENSOR_BASELINE.md](FIELD_SENSOR_BASELINE.md) | Diagnostic procedure, not a runtime architecture contract |
| What operator workflow is currently designed? | [design/operator_mode_upgrade.md](design/operator_mode_upgrade.md) | [HMI_MAP_WORKFLOW.md](HMI_MAP_WORKFLOW.md) | Workflow design and task record |
| Where should historical or superseded documents be routed? | [DEPRECATED.md](DEPRECATED.md) | [DOCUMENT_REVIEW_REPORT.md](DOCUMENT_REVIEW_REPORT.md) | Archive is evidence, not current authority |

## Conflict Rule

When documents disagree, prefer the source in this order:

1. Explicit current contract or machine-readable matrix.
2. Current runtime capability or acceptance procedure.
3. Current architecture/design document.
4. Historical audit, retrospective, or phase report.

Record unresolved conflicts as review items instead of silently choosing a
historical document.