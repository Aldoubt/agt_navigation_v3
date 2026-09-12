# AGT Navigation V3 Context

This document is the first technical context source for an AI agent working
on AGT Navigation V3. It describes the current frozen architecture and points
to the detailed authority documents. It is not a replacement for the
contracts in `docs/contracts/`.

## 1 Project Overview

AGT Navigation V3 is a ROS 2 Humble navigation stack for a tracked robot. Its
field hardware baseline includes a Livox MID360 LiDAR/IMU and a Bunker-class
tracked chassis. The system combines:

- continuous local odometry from the LIO pipeline;
- 3D global relocalization against prepared map assets;
- a single localization correction owner;
- Nav2 for planning and autonomous navigation.

This is not an ordinary SLAM demo. The current product shape is:

```text
3D localization + continuous odometry + navigation
```

The current milestone is `v0.3.0 localization contract freeze`. The next
milestone is P3: runtime acceptance, Nav2 field testing, and operator workflow.

## Current Development Branch

Documentation:
`refactor/docs-architecture`

Latest frozen milestone:
`v0.3.0 localization contract freeze`

Next engineering focus:
P3 runtime acceptance and field workflow.

## 2 Current Frozen Architecture

The system-level flow is:

```text
Sensor
    |
    v
LIO / odometry
    |
    v
Global Localization
    |
    v
LocalizationManager
    |
    v
map -> odom
    |
    v
Nav2
```

More concretely:

- The MID360 publishes the sensor data used by mapping, odometry, and
    relocalization branches.
- FAST-LIO2/Batch-LIO provides continuous local motion estimation. Livox raw
    point timing must be preserved on the LIO path.
- Global localization uses a coarse 3D-BBS search followed by local
    `small_gicp` refinement.
- `LocalizationManager` is the only owner allowed to publish the global
    correction `map -> odom`.
- `MapTracker` consumes the resulting localization state and supports the
    navigation-side tracking path.
- Nav2 consumes the canonical navigation pose and local odometry to plan and
    control the tracked robot.

The architecture therefore separates global correction, continuous local
odometry, and navigation. Do not collapse these into a generic SLAM pipeline.

Detailed structure: [architecture/TARGET_FUNCTIONAL_ARCHITECTURE.md](architecture/TARGET_FUNCTIONAL_ARCHITECTURE.md)
and [architecture/RUNTIME_ARCHITECTURE.md](architecture/RUNTIME_ARCHITECTURE.md).

## 3 Frame Contract (Highest Priority)

The frame contract is frozen. Read
[contracts/TF_CONVENTION.md](contracts/TF_CONVENTION.md) and
[contracts/backend_contract_matrix.yaml](contracts/backend_contract_matrix.yaml)
before changing localization, mapping, or pose publication code.

The formal mapping contract is:

```text
poses.txt:
    T_map_body

query cloud:
    mapping_body

native localization result:
    T_map_body

ROS output:
    T_map_base
```

The only body-to-base conversion is:

```text
T_map_base = T_map_body * T_body_base
```

The current machine-readable matrix names the ROS publication boundary
`T_map_base_link`; treat that as the concrete `T_map_base` output in the
equation above. The semantic rule is unchanged: native localization remains
in `T_map_body`, and conversion occurs exactly once at the publication
boundary.

### Frame prohibitions

- Do not mix `body` and `base` semantics in one backend invocation.
- Do not apply the body-to-base extrinsic conversion twice.
- Do not introduce a frame without declaring its pose and point-cloud
    semantics.
- Do not silently change `mapping_body` to `base_link` compatibility mode.
- Do not change the frozen frame contract to compensate for a bad map, stale
    query cloud, or incorrect calibration.

The current contract also requires the relocalization query frame mode and BBS
query frame mode to agree. Mixed modes must be rejected before the backend.

## 4 Repository Structure

The main runtime domains are:

- `navigation/`: localization, state estimation, Nav2, and navigation runtime
    packages.
- `bringup/`: launch and operator-facing system startup composition.
- `map_data_manager/`: map package lifecycle, active-map state, and map-related
    runtime coordination.
- `docs/`: architecture, contracts, acceptance procedures, and historical
    evidence. This file is the AI context entry point.

Other repository domains such as `sensor/`, `mapping/`, `cleaning/`,
`interfaces/`, and `tests/` support these runtime boundaries. Preserve package
names, installed launch names, topic names, service names, action names, and
TF contracts unless an explicit contract change is approved.

## 5 Current Status

### Completed or frozen

- Documentation authority structure and AI reading path are defined.
- The global localization frame contract is frozen around `mapping_body`.
- The body-to-base publication boundary is explicitly defined.
- The BBS plus `small_gicp` global-localization design is the current backend
    direction.
- `LocalizationManager` remains the sole global correction authority.
- MapTracker validation evidence records `192/192 OK` for the relevant
    `mapping_body` path.
- Current LIO, Livox data, point-cloud ordering, and Nav2 map ownership rules
    have dedicated authority documents.

### Known limitations and open status

- Real field acceptance remains open even where offline or Gazebo gates have
    passed; consult [acceptance/PRE_ACCEPTANCE_GATE.md](acceptance/PRE_ACCEPTANCE_GATE.md)
    and [RVIZ_FIELD_ACCEPTANCE.md](RVIZ_FIELD_ACCEPTANCE.md).
- `FIELD_ACCEPTANCE_V1.md` uses an older commit baseline and requires status
    review before being treated as the current field state.
- LIO latency and end-to-end runtime timing remain areas for measurement and
    optimization; do not change the raw timing contract as a shortcut.
- Dynamic-map behavior is not the current frozen product contract.
- MapManager structure exists, but generation/orchestration completeness must
    be checked against the current audit and capability documents.
- Planned terrain, RTK, recovery, and lifecycle features must not be reported
    as implemented without current capability or acceptance evidence.

## 6 Development Rules for AI Agent

Before modifying code:

1. Query [AUTHORITY_MATRIX.md](AUTHORITY_MATRIX.md) for the relevant question.
2. Read the corresponding contract in `docs/contracts/`.
3. Find existing regression tests, acceptance gates, and nearby call sites.
4. State the affected contract and a minimal validation check.
5. Propose the change before editing when the contract or ownership boundary is
     involved.

Do not:

- use `archive/` documents as current authority;
- modify the frozen frame contract, GICP behavior, BBS search, map assets, or
    `LocalizationManager` without explicit approval;
- mix `mapping_body` and `base_link` semantics;
- remove the measured-stop gate before camera capture;
- bypass tracked-chassis manual/remote safety arbitration;
- place generic point-cloud filtering before the LIO front-end without an
    approved contract change;
- delete historical experiments or validation evidence;
- infer implemented behavior from aspirational design documents;
- change source, launch, parameter, or interface files as part of a docs-only
    organization task.

### Experimental Changes

Any algorithm or capability experiment must:

- use an isolated branch;
- keep default behavior unchanged;
- provide acceptance evidence;
- update the authority documents after the experiment is frozen.

This applies to MapManager, dynamic maps, terrain navigation, RTK, camera
capabilities, and similar future work. Experiments must not silently become
the default architecture or contract.

## 7 Reading Order

For a new AI agent or developer, use this order:

1. `CODEX_CONTEXT.md` — this current context and frozen constraints.
2. [AUTHORITY_MATRIX.md](AUTHORITY_MATRIX.md) — question-to-authority routing.
3. [architecture/](architecture/) — target functional and runtime structure.
4. [contracts/](contracts/) — TF, frame, backend, and interface contracts.
5. Read the specific module README or package documentation for the code being
     changed.
6. Read [acceptance/](acceptance/) before claiming a runtime or field result.
7. Read [DEPRECATED.md](DEPRECATED.md) and [archive/](archive/) only for
     historical context, regression analysis, or evidence provenance.

The human-oriented entry point is [README.md](README.md). The complete file
inventory is [DOCUMENT_INDEX.md](DOCUMENT_INDEX.md), and the relationship audit
is [DOCUMENT_REVIEW_REPORT.md](DOCUMENT_REVIEW_REPORT.md).