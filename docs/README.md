# AGT Navigation V3 Documentation

This is the single entry point for developers and AI agents reading the
documentation under `docs/`.

AGT Navigation V3 is a ROS 2 Humble navigation stack for a tracked robot with
Livox MID360 sensing, continuous LIO odometry, 3D global relocalization, and
Nav2 navigation. The current system is a localization-plus-navigation stack,
not a generic SLAM demo.

## Start Here

Read in this order:

1. [CODEX_CONTEXT.md](CODEX_CONTEXT.md) for the current milestone, frozen
   decisions, and next milestone.
2. [DOCUMENT_INDEX.md](DOCUMENT_INDEX.md) for the complete Markdown/YAML
   inventory and category map.
3. [AUTHORITY_MATRIX.md](AUTHORITY_MATRIX.md) to find the authoritative
   document for a technical question.
4. [DOCUMENT_REVIEW_REPORT.md](DOCUMENT_REVIEW_REPORT.md) for duplicate,
   supersession, and status-conflict analysis.
5. [DOCUMENT_MIGRATION_PLAN.md](DOCUMENT_MIGRATION_PLAN.md) for documentation
   migration rules. It is a plan only; it does not authorize deletion.

Do not start with `archive/` or historical `refactor/` reports unless the task
is specifically about history, validation evidence, or a known regression.

## Documentation Categories

### Architecture

System structure, runtime topology, map workflows, and design decisions.

See [architecture/](architecture/) and the architecture entries in the
[document index](DOCUMENT_INDEX.md).

### Contracts

Stable frames, topics, data formats, ownership boundaries, policies, and
compatibility rules.

See [contracts/](contracts/) and [AUTHORITY_MATRIX.md](AUTHORITY_MATRIX.md).
When a contract question conflicts with a narrative document, prefer the
machine-readable or explicitly named contract source in the matrix.

### Acceptance

Build, replay, simulation, hardware preflight, field, and runtime acceptance
gates.

See [acceptance/](acceptance/) and the current field procedures linked from the
[authority matrix](AUTHORITY_MATRIX.md).

### Archive

Historical audits, superseded contracts, completed phase records, and old test
evidence. Archived documents are retained for traceability and are not current
runtime specifications.

See [archive/](archive/) and [DEPRECATED.md](DEPRECATED.md) for replacements.

## Rules for AI Agents

- Treat [AUTHORITY_MATRIX.md](AUTHORITY_MATRIX.md) as the question-to-source
  routing table.
- Treat [CODEX_CONTEXT.md](CODEX_CONTEXT.md) as the current milestone context,
  not as a replacement for detailed contracts.
- Separate planned architecture from implemented capability; check
  [CURRENT_NAVIGATION_CAPABILITIES.md](CURRENT_NAVIGATION_CAPABILITIES.md) when
  a document describes future behavior.
- Treat `map -> odom` ownership, Livox timing, relocalization frame semantics,
  and the measured-stop gate as protected constraints unless an explicit
  current contract says otherwise.
- Do not use archived material as evidence of current behavior without checking
  its replacement and date.
- Do not infer source-code changes from a documentation migration proposal.

## Contribution Flow

For AI-assisted development:

1. Read [CODEX_CONTEXT.md](CODEX_CONTEXT.md).
2. Identify the authority document in [AUTHORITY_MATRIX.md](AUTHORITY_MATRIX.md).
3. Modify the minimal scope.
4. Add or update a regression test.
5. Update acceptance evidence when behavior changes.

## Current Architecture Reading Path

`CODEX_CONTEXT`
-> `TARGET_FUNCTIONAL_ARCHITECTURE`
-> `RUNTIME_ARCHITECTURE`
-> `TF_CONVENTION` + `backend_contract_matrix.yaml`
-> `MAPPING_AND_LIO_POLICY`
-> `GLOBAL_RELOCALIZATION_BBS_GICP`
-> `NAV2_MAP_POLICY` + map documents
-> `ACCEPTANCE` + field acceptance procedures

For the full path and rationale, see [DOCUMENT_REVIEW_REPORT.md](DOCUMENT_REVIEW_REPORT.md).