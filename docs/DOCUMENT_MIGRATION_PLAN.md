# Documentation Migration Plan

## Scope

This plan covers only Markdown and YAML files below `docs/`. It does not alter
source code, package layout, launch files, configuration, interfaces, map
assets, or runtime behavior. No deletion is authorized by this plan.

The proposed taxonomy is:

| Category | Meaning | Preferred directory |
|---|---|---|
| architecture | Structure, topology, and design decisions | `docs/architecture/` |
| contracts | Stable interfaces, policies, and compatibility rules | `docs/contracts/` |
| acceptance | Gates, procedures, validation, and test evidence | `docs/acceptance/` |
| archive | Historical or superseded records retained for traceability | `docs/archive/` |

The detailed file-by-file proposal is in [DOCUMENT_INDEX.md](DOCUMENT_INDEX.md).

## Current State

The branch already contains an in-progress documentation relocation. Several
old paths are marked deleted and corresponding files exist under the four
target directories as untracked files. This plan treats that state as input;
it does not restore, delete, or move any of those files.

There are also two files named `map_manager_audit.md`. Any future move or link
update must use the full relative path so the documents remain distinguishable.

## Migration Stages

### Stage 0: Inventory and freeze

1. Use [DOCUMENT_INDEX.md](DOCUMENT_INDEX.md) as the complete inventory.
2. Keep `CODEX_CONTEXT.md` and the four category directories available as the
   current branch context.
3. Do not remove a document solely because another document appears newer.

### Stage 1: Review classification

1. Confirm each proposed category with the document owner or branch author.
2. Resolve whether policy documents such as `MAINLINE_POLICY.md` and
   `POINTCLOUD_PIPELINE.md` are authoritative contracts or operational
   acceptance guidance.
3. Decide whether refactor evidence remains in `acceptance/` or is retained in
   `archive/` after the current milestone closes.

### Stage 2: Relocate active documents

1. Move only documents approved as active into their target category.
2. Preserve document content, history where practical, and filenames unless a
   collision requires a deliberate rename.
3. Update relative Markdown links and references in the same change.
4. Keep `CODEX_CONTEXT.md` and the index at the `docs/` root as navigation
   entry points.

### Stage 3: Relocate historical records

1. Move completed phase records, dated retrospectives, and superseded audits
   to `archive/`.
2. Add a short supersession note when an archived document has a clear active
   replacement.
3. Do not delete archived records.

### Stage 4: Verify and close

Run these checks after each relocation batch:

```bash
cd /home/yangxuan/ros2_ws/src/agt_navigation_v3
rg --files docs -g '*.md' -g '*.yaml' -g '*.yml' | sort
rg -n 'docs/(design|refactor|acceptance|contracts|architecture|archive)' docs \
  -g '*.md' -g '*.yaml' -g '*.yml'
git diff --check -- docs
```

The batch is complete only when:

- every Markdown/YAML file appears exactly once in the index;
- no Markdown link points at a path that no longer exists;
- YAML files still parse and retain their schema/content;
- no source, launch, configuration, interface, or map file changed;
- no deletion was performed unless separately and explicitly approved.

## Non-goals

- Do not modify ROS 2 source code or package metadata.
- Do not rename runtime topics, services, actions, frames, or parameters.
- Do not rewrite technical content as part of classification.
- Do not delete duplicates or “obsolete” documents during this pass.