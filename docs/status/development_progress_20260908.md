# Development Progress 2026-09-08

## Current Phase

Map Manager architecture has entered runtime integration phase.

## Completed

- Map Package concept established
- Map Manager lifecycle design completed
- Immutable released map package model completed
- HMI edit session workflow designed
- Map status interface defined

## Current Architecture Decision

Localization map and navigation map are managed as one versioned package.

Runtime modules consume the same generation identifier.

## Next Implementation Tasks

Priority 1:

- Create `agt_map_runtime_bridge`
- Connect `/agt/map/status`
- Add Nav2 map reload adapter
- Add global localization map reload adapter

Priority 2:

- Add runtime acceptance tests
- Verify restart recovery
- Verify map version consistency

## Development Notes

Do not extend HMI features before runtime map switching is complete.

Do not add more terrain algorithms before map pipeline interfaces are stabilized.

The next milestone is:

Map Manager -> Runtime Bridge -> Nav2 + Localization closed loop.
