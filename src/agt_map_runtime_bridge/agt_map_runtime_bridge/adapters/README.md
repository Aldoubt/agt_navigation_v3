# Runtime Adapters

Runtime Bridge keeps backend-specific logic isolated.

## Planned adapters

## Nav2 Adapter

Responsibilities:

- load navigation map yaml
- trigger lifecycle transitions
- report navigation readiness

## Localization Adapter

Responsibilities:

- load localization point cloud package
- update global localization backend
- report localization readiness

## Rule

Adapters must consume the same:

- map_id
- version
- generation

A mismatch must block navigation startup.
