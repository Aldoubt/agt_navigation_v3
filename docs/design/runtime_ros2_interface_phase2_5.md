# Runtime ROS2 Interface Phase 2.5

## Service

`/agt/map/runtime/apply`

Request:

- map_id
- version
- generation

Response:

- success
- state
- reason

## Topic

`/agt/map/runtime/state`

Published information:

- map_id
- version
- generation
- runtime state
- backend readiness

## Design rule

Navigation and localization backends must consume the same MapPackage generation.

A mismatch must prevent READY state.
