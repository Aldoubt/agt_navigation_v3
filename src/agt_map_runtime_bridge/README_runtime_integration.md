# agt_map_runtime_bridge

## Responsibility

Coordinate applying a selected Map Package to runtime components.

## Components

- Runtime Bridge Node
- Runtime Apply Controller
- Nav2 Adapter
- Localization Adapter
- Consistency Checker

## Non-responsibility

This package does not implement:

- SLAM
- Localization algorithm
- Navigation planning algorithm

It only coordinates runtime state transitions.

## Data contract

Every runtime component must agree on:

- map_id
- version
- generation
