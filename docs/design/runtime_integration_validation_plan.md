# Runtime Integration Validation Plan

## Purpose

Validate that a selected Map Package can drive navigation and localization consistently.

## Validation items

### 1. Map consistency

Check:

- map_id
- version
- generation

Navigation and localization assets must match.

### 2. Runtime apply flow

Expected:

```
REQUESTED
 -> VALIDATING
 -> APPLYING
 -> WAITING_BACKENDS
 -> READY
```

Failure:

```
ERROR
```

### 3. Recovery tests

- restart after map activation
- invalid package rejection
- backend unavailable handling

## Merge criteria

Before merging to main:

- ROS2 build passes
- interfaces compile
- runtime node starts
- Nav2 adapter tested
- localization adapter tested
