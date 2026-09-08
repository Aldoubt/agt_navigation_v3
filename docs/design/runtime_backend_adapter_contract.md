# Runtime Backend Adapter Contract

## Purpose

Separate map lifecycle management from navigation and localization backends.

## Runtime Bridge

The Runtime Bridge owns:

- map package validation
- generation consistency
- state transitions
- backend coordination

It does not directly implement navigation or localization algorithms.

## Nav2 Adapter

Responsibilities:

- receive navigation map asset
- trigger lifecycle transitions
- report navigation readiness

Required states:

```
PREPARED
ACTIVE
ERROR
```

## Localization Adapter

Responsibilities:

- receive localization map asset
- load backend configuration
- report localization readiness

Required states:

```
PREPARED
ACTIVE
ERROR
```

## Consistency rule

All backends must use the same:

- map_id
- version
- generation

A mismatch prevents navigation activation.
