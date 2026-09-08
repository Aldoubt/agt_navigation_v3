# Runtime State Machine Contract

## States

```
IDLE
 |
RECEIVED
 |
VALIDATING
 |
APPLYING
 |
WAITING_BACKENDS
 |
READY
```

Failure path:

```
ANY STATE -> ERROR
```

## READY condition

All conditions must be satisfied:

- Map package valid
- generation consistent
- navigation backend ready
- localization backend ready

## Principle

Map selection and robot runtime application are separate responsibilities.
