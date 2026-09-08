# Runtime Service Implementation Plan

## Service
/agt/map/runtime/apply

## Flow

request
 -> validate MapPackage
 -> check map_id/version/generation
 -> apply runtime controller
 -> prepare Nav2 backend
 -> prepare Localization backend
 -> publish runtime state

## Runtime State
States:

IDLE
RECEIVED
VALIDATING
APPLYING
WAITING_BACKENDS
READY
ERROR

## Design Rule
Runtime Bridge coordinates backends but does not contain navigation or localization algorithms.
