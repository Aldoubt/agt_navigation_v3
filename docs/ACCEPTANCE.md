# Acceptance plan

## V1 core acceptance

The acceptance target is:

> **3D LiDAR global localization with no initial pose, followed by stable FAST-LIO2 continuous tracking.**

### Test A — static startup

1. Start the system with no `/initialpose`.
2. Load a known 3D map.
3. Present a static MID360 scan.
4. Expect `GLOBAL_SEARCH -> ... -> LOCALIZED`.

### Test B — multiple map locations

Repeat from at least 5 materially different locations and headings. Do not seed the real pose.

### Test C — rough tracked motion

Repeat localization/continuous tracking over rough terrain with the actual tracked chassis vibration.

### Test D — tree shade

Repeat in the known tree-shadow environment. Record success rate and time to localization.

### Test E — rear vehicle geometry

Compare self-filter OFF/ON and confirm chassis rods disappear while environmental points remain.

## Initial target metrics

These are V1 engineering targets, not frozen product specifications:

- global relocalization success: >= 90%
- time to accepted pose: <= 10 s
- planar position error: <= 0.5 m where ground truth is available
- yaw error: <= 5 deg where ground truth is available
- post-handoff FAST-LIO2 tracking: stable for the test trajectory

## V2 system acceptance

Only after V1 is stable:

- software restart without initial pose
- persisted last-pose acceleration
- complete robot reboot
- power-cycle recovery
- RTK/INS prior/fallback integration
- Nav2 mission continuation

Power-cycle tests should verify the whole boot-to-navigation chain separately from the localization algorithm.
