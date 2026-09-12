# AGT Navigation V3 Context


## Current milestone

v0.3.0 localization contract freeze


## Architecture

FAST-LIO2
    |
Batch-LIO
    |
Global Localization
    |
mapping_body contract
    |
LocalizationManager
    |
MapTracker
    |
Nav2


## Frozen decisions

Do not modify:

- GICP
- BBS search
- map assets
- LocalizationManager


## Current validated result

Mapping_body:

MapTracker:
192/192 OK


## Next milestone

P3:

runtime acceptance
Nav2 field test
operator workflow