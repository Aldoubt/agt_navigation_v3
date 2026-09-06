# agt_mapping_backend_adapter

Mapping backend abstraction layer.

Purpose:

- isolate mapping session from FAST-LIO2/LIO backend details
- provide a stable save/start/stop interface
- allow future backend replacement

Interfaces:

- start_mapping()
- stop_mapping()
- save_map()

The first implementation will wrap the existing FAST-LIO2 workflow without changing the SLAM backend.
