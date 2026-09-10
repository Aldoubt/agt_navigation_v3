# Ground segmenter adapters

Implementations in this directory adapt third-party ground-segmentation libraries to `agt_terrain_map_generator::GroundSegmenter`.

Rules:

- Third-party types and headers stay inside the adapter implementation.
- The public terrain pipeline uses `PointCloud` from `terrain_types.hpp`.
- An adapter must return both `ground` and `non_ground` clouds.
- Adapter failure must be explicit; never silently fall back to treating all points as ground/free.
- Parameter translation belongs to the adapter/config layer, not to downstream elevation or obstacle builders.

Planned first adapter: Patchwork++.
