# 地图生成职责审计

`agt_pcd2grid_exporter` 与 Map Studio 在 Mapping 仓库；V3 的
`agt_map_converter` 也包含 PCD→PGM 和离线验图算法，
`agt_terrain_map_generator` 完全是离线地形生成。
正式导航只消费 registry 解析出的不可变 Map Package。

2026-10-02：已退役 `agt_map_manager` 内的历史 `map_pipeline.py`、
`generate_map_package` CLI 和 `/agt/map/generate` Action。Mapping producer
负责地图生成；V4 的候选构建、校验与晋升仍由 Map Manager 承担。

`agt_map_converter` 的 PCD→PGM 算法保留为显式回归/后备路径，Map Manager 也继续
复用其 PGM 结构校验。由于尚无相同 PCD 对 Mapping 投影器与 V3 converter 的栅格
parity 结果，不能删除或静默替换 converter，也不改变既有地图的 PGM。完成相同输入、
相同参数的 parity fixture 和离线验收后，再决定是否移除 converter；包格式校验与
栅格算法应保持解耦。
