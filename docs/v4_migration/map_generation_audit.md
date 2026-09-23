# 地图生成职责审计

`agt_pcd2grid_exporter` 与 Map Studio 在 Mapping 仓库；V3 的
`agt_map_converter` 也包含 PCD→PGM 和离线验图算法，
`agt_terrain_map_generator` 完全是离线地形生成。
正式导航只消费 registry 解析出的不可变 Map Package。

`agt_map_manager` 目前仍有历史 `map_pipeline.py`，其生成入口依赖
`agt_map_converter`；`map_package.py` 仅复用其中的 PGM 头解析。
这两个依赖是尚待拆分的历史耦合。由于尚无相同 PCD 对两套投影器的栅格 parity
结果，当前不迁移或删除任何一套生成算法，也不改变既有地图的 PGM。
新建图发布入口已限制输出在 `experiments/`；正式发布经候选验证、兼容检查和原子
晋升进入 `maps/`。下一轮应先固定相同输入和参数的 parity fixture，再把 V3 离线
生成代码迁入 Mapping，并抽出 Map Package 格式校验，最后移除 Map Manager 的
生成入口。
