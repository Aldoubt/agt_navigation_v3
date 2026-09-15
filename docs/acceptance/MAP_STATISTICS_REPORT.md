# v003-indexed Navigation Map Statistics Report

审计对象：`bunker_mid360_mapping_20260901_205036/v003-indexed`  
方式：只读 PGM、YAML、metadata 与既有 P0.7 evidence；未重新生成 PGM/PCD，未修改 map package。

## 资产与来源

`active_map.yaml` 指向该 package。`navigation/converter_metadata.yaml` 记录 navigation PGM 的直接来源为：

```text
v002-edited/localization/global_map.pcd
  -> agt_map_converter pcd_to_nav_map
  -> v003-indexed/navigation/map.pgm + map.yaml
```

来源 PCD 当前 sha256 为 `0f528bf4499b4543f20725ea6c068eca2216f4d7522989ffae0ea179c671d5f2`，与 v003 `metadata.yaml` 的 localization map digest 一致。`map.yaml` sha256 也与 package metadata 一致。package metadata 不单独列出 `map.pgm` 的 digest；本次只读计算的 PGM sha256 为 `92ad12e0ffa9ef59a5d6555fe44aec8d69f6775793018b8ad72b585f864af45d`。

## 栅格统计

| 属性 | 值 |
|---|---:|
| PGM | P5，1125 × 947，8-bit |
| 总 cells | 1,065,375 |
| resolution | 0.10 m/cell |
| 覆盖面积 | 约 10,653.75 m² |
| PGM `254`（Nav2 free） | 36,707（**3.45%**） |
| PGM `0`（Nav2 occupied） | 53,186（**4.99%**） |
| PGM `205`（Nav2 unknown） | 975,482（**91.56%**） |

转换 metadata 的历史字段 `valid_cells=89,893` 与 PGM 中 free + occupied = 89,893 一致，但该字段实际表示 trajectory carve **之后的最终 non-unknown cell 数**，并不是原始 `count>=min_points` 的 cell 数。MQ2 审计确认同一源 PCD 的 raw valid cell 数应单独记录；后续 converter 已新增 `raw_valid_cells` 与 `final_known_cells`，并仅为兼容保留 `valid_cells`。因此 91.56% unknown 的 PGM 统计仍成立，但不能再用该历史字段推断全部 unknown 都来自原始稀疏/未观测栅格；trajectory carve 会把部分原始 unknown 显式改为 free。

## Unknown、线状 occupied 与残影判断

- unknown 共有 1,292 个 8-connected components；最大背景 component 为 887,727 cells，覆盖整张图边界。另有两个内部大 component：70,596 cells（bbox 382×375 cells）和 11,911 cells（303×144 cells）。因此该图存在**大片 unknown**，且部分 unknown 位于已建图区域内部。
- occupied 共有 2,620 个 components；最大的四个面积为 16,655、13,869、6,630、1,666 cells。自动扫描未发现 `area>=20` 且 bbox aspect ratio `>=10` 的极端细长 component；但 PGM 可视化仍能看到沿场地边缘、立杆/植被附近的细长黑色轨迹和点状聚集。
- 这些黑色结构可能是固定边界、杆、植被、动态残影或建图稀疏性造成的投影，**仅凭 PGM 不得标记为动态物体或自动清除**。

## 已冻结的 corridor evidence

权威 P0.7 evidence 位于 `agt_data/acceptance_evidence/.../v003-indexed/p0_7/existing_map_audit.yaml`：使用实际 Nav2 padded `base_link` footprint（front/rear `0.55 m`、half-width `0.43 m`）和 292 个派生 `T_map_base` formal poses，结果为：

| 指标 | 值 |
|---|---:|
| swept | 13,245 |
| free | 13,229 |
| unknown | 11 |
| occupied | 5 |
| occupied regions | 3 |
| MAP Gate | **REVIEW** |

这不是 map package 损坏或自动修图的依据；它说明导航 footprint 与 3 个小 occupied review regions 相交，仍须人工确认。

## 结论

v003 的 PGM 结构和 PCD→PGM provenance 可追溯，但它是一个已知区域占比低、unknown 广泛存在的 navigation map。该性质会降低某些区域规划的选择余量；应先在 RViz 中结合 global costmap、local costmap 和当前机器人位置检查，再决定是否提出有审计记录的 patch。当前审计不建议更改 v003。
