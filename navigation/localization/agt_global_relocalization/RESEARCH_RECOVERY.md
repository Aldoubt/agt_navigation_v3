# 冻结地图研究分支的定位契约

默认配置保留 V1 静止查询、传统 pose 输出和 formal manager 硬锚定。研究开关默认关闭，标定开关默认 `false`。研究代码通过静态检查和 `BUILD_TESTING=OFF` 编译；尚未得到回放或实车验收结论。

## 数据流

```text
原始 Livox -> 单一 LIO -> /agt/odometry/local
            格式转换 -> 原字段保留的自体过滤 -> 点时刻 deskew
                                            -> immutable query / 单后台 worker
                                            -> BBS / GICP candidate
shadow GICP tracker -> GlobalQuality observation -> formal manager
                                                   map -> odom 唯一发布者
```

自体过滤仅放在 `/agt/livox/points_self_filtered` 的副分支，不能接到 LIO 原始输入之前。`offset_time` 为纳秒，`header.stamp` 为扫描时基。移动查询要求点时刻被同一 odom epoch 的姿态历史包围；越界、缺字段或 odom 间隔过大均拒绝，不能用最近姿态替代 deskew。

## 进程与话题

| 节点 | 研究参数 | 输出 |
|---|---|---|
| `agt_global_relocalization/global_relocalization` | `publish_legacy_pose=false`，`scan_topic=/agt/livox/points_self_filtered`，冻结 `global_map/relocalization_assets/map_id/map_version/map_hash`，实际 `body_to_base_calibration_file` | `/agt/localization/candidate`，查询/配准诊断 |
| `agt_map_tracker/map_tracker` | `shadow_mode=true`，`apply_correction=false`，同一冻结身份和自体过滤输入 | `/agt/localization/global_observation`，强制隔离 `/agt/map_tracking/shadow/pose` 和 `/agt/map_tracking/shadow/status` |
| `agt_localization_manager/localization_manager` | `research_recovery_enabled=true`，同一冻结身份 | 唯一 `map->odom`，`/agt/localization/quality`，`/agt/localization/recovery_status` |

`map_hash` 是实际传给 native 后端的 **PCD 文件原始字节 SHA256**。后台任务携带 job UUID、递增请求 epoch、odom epoch、扫描参考时刻、地图身份、文件状态和截止时间。查询转换与 subprocess 在后台执行；取消终止整个 process group；地图变化、资产文件集合变化、epoch 重置、过期任务均不能发布可用结果。

## 启用边界

- 移动查询需要同时设置 `require_stationary=false`、`moving_query_enabled=true`、`moving_query_calibrated=true`，且收到新鲜有效 `OdomQuality`。
- `quality_calibrated=false` 的 shadow tracker 只生成诊断，不能提供有效全局证据。
- `research_recovery_calibrated=false` 的 manager 不接纳候选，不建立或改变研究 TF 锚。
- 接纳候选需至少两次不同 job、递增参考时刻/请求 epoch，具有一致校正；初次建锚也适用。候选身份、时刻、原始 inliers/overlap/residual 和空间竞争候选 margin 必须满足条件。`candidate_min_ambiguity_margin` 需实测后填为正数；默认 `0` 保持拒绝。
- 恢复限制当前车辆位置的位移及 yaw 改变量，而不只限制 map 原点变化。校正经 SE(3) 插值、当前位置变化率上限处理；同 epoch 的旧校正可作为连续性元数据保留，全局有效性独立为 false。
- 恢复完成需要插值收敛，以及恢复目标设定之后至少两次新鲜 shadow 匹配验证。formal `GlobalQuality.job_id` 和 `RecoveryStatus.job_id` 均指向实际采用的目标 candidate；原始 shadow observation 保留自己的 tracker job ID。

候选 BBS 输出的 ambiguity margin 仅比较已探索的空间分离 BBS 假设，尚不是全局位置后验概率；竞争搜索超时或缺失时无效。普通 full-map BBS 尚无这种竞争证据，因此不能满足研究恢复的 ambiguity gate。研究工具 `build_stable_submap` 可把人工核验的稳定结构 boxes 编译为冻结 stable PCD；使用时必须为该 PCD 重建匹配资产。未实现自动植被语义 mask，不能把人工选择或冻结记录当作在线稳定点识别算法。

配准 score 和旧 covariance 插值仅是诊断。研究质量保留不确定性不可用标记，不把启发式 score 当成概率或实测协方差。所有运动/恢复阈值、车体外参、LIO 噪声和质量映射均需回放与实车后确认。
