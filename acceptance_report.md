# MID360 + Bunker Hardware Acceptance Report

- Overall verdict: **NOT_RUN**
- Scope: v3.1 Runtime Refactor hardware acceptance; architecture unchanged.
- Evidence: 尚未连接实机采集。

## Readiness

| Check | Status | Evidence produced by |
| --- | --- | --- |
| 唯一运行节点 | NOT_RUN | `scripts/check_runtime.sh` |
| 唯一 TF publisher authority | NOT_RUN | `scripts/check_tf_publishers.sh` |
| 关键 topic 频率 | NOT_RUN | `scripts/check_topic_rates.sh` |
| `base_footprint` 运动中心 | NOT_RUN | `scripts/check_base_footprint_center.sh` |
| 导航运行数据 | NOT_RUN | `scripts/record_navigation_run.sh` |

该文件是实机运行前的状态页，不能作为验收通过证据。执行：

```bash
ACCEPTANCE_DURATION_SEC=180 scripts/run_hardware_acceptance.sh
```

每次实机运行会在独立证据目录内生成同名的最终报告。只有五项均为 `PASS` 时，
该次运行才可判定通过。详细步骤见
[`docs/HARDWARE_ACCEPTANCE_MID360_BUNKER.md`](docs/HARDWARE_ACCEPTANCE_MID360_BUNKER.md)。
