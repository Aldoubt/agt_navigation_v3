# TF 审计报告

审计日期：2026-09-19
范围：`agt_system_bringup` 的四个正式入口、定位适配器以及
`tracked_chassis_description` 的现场标定 URDF。

## 目标链

```text
map
 |
odom
 |
base_footprint
 |
base_link
 |
lidar_link
```

实际 URDF 在 `base_link` 与 `lidar_link` 之间保留了两个用于 CAD 和安装标定的
固定帧，因此展开后的物理链为：

```text
map -> odom -> base_footprint -> base_link
    -> chassis_cad_link -> lidar_mount_link -> lidar_link
```

中间固定帧不改变目标链的父子关系语义。

## 审计结论

| 检查项 | 结论 | 依据 |
|---|---|---|
| 连通性 | 静态设计通过；实机需运行脚本确认 | 四段目标关系均有唯一责任方，`scripts/check_tf.sh` 逐段查询 |
| 闭环 | 无闭环，符合 TF 树要求 | TF 应为单连通无环树；可查询反向变换是 tf2 求逆，不是发布闭环 |
| 重复 publisher | 静态设计通过 | `agt_localization_manager` 独占 `map->odom`；LIO 适配器独占 `odom->base_footprint`；`robot_state_publisher` 独占后续固定链 |
| 旋转中心 | 通过 | 动态里程计 TF 的 child 已改为地面投影帧 `base_footprint`；`base_link` 保持固定高度 0.20 m |

## 边与责任方

| TF 边 | 类型 | 唯一责任方 |
|---|---|---|
| `map -> odom` | 动态 | `/agt_localization_manager` |
| `odom -> base_footprint` | 动态 | `/agt_batch_lio_adapter`（正式 launch 只选择 Batch-LIO 路径） |
| `base_footprint -> base_link` | 固定 | `/robot_state_publisher` |
| `base_link -> ... -> lidar_link` | 固定 | `/robot_state_publisher` |

里程计消息 `/agt/odometry/local` 仍以 `base_link` 为 child，保持定位算法和 Nav2
接口不变；只有 TF 动态边换算到 `base_footprint`，从而避免过去
`odom->base_link` 与 URDF `base_footprint->base_link` 让 `base_link` 出现双父节点的风险。

## 标定与旋转中心

- `base_footprint -> base_link`：`xyz=[0, 0, 0.20]`、`rpy=[0, 0, 0]`。
- MID360 相对 CAD 安装俯仰角：`0.226892802759063 rad`（约 13°），属于传感器安装姿态，不应施加到底盘旋转中心。
- 平面导航的转动中心是 `base_footprint`；自车过滤几何仍在 `base_link` 中计算。

## 运行时验收

依次启动 `hardware.launch.py`、`localization.launch.py`、
`navigation.launch.py` 后执行：

```bash
scripts/check_runtime.sh
scripts/check_tf.sh
scripts/check_tf_publishers.sh /tmp/tf_authority.json
scripts/check_topic_rates.sh /tmp/topic_rates.json
```

完整留证：

```bash
ACCEPTANCE_DURATION_SEC=180 scripts/run_hardware_acceptance.sh
```

其中 TF authority 工具按 DDS publisher GID 追溯每条边，运动中心工具只在既有安全控制链
输出原地转向指令时观察 `odom -> base_footprint`，自身不发布速度。详细实机流程见
[docs/HARDWARE_ACCEPTANCE_MID360_BUNKER.md](docs/HARDWARE_ACCEPTANCE_MID360_BUNKER.md)。

本报告的“通过”是代码与配置静态审计结果；没有连接现场 MID360、CAN 与地图资产时，
不能把静态结果冒充为实机运行结果。每次运行证据目录中的 `acceptance_report.md` 必须为
`PASS` 才完成最终验收。
