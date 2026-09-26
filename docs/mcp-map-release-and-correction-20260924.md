# 正式地图发布与 map→odom 修正核查

日期：2026-09-24。导航仓库：`/home/yangxuan/ros2_ws/src/agt_navigation_v3`。

## 一、发布结果

用户已确认将上一轮 traversability 候选作为下午受控测试用正式地图发布。

- 新地图包：`/home/yangxuan/ros2_ws/maps/bunker_mid360/20260924-trav-integration-v1`。
- 二维地图：包内 `navigation/bunker_v1/map.yaml` 与 `map.pgm`。
- 定位点云：包内 `localization/global_map.pcd`。
- 重定位资产：包内 `localization/relocalization/`。
- `maps/registry.yaml` 的 `latest_validated` 已更新为该版本。
- 正式启动脚本 dry-run 已确认：`lio_backend=fastlio2`、`map_spec=auto`、`map_version=20260924-trav-integration-v1`。
- 旧版 `20260922_143427-v4-001` 完整保留。旧 `active_map.yaml`、`map_registry.yaml` 未改变；请使用新的 registry/auto 启动入口。

通过已安装 `agt_map_manager.map_promotion.promote_candidate` 的校验、锁及原子注册事务发布，没有绕过包校验。本次没有使用 `mapping_map_release --confirm-reviewed` 冒充已完成现场验收；根据用户明确确认，直接复用底层发布事务，并把真实验收状态写入候选后随包发布。

新包 `field_acceptance.yaml` 记录：

```yaml
purpose: user_authorized_controlled_field_test
user_authorized_publication: true
software_integrity_check: pass
field_acceptance: pending
robot_motion_started: false
```

注册表的 VALIDATED 是软件包校验状态，**不是实车安全验收通过**。现场状态是审计记录，目前不会被已有导航解析器自动作为阻止启动的条件，因此需要操作人落实受控测试门。

### 数据一致性

- 发布后重新执行 `resolve_map(auto, bunker_v1)`，包校验通过且解析到新版本。
- 正式二维 PGM 与实验 v5 输出 SHA-256 一致。
- 新版与之前默认 v4-001 的 `localization/global_map.pcd` SHA-256 相同：`b14492dfea7bc4c4e58e79b74e787c0be5fcc91fd8ec1fe6dc94c8b3fd96e451`。
- 本次主要更新二维规划图，未重新计算一套不同的三维 SLAM 地图。不能因此声称定位精度已改善。

备份和记录：`/home/yangxuan/ros2_ws/experiments/mcp_map_release_20260924/`，包含三个旧指针文件、旧候选状态、`publication-result.json`、发布后 dry-run 和定位测试日志。

## 二、当前默认是否持续修正 map→odom？

**默认没有启用持续地图跟踪纠漂。**

实际安装的 `agt_system_bringup/launch/localization.launch.py` 中：

```text
enable_map_tracking = false
```

`scripts/run_field_stack.sh` 启动 localization 时没有覆盖它，也没有提供对应的命令行开关。脚本传入 `auto_relocalize:=false`，等待传感器和适配器就绪，再主动调用 `/agt/localization/relocalize` 完成初始定位，失败时最多作两次启动尝试。

所以默认流程是：

```text
启动并保持静止
 → 全局重定位
 → 建立 map→odom
 → FAST-LIO2 提供连续 odom→base
 → 持续广播已有 map→odom，而不是每帧重新匹配地图
```

如果没有新的全局定位结果或地图跟踪输入，map→odom 的数值保持不变。30 Hz 的 TF 广播频率不等于 30 Hz 全局纠漂。

这意味着：**FAST-LIO2 持续输出但缓慢漂移时，默认配置不会依靠后台地图匹配自动消除这部分漂移。** `LOCALIZED` 状态不等于持续测量了地图匹配误差。

本次没有擅自打开 tracker 或改变纠偏门限，也没有检查正在运行的 ROS 图；上述结论针对当前安装代码的正式默认启动流程，不代表另行手动启动的节点。

## 三、首次定位及重新定位：硬更新

权威发布者只有 `agt_localization_manager`。关键实现：

`navigation/localization/agt_localization_manager/agt_localization_manager/localization_manager.py` 的 `_on_global_pose`、`_nearest_odom`、`_validate_global`。

1. 全局定位器从实时点云和正式三维地图求机器人在 map 中的位置。当前优先采用 Polar Context 候选、3D-BBS 粗定位、small_gicp 精配准；完成 mapping-body 到 base 的一次坐标转换后，发布 `/agt/relocalization/pose`。
2. 全局定位要求静止，使用 `/wheel/odom` 作为静止判据；当前配置还具有 score、fitness、overlap 等匹配质量门限。
3. Localization Manager 从 30 秒里程计缓存中取与全局位姿时间戳最近的一条本地里程计，允许最大时间差 0.10 秒。这里是最近邻采样，不是插值。
4. 按完整三维刚体变换计算：

```text
T_map_odom = T_map_base(t) × inverse(T_odom_base(t))
```

不能直接用两个 xyz 相减，必须考虑旋转。

5. Manager 要求 frame=`map`、非零时间戳、有效协方差；位置标准差上限 1.00 m、yaw 标准差上限 20°，默认拒绝全零协方差。
6. 接受后**同时直接替换 current 和 target**，不是渐变。首次定位和再次成功全局定位都走硬更新路径，可能造成 map 下位姿跳变。

0.50 m / 5° 的小创新门限属于下面的 tracker 路径，不能套用来描述全局硬更新；Manager 的 `_on_global_pose` 本身没有用这些门限限制重定位跳变量。上游匹配质量门不等于“新旧位置跳变一定很小”。

手动调用 `/agt/localization/relocalize` 会先清除现有修正、进入 RELOCALIZING，再请求新定位。因此不要行驶中随意调用它，先停任务并确认车辆静止。

## 四、代码已有但默认未启动的持续跟踪纠偏

若未来明确启用 `agt_map_tracker`，它以配置的 0.5 Hz（目标每两秒一次，实际受计算耗时影响）做局部地图配准，输出 `/agt/map_tracking/pose`，**tracker 自己不发布 map→odom**。

Manager 的 `_on_tracking_pose` 只在已有全局锚点、且未处于 LOST/恢复状态时接受跟踪输入；除通用 frame/时间/协方差检查外还要求：

| 门限/策略 | 当前配置 |
| --- | --- |
| 测量与预测 base 位姿的平移差 | ≤0.50 m |
| yaw 差 | ≤5° |
| 连续一致测量 | 至少 2 次 |
| 连续两次候选修正平移差 | ≤0.20 m |
| 连续两次候选修正 yaw 差 | ≤2° |
| 平滑时间常数 | 3.0 s |
| 平移修正最大速率 | 0.10 m/s |
| yaw 修正最大速率 | 2°/s |
| TF 广播频率 | 30 Hz |

满足要求后只更新 target；`_tick` 以 `alpha=1-exp(-dt/3)` 并叠加平移/yaw 速率限制，平移线性插值、四元数球面插值推进 current。变换是 SE(3)，不是只修 x/y/yaw；速率门显式约束平移长度和 yaw，并非独立的 roll/pitch 速率门。

tracker 的匹配失败、DEGRADED、RECOVERY_REQUIRED 还有对应恢复逻辑；RECOVERY_REQUIRED 会清除旧修正并根据配置请求全局重定位，恢复请求冷却为 5 秒。这些路径只有实际存在 tracker 输入才会触发，不能当成当前默认已开启的保护功能。

## 五、本地里程计过期处理

`_update_state` 按本地里程计的**接收时间年龄**判断：

- 超过 0.30 秒：DEGRADED。
- 超过 1.00 秒：LOST。
- LOST 后停止刷新 map→odom，不继续伪装成有效 TF。
- 仅由暂时本地里程计中断引起的 LOST，在已有修正仍保留且里程计恢复时允许恢复，不必强制重定位。

这不是 FAST-LIO2 重启后局部坐标原点自动对齐的保证。如果 LIO 重启、原点重置或位姿突跳，应停止测试并重新静止定位，而不是仅看话题恢复就继续跑。

## 六、下午受控测试建议

1. 场地隔离、急停和人工接管可用，先保持车辆静止。此次不是常规无人值守运行。
2. 首先执行无运动检查：

```bash
cd /home/yangxuan/ros2_ws
src/agt_navigation_v3/scripts/run_field_stack.sh --mode navigation --dry-run
```

确认 `fastlio2` 和 `20260924-trav-integration-v1`。不要另用旧 active 地图入口。

3. 完成既有现场验收前置门后，按 `导航启动文档.md` 的顺序启动后台与 RViz；只启动一套栈。等初始化定位完成、健康状态就绪，核对地图、车头方向和实际障碍，再开始短距离、可随时停下的受控试跑。
4. 本轮先明确是在测试“新二维地图 + 默认 FAST-LIO2 + 启动全局锚定”，不是已经测试了持续 tracker 纠漂。建议记录 `/tf`、`/tf_static`、`/agt/odometry/local`、`/wheel/odom`、`/agt/relocalization/pose`、`/agt/localization/status`、`/agt/localization/metrics` 和 `/agt/global_relocalization/status`；如果另行启用 tracker，再记录其 pose/status。
5. 重点看起点静止定位、直行和转弯、回到起点后的偏移；在 RViz 同时观察障碍点云与地图，不要只看规划路径是否能生成。地图中的路沿、窄通道、低悬物与历史足迹清空区域需要重点确认。
6. 出现地图位姿跳变、车头方向错误、点云明显错位、LOCALIZED/健康状态丢失时，停止任务并确认静止，保留日志后再排查，不通过运行中手动重定位掩盖问题。

### 回退到上一个默认地图

先停止旧任务/旧栈并确认静止。以下先 dry-run，只验证选择：

```bash
src/agt_navigation_v3/scripts/run_field_stack.sh \
  --mode navigation --map bunker_mid360/20260922_143427-v4-001 --dry-run
```

完成现场检查后，在实际启动命令里保留同一个显式 `--map`。这不会删除新地图，也不改变 latest_validated。不要只改旧 active_map.yaml 期待 auto 回退。

## 七、验证与变更范围

- 核对实际安装的 Localization Manager Python、Manager YAML、tracker YAML、顶层 localization launch 以及全局重定位模块/配置，与审阅源码一致。
- 本轮重跑定位修正数学/状态和 query-frame 合同测试：**26 passed**。
- 完成正式发布后包哈希校验和默认导航 dry-run。
- 本次只发布地图并更新说明；没有更改修正算法、tracker 开关、标定或控制参数，没有启动机器人。
