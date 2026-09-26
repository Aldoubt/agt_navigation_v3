# 导航默认地图目录与 FAST-LIO2 切换

> 后续发布更新：本文保留切换 FAST-LIO2 时的核查快照。随后已发布 `20260924-trav-integration-v1` 并更新 latest_validated；当前地图及 map→odom 修正逻辑请见 [发布记录](mcp-map-release-and-correction-20260924.md)。

日期：2026-09-24。仓库：`/home/yangxuan/ros2_ws/src/agt_navigation_v3`。

## 实测默认地图目录

在 Ubuntu 22.04 / ROS 2 Humble 主机上执行正式启动脚本的 `--mode navigation --dry-run`，确认：

| 项目 | 当前值 |
| --- | --- |
| 默认地图根目录 | `/home/yangxuan/ros2_ws/maps` |
| 默认注册表 | `/home/yangxuan/ros2_ws/maps/registry.yaml` |
| 选图参数 | `auto` |
| 注册表规则 | `default_map_id: bunker_mid360`，`default_map: latest_validated` |
| 当前地图包 | `/home/yangxuan/ros2_ws/maps/bunker_mid360/20260922_143427-v4-001` |
| 二维地图（相对地图包） | `navigation/bunker_v1/map.yaml`，引用同目录 `map.pgm` |
| 定位点云（相对地图包） | `localization/global_map.pcd` |
| 重定位资产（相对地图包） | `localization/relocalization/` |

根目录由 `scripts/run_field_stack.sh` 所在工作空间计算，不是源码仓库中的目录。`AGT_MAP_REGISTRY` 可覆盖默认注册表，命令行 `--map-registry` 优先于该环境变量。

导航通过 `resolve_map` 校验 VALIDATED 状态、机器人兼容性和包哈希。latest_validated 是成功发布更新的注册表指针，不按文件修改时间或版本字符串猜最新。以后发布同一地图 ID 的新版，会在下次启动解析；没有热切换运行中导航的行为。

本次没有修改地图注册表。旧 `active_map.yaml` 仍指向 `20260922_143427-fixed-replay-v1`；新版注册表的 active 字段为空，它们不是当前 auto 的来源。上一轮生成的 `experiments/mapping/candidates/bunker_mid360/20260924-trav-integration-v1` 未发布，不会被默认加载。

## 默认里程计修改

以下两个正式入口均已由 `batch_lio` 改为 `fastlio2`：

- `scripts/run_field_stack.sh` 的 `LIO_BACKEND` 默认值及帮助文本。
- `bringup/agt_system_bringup/launch/localization.launch.py` 的 `lio_backend` 默认参数。

默认链路为：

```text
原始 Livox CustomMsg / IMU
  → fastlio2
  → /fastlio2/lio_odom
  → agt_fastlio_adapter
  → /agt/odometry/local
```

默认配置：

```text
/home/yangxuan/ros2_ws/install/agt_navigation_runtime/share/agt_navigation_runtime/config/fastlio2_mid360_navigation.yaml
```

里程计消息表达 `odom -> base_link`；适配器发布 `odom -> base_footprint` TF，Localization Manager 仍独占 `map -> odom`。没有改变算法、外参、传感器话题、速度限制或障碍物配置。

Batch-LIO 保留为显式 `--lio-backend batch_lio` 选项，不同时启动。底层 `navigation_lio.launch.py` 原本就是 Batch 专用入口，本次没有偷偷把它改成另一种后端；正常导航使用顶层 localization 选择器。

两个入口的默认值不按任务模式区分，因此 navigation 和 inspection 都默认 FAST-LIO2。运行中的旧进程不会自动切换；需要先正常退出旧栈，在车辆静止时重新启动和定位。

## 文档更新

- `导航启动文档.md`：默认目录、优先级、具体资产路径、FAST 默认启动命令。
- `README.md`：简介、架构、能力说明及默认地图规则。
- `docs/MAPPING_AND_LIO_POLICY.md`：FAST 默认策略及已发布地图交接说明。
- `AGENTS.md`：维护上下文同步默认后端。
- `docs/mcp-fastlio2-navigation-mode.md`、`docs/archive/导航启动文档_V3_20260923.md`：添加当前规则提示，保留原历史验收记录，不篡改过去的测试结论。
- 本报告：`docs/mcp-navigation-defaults-20260924.md`。

## 验证与范围

- 68 项相关 pytest 测试全部通过，无跳过：后端互斥、默认 FAST、显式 Batch、适配器、launch ownership、任务模式合同等。
- 为旧测试补齐 V4 必需的 `map_id/map_version`；shell dry-run 夹具改用合成的已注册地图，替换已不符合 V4 合同的散装 PCD/YAML。没有为通过测试放宽生产校验。
- 增加省略 `--lio-backend` 的 navigation/inspection 回归，并断言原始里程计话题及解析地图路径。
- 已构建 `agt_system_bringup` 到工作空间正式 install，安装后的 `localization.launch.py --show-args` 显示默认 `fastlio2`。
- 真实 navigation 和 inspection dry-run 均选择 FAST；显式 Batch dry-run 仍可选。
- shell 语法与 Git diff 格式检查通过。
- 正式 `registry.yaml`、`active_map.yaml`、`map_registry.yaml` 的修改前后 SHA-256 一致。

本次没有启动真实传感器、底盘、导航或机器人运动。测试及 dry-run 不代表 FAST-LIO2 已通过本现场的定位精度、长距离漂移或实车导航验收。原 V4 `NOT_READY_FOR_FIELD_ACCEPTANCE` 状态未被改成通过。

## 安全复查命令

在 22.04 主机上执行，以下命令不会启动导航节点：

```bash
cd /home/yangxuan/ros2_ws
src/agt_navigation_v3/scripts/run_field_stack.sh --mode navigation --dry-run
```

应看到：

```text
lio_backend=fastlio2
lio_raw_odometry=/fastlio2/lio_odom
map_root=/home/yangxuan/ros2_ws/maps/bunker_mid360/20260922_143427-v4-001
```

现场启动和停止仍按 `导航启动文档.md` 的验收门与操作顺序进行。

## 备份与回退

备份和日志：`/home/yangxuan/ros2_ws/experiments/mcp_navigation_fastlio_default_20260924/`。

- 修改前工作树干净，`before-source.tar` 是修改前 HEAD 的源码归档。
- `before-installed-bringup-links.tar.gz` 保留修改前安装层文件与符号链接；恢复源码后应重新构建对应包。
- 首次解引用安装层的 `before-installed-bringup.tar.gz` 因旧的失效 launch/缓存链接而不完整，不用于恢复。本次未清理这些无关旧链接。
- `tests-final.log`、`build.log`、三种 dry-run 记录、`installed-launch-args.txt` 与地图指针校验值保留在上述目录。
- 未提交或推送 Git。临时选择旧后端可显式传 `--lio-backend batch_lio`；切换前必须正常停止旧栈，不能同时启动两个前端。
