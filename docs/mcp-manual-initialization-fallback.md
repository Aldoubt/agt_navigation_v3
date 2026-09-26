# 自动／人工初始化定位保底模式

日期：2026-09-24。仓库：`/home/yangxuan/ros2_ws/src/agt_navigation_v3`。

## 交付状态

已实现三种初始化模式，保留原 `auto` 默认值；默认里程计仍为 FAST-LIO2，正式地图仍是 `bunker_mid360/20260924-trav-integration-v1`。持续地图 tracker 仍默认关闭。

| `--localization-mode` | 行为 |
| --- | --- |
| `auto`（默认） | 自动全局重定位最多两次；仍失败则退出并清理本次启动的进程 |
| `auto_then_manual` | 自动最多两次；失败后进入 `WAIT_MANUAL_INITIAL_POSE`，保留硬件/LIO，等待人工种子＋局部 GICP |
| `manual` | 不发起自动全局搜索，直接等待人工种子＋局部 GICP |

这些是**初始化定位模式**，独立于 `--mode navigation/inspection`、`--lio-backend` 和地图选择，不代表启用持续跟踪纠漂。

## 下午推荐命令

先做不会启动机器人节点的配置检查：

```bash
cd /home/yangxuan/ros2_ws
src/agt_navigation_v3/scripts/run_field_stack.sh \
  --mode navigation --map auto \
  --localization-mode auto_then_manual --rviz --dry-run
```

应显示：

```text
localization_mode=auto_then_manual
lio_backend=fastlio2
map_version=20260924-trav-integration-v1
```

### 现场受控测试

先满足现有 `导航启动文档.md` 的硬件、急停、场地和验收前置要求，确认没有另一套导航在运行。保持车体静止，再执行：

```bash
cd /home/yangxuan/ros2_ws
src/agt_navigation_v3/scripts/run_field_stack.sh \
  --mode navigation --map auto \
  --localization-mode auto_then_manual --rviz
```

若你想一开始就手动定位：

```bash
src/agt_navigation_v3/scripts/run_field_stack.sh \
  --mode navigation --map auto \
  --localization-mode manual --rviz
```

脚本会自行加载主机 Humble 和工作空间 install。这些实际启动命令会启动真实传感器与底盘相关进程，本次远程验证没有执行它们。

## 进入人工等待后怎么操作

1. 自动两次失败后，终端出现 `WAIT_MANUAL_INITIAL_POSE`；`manual` 则直接进入。车辆必须保持静止，等待收集新的静止点云。
2. 使用 `--rviz` 时会打开**初始化专用地图窗口**。选择工具栏 **2D Pose Estimate**。
3. 在地图上点选机器人**base_link 的大概位置**，拖动箭头给出大概车头方向。不是点击雷达安装点，也不是发送 Nav2 Goal。
4. 程序以这个位姿为初值，调用已有原生 `map_gicp_tracker` 做局部点云精配准。人工点击本身不会直接生成有效定位。
5. 若质量检查或定位管理器的时间对齐检查失败，继续等待；重新给一个更准确的位置和朝向即可。失败不启动 Nav2。
6. 匹配结果通过并且 Localization Manager 连续三个采样保持 LOCALIZED、里程计新鲜、全局修正有效后，初始化窗口关闭，脚本才继续启动导航并执行原有后续就绪检查。使用 `--rviz` 时随后打开正常导航 RViz。

人工等待没有强制结束时间；按一次 `Ctrl+C` 走原有有序清理。等待时若本次启动的进程退出，脚本报错退出，不继续启动导航。

### 没有本机桌面

不加 `--rviz` 时仍启动只读初始化地图服务，在远程 RViz 中手动配置：

- Fixed Frame：`map`。
- 地图 topic：`/agt/initialization/map`，Reliable + Transient Local。
- 2D Pose Estimate 输出：`/initialpose`，frame=`map`。
- 与后台使用相同 ROS domain，并保证机器时钟一致。

不要另启动一套 localization/LIO 来获得地图显示，不要添加假的 identity `map -> odom`。若指定了 `--rviz` 但初始化 RViz 节点未起来，脚本会报错，不无限等待一个不存在的窗口。

## 安全边界和状态机

```text
auto_then_manual
  → AUTO_SEARCH，最多两次
     ├─ 成功 → 原有定位就绪门 → 继续 Nav2 启动
     └─ 失败 → 同一定位节点切换为 WAIT_MANUAL_INITIAL_POSE
                  → 人工初始位姿
                  → MANUAL_GICP_REFINING
                     ├─ 失败 → 继续等待人工位姿
                     └─ 发布精配准结果 → VERIFY_MANUAL
                                          → 定位管理器接受 → INITIALIZED
                                          → 三次稳定状态 → 继续 Nav2 启动
```

- `auto_then_manual/manual` 使用一个 `initialization_relocalization` 节点，复用原 `ManualSeedRelocalization` 和 `GlobalRelocalization`；不是自动/人工两个定位节点并行。
- 切入人工阶段时清空 pending 自动请求和旧点云，关闭自动请求定时器。后续恢复请求在这个阶段也只重新开放人工等待，不悄悄重新跑 BBS。
- 自动结果已在确认中或已建立有效定位时，不强行切人工覆盖它。
- 新节点**不发布 TF**；只有原 Localization Manager 能建立 `map -> odom`。
- 人工等待时 Nav2、任务入口及它们的导航动作服务器尚未由本脚本启动。硬件已运行，不代表绝对物理隔离：不要从其他遥控/速度发布源让车辆移动。
- 初始化显示仅含 map_server、它的 lifecycle manager 和可选 RViz，不含 planner/controller/BT，也不发布虚假 TF。地图发布在专用 topic，与后续正式 Nav2 map_server 不冲突。
- 定位建立后，新的 `/initialpose` 点击默认被拒绝，不会悄悄改变运行中的全局锚点。再次初始化应先停止任务并确认静止，按受控恢复流程处理；本次没有设计行驶中的自动人工恢复 UI。
- 常规 `auto` 仍使用原全局定位可执行程序；默认语义没有改变。

全局两次尝试及阶段编排属于 `scripts/run_field_stack.sh`；直接运行底层 launch 不等于运行完整的重试、等待与导航启动编排。

## 人工输入和结果门限

保留原人工 GICP 的静止判据、源地图快照检查、匹配成功标志、进程退出检查、fitness ≤0.60、overlap ≥0.20 等门限，并为新模式增加：

- 初始位姿必须明确在 `map`，位置/四元数为有限数值，四元数非零。
- 位姿时间戳非零，默认不超过 30 秒旧、不能超过当前时间 0.10 秒。
- 必须收集完整静止查询点云；最新点云默认不超过 0.50 秒旧。
- 配准结果及质量指标必须为有限且合法数值。
- 精配准相对人工种子的三维平移不超过 2.0 m、yaw 差不超过 30°；超出时要求操作人给更准确的种子，不直接放宽门限。
- 不把“后端发布了位姿”当成初始化成功，仍等待 Manager 的 frame、协方差、里程计时间对齐和新鲜度检查。

这是局部优化，不是围绕人工点做大范围穷举搜索。重复结构、错误地图、坏标定、错误朝向或距离过远的种子仍可能导致失败甚至错误局部匹配。匹配全程必须保持静止；本次没有把同步 GICP 实现改成具有持续运动监控的异步算法，也没有完成现场误匹配率验收。

## 可观察状态

```bash
ros2 topic echo /agt/localization/initialization_status
ros2 topic echo /agt/global_relocalization/status
ros2 topic echo /agt/localization/status
```

前两个用于查看初始化阶段、人工拒绝原因、GICP 结果。原 `LocalizationStatus` 消息枚举未改变：等待人工在原接口上仍是 WAIT_GLOBAL，不会伪装成 LOCALIZED。完整阶段在新的 JSON 状态 topic 中给出。

内部切换服务为 `/agt/relocalization/enter_manual`，由启动脚本管理；普通操作只需上述脚本和 RViz，不必手动调用服务。

## 实现位置

- `scripts/run_field_stack.sh`：参数、日志、dry-run 和初始化门调用。
- `scripts/localization_initialization.sh`：模式编排、人工等待、地图窗口生命周期与取消。
- `bringup/agt_system_bringup/launch/localization.launch.py`：唯一定位节点选择，沿用选定 FAST-LIO2 冻结标定。
- `bringup/agt_system_bringup/launch/initialization_view.launch.py`、`config/initialization.rviz`：地图显示与人工工具。
- `navigation/localization/agt_global_relocalization/agt_global_relocalization/initialization_relocalization.py`：同节点模式切换、人工输入防护、Manager 确认。
- 同目录 `initialization_policy.py`：模式和输入／结果纯函数检查。
- 原 `manual_seed_relocalization.py`：新增可覆盖的结果校验钩子，原配准算法未替换。
- `navigation/localization/agt_localization_manager/agt_localization_manager/localization_manager.py`：显示等待人工和人工匹配状态；修正公式、TF ownership、平滑参数不变。

## 验证证据与局限

在 22.04 / Humble 主机完成：

- 三个相关包构建并更新到正式 `install/`。
- **115 项 pytest 全部通过**：三模式选择、保留原自动路径、单一定位节点、等待门、取消、错误种子／过期数据、后端质量门、已有定位合同和模式合同。
- shell 语法检查、三个模式真实 dry-run、安装后的入口和 launch 参数检查通过。
- 独立 `ROS_DOMAIN_ID=89`、`ROS_LOCALHOST_ONLY=1` 下进行真实 ROS 验证：故意注入两次自动失败 → 服务切换人工 → 错误 frame 被拒 → 很远的种子被原生 GICP 拒绝 → 近似正确种子由**真实原生 GICP**修正并由真实 Localization Manager 建立 TF → 成功后的额外种子被拒。
- 上述几何测试使用合成点云和 `base_link` 兼容查询模式；预期位置 `(2,-1)`，返回约 `(1.99972,-0.998942)`。这是合成几何功能验证，不是实车厘米级精度承诺，也不代替当前现场 `mapping_body` 标定验收；原 mapping-body 合同单元测试继续通过。
- 无效输入及拒绝阶段没有 `map -> odom`；隔离图中没有 Nav2 控制节点或 cmd_vel topic。
- 初始化地图服务真实启动并读取当前正式地图，收到 2719×1985 OccupancyGrid，除 ROS CLI 查询进程外，功能节点仅 map_server、lifecycle manager 与测试探针；退出后测试节点及本次私有 domain 的 CLI daemon 已清理。
- **没有验证实际桌面 RViz 渲染／鼠标交互，没有启动实机硬件，没有实车行驶验收。** 正式地图的软件校验与现场待验收状态保持不变。

备份和完整日志：`/home/yangxuan/ros2_ws/experiments/mcp_manual_fallback_20260924/`。源码备份包含本轮开始前已有修改；安装层符号链接备份只用于辅助，源码回退后需重编译相应包。未 commit、未 push，未覆盖旧地图或本轮前已有的 FAST-LIO2 默认选择。
