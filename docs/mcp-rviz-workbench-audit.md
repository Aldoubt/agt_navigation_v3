# RViz 离线绘线：源码诊断与隔离测试

日期：2026-09-24

## 结论与交付范围

当前 `path_tool_offline.launch.py` 是**地图上的折线插值预览器**，不是离线路径规划/闭环跟踪仿真。旧方案不跟手首先是输入交互不匹配，其次有可测量的后端阻塞与全量重建开销，不能仅靠调整 Nav2 参数解决。

本轮完成源码审查、原有测试、问题复现、真实 ROS 离线启动测试及性能基线。**没有修改生产绘图/导航代码；连续拖画、逐点编辑和控制面板尚未实现。** 下文双模式工作台为改造方案，不是已交付功能。没有连接底盘、发布真实运动指令、修改地图/定位默认值，也没有提交或推送代码。

源码根目录：`/home/yangxuan/ros2_ws/src/agt_navigation_v3`。
证据目录：`/home/yangxuan/ros2_ws/experiments/mcp_rviz_workbench_20260924`。

## 一、现在究竟用了什么

### 1. 离线入口

`navigation/nav2/agt_rviz_patrol/launch/path_tool_offline.launch.py` 启动：

- `nav2_map_server` 与生命周期管理器；
- 静态 `map -> base_link`，默认机器人位于原点；
- Python `rviz_path_tool`，显式 `preview_only=True`；
- 可选 RViz。

没有 planner_server、controller_server、Navigation Capability。地图从正式 registry 解析；本次实测加载 `bunker_mid360/20260924-trav-integration-v1` 的 2719×1985、0.05 m 栅格。

离线入口含假机器人 TF，**必须放在独立 ROS domain，不能与真实定位同时运行**。它不能证明机器人定位正确或路线可通行。

### 2. 绘线路径

```text
RViz PublishPoint /clicked_point
  -> RvizPathTool.on_click()
  -> 保存顶点
  -> interpolate_polyline()，每段直线按最大 0.05 m 间距采样
  -> /agt/path_tool/route
```

这不是 A*/Smac 求路，也不是曲线拟合或避障。拐角保留为折线；本工具没有调用 smoother_server。

### 3. 正式导航与手绘跟踪是两条不同路径

- `config/navigation.yaml`：全局规划配置为 `nav2_smac_planner/SmacPlanner2D`。
- `config/controller.yaml`：`FollowPath` 使用 `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`，控制器配置频率 50 Hz。
- `capability/agt_navigation_capability/agt_navigation_capability/capability.py::_execute_follow()`：手绘路线经健康门控和 `IsPathValid` 校验后送给 `/follow_path`；**没有先调用 SmacPlanner2D 绕障重规划**。
- RPP 配置了碰撞检测、速度调节、前视距离及转向对齐。这些不等于“手绘线自动变成可通行路线”。50 Hz 是配置值，不是本次实测控制频率。

因此工作台必须区分“按参考线跟踪”和“经航点自动规划”，不能用相同颜色/按钮模糊这两种语义。

## 二、旧方案为什么画得不好、不跟手

主要文件：`navigation/nav2/agt_rviz_patrol/agt_rviz_patrol/path_tool.py`。

| 问题 | 源码依据 | 影响与证据边界 |
|---|---|---|
| 根本没有连续拖画输入 | `agt_rviz_path_preview.rviz` 仅配置 stock PublishPoint，`Single click: true` | 不是持续鼠标移动采样的绘图工具；不能通过提高 Python 回调频率补出原生拖画体验 |
| 第一点击无反馈 | `interpolate_polyline()` 在不足两个顶点时返回空列表 | DDS 测试收到空 Path；保存了顶点但没有可见的首点标记 |
| 每次点击重建整条路线 | `on_click -> route_path -> interpolate_polyline -> make_path` | 输入越长单次编辑越重；不适合直接把每个鼠标移动都变成 clicked_point |
| 回调里等待 TF | `current_map_pose()` 使用 0.1 s timeout | 缺失 TF 实测约 102 ms；默认单线程执行器中会延迟其他回调 |
| 轨迹全量构造/发布 | `on_lio/on_wheel -> make_path`；单条最多 20,000 点 | 长时间轨迹积累后开销显著。无里程计输入时这项不是当前纯离线界面的直接瓶颈 |
| 等待 action server 阻塞 | `submit_path()` 等待 0.5 s | 发生在执行控制路径，不是每一次绘图输入；不应混为同一延迟来源 |
| 画出来是折线，不是平滑可跟踪曲线 | 仅分段线性插值 | 转角突变；后续如平滑必须重新检查碰撞，不能为了视觉圆滑而切过墙角 |
| 缺少工作台控制面板 | 预览配置仅 Displays / Tool Properties / Views | 执行、暂停、清空等依赖 Trigger 服务，没有统一任务状态和错误提示 |

这些是确定的结构缺陷与后端耗时基线；没有测显卡帧率、鼠标事件到显示的端到端延迟，不能把表中耗时直接叫作 RViz 显示延迟。

## 三、比手感更先需要修的执行一致性问题

1. **pending 期间取消丢失**：`on_cancel()` 仅判断 active_goal，start_pending 时返回“no active route”；之后 `on_goal_response()` 接受目标仍进入 RUNNING。使用模拟目标句柄复现，无真实 action 服务器/底盘执行。
2. **运行中仍允许编辑**：clear_route 拒绝活动任务，但 on_click 不拒绝；显示的草稿可能和已提交路径不一致。直接回调测试已复现。
3. **执行时路线改变**：on_start 自动添加机器人到首点的直线。DDS 测试中原路线 21 个点，调用 start 后变为 245 个点，增加约 11.18 m 连接段。离线明确禁用碰撞校验；在线会交给 Capability 校验，但此前预览与提交路线不一致。
4. **恢复时连接段未重采样**：on_resume 直接拼接机器人当前位置和旧路径剩余点；测试构造出 3.162 m 点间距。不能沿用首次提交时“5 cm 采样”的假设。实际 Nav2 对该连接段的安全性本次未测试。
5. **交叉路线恢复歧义**：按全路径欧氏最近点截断，而不是按已确认进度选择，交叉/回环路线存在歧义。这项为源码判断，未做车辆复现。
6. **无有限数值/规模约束**：NaN 顶点会进入列表；后续插值抛出 `cannot convert float NaN to integer`。直接回调已复现异常，不是实际 GUI 输入或网络攻击测试。
7. **preview_only 默认值为 False**：离线 launch 显式覆盖为 True；单独启动可执行文件不能默认当成纯预览。后续应采用显式执行许可并保持正式入口兼容。

## 四、已经做过的测试

环境：通过 SSH 在 Ubuntu 22.04 / ROS 2 Humble 主机执行，`ROS_DOMAIN_ID=90`、`ROS_LOCALHOST_ONLY=1`；未启动任何硬件驱动。已逐字节确认安装环境导入的 path_tool.py 与审查源码一致。

### A. 原有单元测试

```text
navigation/nav2/agt_rviz_patrol/test
capability/agt_navigation_capability/test
3 passed in 0.46s
```

原有测试只覆盖少量几何及策略行为。通过不代表 UI、碰撞检查、取消竞态或实车跟踪合格。

### B. 新增基线复现探针

脚本：证据目录下 `probe_rviz_workbench.py`。

- 首点生成空路径：复现。
- 活动路线仍可添加顶点：复现。
- pending 取消后模拟目标接收仍进入 active：复现。
- NaN 后续插值异常：复现。
- 缺失 TF 等待：101.71 ms。
- 恢复连接段未采样：3.162 m。

**这些断言通过表示旧问题被成功复现，不是问题已修复。** 动作句柄/恢复提交用 mock，ROS 消息类、TF Buffer、路径构造和序列化使用真实 Humble 实现。

### C. 后端性能基线

同一主机，每档路径构造/序列化各 12 次，以下为中位数；非硬实时基准，不含 RViz 渲染、DDS 网络传输。

| 单条轨迹点数 | Path 构造 | 序列化 | 消息大小 |
|---:|---:|---:|---:|
| 100 | 1.88 ms | 0.64 ms | 7,228 B |
| 1,000 | 17.36 ms | 4.86 ms | 72,028 B |
| 5,000 | 86.61 ms | 23.60 ms | 360,028 B |
| 20,000 | 372.70 ms | 93.94 ms | 1,440,028 B |

路线顶点重建每档 8 次；合成正弦轨迹、x 间隔 0.05 m、采样间距 0.05 m：

| 输入顶点 | 输出路径点 | 完整重建中位耗时 |
|---:|---:|---:|
| 100 | 199 | 3.44 ms |
| 1,000 | 1,999 | 36.51 ms |
| 5,000 | 9,999 | 197.95 ms |

20,000 点是当前上限压力档，不表示每次日常操作都达到这一规模。结果支持“限制显示历史、降低发布频率、复用消息、避免鼠标每动一次就重建全路径”，而非宣称 GPU 卡顿已定位。

### D. 真实离线 launch / DDS 冒烟

真实运行已安装 `path_tool_offline.launch.py start_rviz:=false`：

- 收到正式地图 2719×1985；
- 通过 `/clicked_point` 发布两个点，通过 DDS 收到路径；
- 通过真实 `/agt/path_tool/start` Trigger 服务得到离线预览响应；
- 确认提交前后路径变化与自动连接段；
- ROS graph 中无 planner_server/controller_server/bt_navigator，也无 cmd_vel 话题；
- 退出后只剩测试探针，随后关闭探针。

图中出现 FollowRoute 的 feedback/status 话题并不代表有 action server：工具创建 ActionClient 即可产生相应端点。

首次/第二次探针的清理断言失败：同进程基线阶段保留的 ActionClient 资源导致节点仍可发现。探针增加显式销毁 ActionClient、注销 TF listener，并隔离两个阶段的 rclpy context 后重跑通过。第一次向进程组发送 SIGINT 又被 launch 转发导致重复中断，最终探针改为先通知 launch 父进程，保留进程组 TERM 兜底；最终四个子进程均正常退出。原始日志保留，不把测试夹具失败隐去或冒充生产缺陷修复。

### 未测试

- RViz 窗口打开、实际连续鼠标输入和帧延迟；
- 实际 Nav2 IsPathValid 对墙体、未知区和机器人 footprint 的行为；
- Smac 求路与 RPP 闭环仿真；
- 真车横向误差、转角能力、制动距离或底盘安全链。

## 五、双模式验收工作台的改造方案（未实施）

用户已选择同时支持连续拖画与逐点编辑，并分离预览/执行。

### 1. 原生 RViz Tool + Panel

- **拖画模式**：鼠标按下开始一笔，移动在本地立即显示草稿，松开提交该笔；Esc 取消当前笔。不依赖 ROS 回传才能画出游标附近线段。
- **航点模式**：持续点选；首点立即可见；选中、移动、删除、撤销/重做；必要时显示序号与方向。
- 在 map 平面做鼠标射线投影，处理平行射线/视口边界/固定坐标系变化；不把稠密点云拾取当作必需输入条件。
- 采样去抖、最小距离阈值、点数和长度上限；原始笔迹与确认的参考路径分开保存。
- Panel 提供预览/校验、确认执行、暂停、恢复、取消、清空；显示路径版本、地图版本、导航健康、定位状态及错误原因。

### 2. 预览与执行的状态机

```text
DRAFT -> PREVIEWED/VALIDATED -> CONFIRMED -> PENDING -> RUNNING
                      编辑后失效 ↖               -> PAUSING -> PAUSED
                                                -> CANCELING -> CANCELED
                                                -> SUCCEEDED / FAILED
```

- 预览必须包含起点连接段；不能执行时悄悄补线。
- 起点离机器人过远时拒绝直接连接，或显式请求规划连接并重新展示/校验。
- 执行引用已确认的不可变路径版本；编辑、地图变化、定位变化后校验失效。
- pending/active 都能取消；活动路线冻结或明确独立草稿，不能混用。
- 恢复使用进度约束，连接段重新采样和完整校验。
- 离线预览没有执行许可；按钮应明确禁用真实执行。软件取消不替代底盘急停/遥控优先权。

### 3. 性能与安全修改

- 鼠标草稿本地增量绘制；重采样/简化以受控频率或结束一笔时进行。
- TF 查询采用非阻塞读取或异步缓存；不能只加多线程而忽略共享状态。
- 轨迹采集与显示发布解耦、限制历史窗口、定时低频发布；保存完整轨迹交给记录链。
- 所有入口验证 frame、有限数值、长度、点数；平滑后的最终路径重新碰撞校验。
- 不在本轮未经闭环测试就调整 RPP 参数或换规划算法。

### 4. 分层验收，不把离线预览当作全链路通过

1. 几何/状态机单元测试：撤销、重复点、NaN、超限、急转弯、pending cancel、交叉路径恢复。
2. 原生 RViz GUI：同机固定地图与视图，记录鼠标事件到本地草稿帧的 p50/p95/max；长笔迹与长轨迹压力测试。可先设 p95 < 50 ms 为目标，**这不是已达到的结果**。
3. 独立 domain 静态 Nav2：真实 planner/IsPathValid，对墙、未知区、窄通道、机器人 footprint 和起点连接段测试。
4. 独立仿真闭环：真实 RPP + 运动学仿真，速度输出仅接仿真节点；测横向误差、转角、暂停/恢复/取消；不连接 chassis/CAN。
5. 现场低速验收：定位与导航健康通过、人工遥控/急停可用后执行，记录参考线、实际轨迹、速度、误差与地图版本。

## 六、证据与重跑

证据目录包含：

- `before-status.txt`、`before.patch`、`before-rviz-source.tar.gz`：原工作树/源码备份；
- `baseline-pytest.log`：3 项原有测试；
- `probe_rviz_workbench.py`、`probe.log`、`results.json`：最终复现脚本/结果；
- `offline-launch.log`：实际离线节点启动与退出；
- `probe-first.log`、`probe-second.log` 等：初次测试夹具失败证据；
- `maps-before.sha256`：registry.yaml、active_map.yaml、map_registry.yaml，最终校验均 OK。

在主机独立终端、确认 domain 90 未被其他任务使用后重跑：

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1
python3 /home/yangxuan/ros2_ws/experiments/mcp_rviz_workbench_20260924/probe_rviz_workbench.py
```

旧 `docs/RVIZ_FIELD_ACCEPTANCE.md` 含过时入口/后端描述，不能据此推断当前启动链。本报告以当前源码及实测 graph 为依据；未替换既有现场流程。
