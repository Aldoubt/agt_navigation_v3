# RViz 导航验收工作台：实现与测试说明

日期：2026-09-24。源码仓库：`/home/yangxuan/ros2_ws/src/agt_navigation_v3`。

## 交付状态

已实现、在 Ubuntu 22.04 / ROS 2 Humble 主机编译并安装：

- 原生 C++ RViz **Draw Route** 连续拖画工具；
- 原生 **Edit Waypoints** 航点添加、拖动、删除工具；
- **Route Workbench** 操作面板：预览/校验、执行、撤销/重做、清空、暂停、恢复、取消；
- Python 安全后端：版本化草稿、预览快照、显式执行、pending 取消/暂停、恢复重采样、输入限界；
- 默认纯预览，正式导航入口显式启用执行；
- 非阻塞 TF 查询、限长且定时发布的 LIO/轮速轨迹。

**这是编辑器和执行接口的第一版，不是新建的 Nav2 离线规划/闭环仿真系统。** 离线预览仍不运行碰撞校验或控制器；在线沿用 Capability → IsPathValid → RPP，不绕过定位、健康、速度安全链。没有实际车辆运动、没有修改正式地图或定位默认配置，没有提交/推送代码。

## 1. 如何打开离线工作台

在带图形桌面的 Humble 主机终端运行，不是在没有 ROS 的 Ubuntu 24.04 容器运行：

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash

# 必须选一个未被车辆或其他任务使用的独立 domain。
export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1

ros2 launch agt_rviz_patrol path_tool_offline.launch.py \
  map_registry:=/home/yangxuan/ros2_ws/maps/registry.yaml \
  robot_x:=17.0 robot_y:=8.0
```

`robot_x/robot_y/robot_yaw` 是**离线假机器人位置**，不是定位结果，也不是对真车下发的位置。上例是图形测试所用位置，可按预览区域调整。启动器现在要求显式非零 domain（1..232）及 `ROS_LOCALHOST_ONLY=1`，否则拒绝启动；这仍不能代替人工确认该 domain 未被真实车辆使用。

后台无图形测试可加 `start_rviz:=false`。本轮已编译安装，无须重复构建。若在其他工作区部署，新增的 C++ 包也必须构建：

```bash
cd /home/yangxuan/ros2_ws
colcon build --symlink-install --packages-select \
  agt_rviz_route_tools agt_rviz_patrol agt_system_bringup
source install/setup.bash
```

### 操作顺序

1. 选择顶部 **Draw Route**（快捷键 D）。左键按住拖动，松开提交一笔；绘制中由 RViz 本地立即显示笔迹，不等 ROS 回传。下一笔追加到已有草稿，笔间连接也会显示。
2. 选择 **Edit Waypoints**（E）。空处点击增加顶点，靠近顶点按住拖动修改；选中的顶点可用 Delete 删除。拾取半径约 10 屏幕像素。
3. Esc 取消当前未提交手势；右键也可取消当前手势。已经松开提交的笔迹用 **Undo** 撤销，**Redo** 重做，最多保存 20 次编辑历史。
4. **Preview / Validate** 显示完整路径，包括机器人到首点的连接段。默认连接距离不得超过 1 m，超出则拒绝；不会执行时偷偷加长连接线。
5. 离线面板明确显示 `OFFLINE / NO MOTION`、地图版本和 `no collision validation`；**Execute / Resume 禁用，直接调用后端执行服务也会拒绝**。
6. 需要平移、缩放视图时，切换 RViz 的 **Move Camera** 工具。

首个顶点由独立 Marker 显示，避免旧版“点击了但空 Path 看不到”的情况。草稿线仍是折线参考路径，不自动做曲线平滑、绕障或全局规划。

## 2. 在线接入与确认执行

既有 `scripts/run_field_stack.sh` 默认加载的 `agt_rviz_costmap_diagnostics.rviz` 已加入工具、面板和顶点显示；`navigation.launch.py` 为工作台显式设置 `preview_only=False`，传入地图标识及 use_sim_time。没有新增第二个定位/导航所有者。

**在线使用单独终端，恢复车辆既有 ROS 环境；不要沿用上面离线 domain 的终端直接启动车辆栈。** 在线不能启动 `path_tool_offline.launch.py`，因为该入口发布假 TF。

执行顺序：

```text
绘制/编辑草稿
  -> Preview / Validate
  -> 等待 VALIDATED
  -> Execute 弹窗确认
  -> Navigation Capability 再次健康门控及路径校验
  -> Nav2 FollowPath / Regulated Pure Pursuit
```

面板显示模式、草稿版本、地图版本、导航健康和定位标志。在线执行要求：

- 已收到地图、导航健康 READY 且接收时间不超过 1.5 s；
- 定位 TF 非阻塞可读，在线时间戳年龄不超过 0.5 s（未来容忍 0.1 s）；
- 预览已通过 `/is_path_valid`，异步校验超过 3 s 作废；
- 预览不超过 30 s；地图/草稿版本未变；
- 确认时机器人相对预览时位移不超过 0.1 m、偏航变化不超过 0.15 rad；
- 无活动或等待接收的任务。

提交的是**已预览且校验的路径快照**，不是执行按钮点击时重新生成的另一条路径。服务响应“已提交”不表示车辆已到达或任务已成功；以最终任务结果为准。

### 暂停、恢复、取消

- pending 和 active 阶段均支持取消/暂停；pending 的请求被记录，目标接收后立即请求取消，等待终态。
- 暂停成功后草稿仍锁定。先点 **Preview / Validate** 预览剩余路径，再确认 **Resume**；不是直接猜测最近点后重新发车。
- 进度只在当前进度附近的局部前向窗口估计，不在全路线寻找最近点。恢复连接段重新按间距采样和校验。
- 暂停后机器人移动超过 0.15 m，或者暂停点附近存在非局部交叉/回环歧义，恢复被拒绝；需要取消旧任务后重画。
- 目标接收/结果状态未知时保持锁定，不允许启动第二条任务。取消服务请求成功也不等于底盘已物理停车。

**软件 Cancel 不是硬件急停；必须保留遥控优先权和实际急停。** 当前进度估计并非来自控制器的精确已执行索引，复杂路线仍需闭环及现场验收。

## 3. 实现文件与兼容性

新增包：`navigation/nav2/agt_rviz_route_tools/`

- `src/route_tools.cpp`、`include/.../route_tools.hpp`：原生 Tool / Panel；
- `plugins.xml`、`CMakeLists.txt`、`package.xml`：RViz 插件注册及构建；
- `test/test_plugin_load.cpp`：无显示器的插件装载测试；
- `test/gui_probe.cpp`：真实 RViz/Ogre/Qt 窗口交互探针。

原包 `navigation/nav2/agt_rviz_patrol/`：

- 新增 `route_editor.py`：有界草稿、版本、撤销/重做；
- 新增 `workbench.py`：生产入口使用的安全后端；
- `path_tool.py`：保留基础几何/轨迹支持，main 改为 RouteWorkbench；TF 改为零等待；
- `test/test_workbench.py`：新增输入、状态机、安全回归；
- 更新离线 launch、离线 RViz 配置、正式诊断 RViz 配置与包依赖。

草稿协议：`/agt/path_tool/edit` 为 String JSON，包含 `frame: map`、整数 `revision` 和二维 `points` 数组；后端使用期望版本拒绝陈旧编辑。`/agt/path_tool/editor_state` 为可靠、持久化深度 1 的状态/草稿快照。仍兼容 `/clicked_point`，但同样受输入、版本及锁定策略约束。

新增 Trigger 服务：`/agt/path_tool/preview`、`undo`、`redo`。保留 start/cancel/pause/resume/clear_route/clear_trails。

**兼容性变化：离线 `/start` 不再顺便生成预览，必须改用 `/preview`；单独启动 rviz_path_tool 默认禁止执行。** 在线必须先预览校验后启动。

输入限制：2,000 顶点、200 m 草稿长度、最多 10,000 个最终采样点；拒绝 NaN/Inf、非 map 坐标系、异常 JSON、陈旧编辑；采样间距限制为 0.01～0.05 m。自由笔迹最小采样距离约 3 cm，松开后才提交整笔。默认仍使用 5 cm 最终采样。

## 4. 性能变化与证据边界

原生拖画采用本地增量绘制，不再将每次鼠标移动送到 Python 重建全路径。路径处理发生在笔迹提交/编辑提交阶段，并有点数与长度上限。

轨迹采集回调只采样，显示轨迹最多保留每条 1,000 点，由 2 Hz 定时器发布，不再最多 20,000 点且每个里程计更新全量重建。完整轨迹应交给原有记录链，而非无限留在 UI 显示历史。

同机测量：

| 项目 | 旧版基线 | 新版测量/机制 |
|---|---:|---:|
| 缺失 TF 查询 | 约 101.7 ms | 中位 0.019 ms，最大 0.113 ms |
| 单条显示轨迹上限 | 20,000 点 | 1,000 点 |
| 上限轨迹构造耗时 | 中位约 372.7 ms | 中位约 18.8 ms |
| 轨迹发布 | 每次采样后发布 | 最大 2 Hz，且只发布变化轨迹 |
| LIO 采样回调（用固定姿态替代 TF） | 未测相同档位 | 中位约 0.003 ms，p95 约 0.0043 ms |

轨迹耗时对比包含了**降低显示历史上限**这一设计变化，并非相同 20,000 点负载下提速。以上均为后端/处理基线，不是鼠标到屏幕的端到端延迟。没有宣称 p95 < 50 ms 的 GUI 显示目标已经达标。

## 5. 已执行的测试

### 编译与单元/回归

- 三个相关包在 Humble 主机编译安装通过。
- 插件装载/构造：**1 项通过**，同时构造两个 Tool 和一个 Panel。
- 最终仓库回归（排除无法导入的相机测试目录）：**367 passed，22 skipped，11.15 s**。
- 未排除目录的全仓库测试在收集阶段因相机模块的 NumPy/OpenCV ABI 冲突失败：`numpy.core.multiarray failed to import` / `_ARRAY_API not found`。没有修改主机 NumPy/OpenCV 或把这次运行称为全仓库全部通过。

新增回归覆盖 NaN/Inf/超限/错误 JSON、版本冲突、首点状态、撤销重做、有界历史、活动/暂停锁定、默认离线拒绝执行、连接段采样、过期/姿态变化预览、陈旧校验回调、pending 取消/暂停、恢复歧义和未知目标状态保持锁定。

### 真实 RViz 图形交互

使用实验目录内解压的 Xvfb，不进行系统级安装、不占用主机现有桌面；独立显示 `:191`、独立 ROS domain 91、软件 OpenGL 渲染。

加载正式 2719×1985 地图和新 RViz 配置，在真实窗口发送 Qt 鼠标/键盘事件：

- 连续拖画生成 26 个草稿点；
- 点击 Preview 并收到离线预览状态；
- Esc 不提交未完成笔迹；
- Undo / Redo；
- 航点点击添加、拖动修改、Delete 删除；
- Execute / Resume 在离线模式禁用。

最终日志：`GUI_PASS: drag, preview, Escape, undo, redo, add, move, delete, offline execution disabled`。

真实窗口截图保存为 `workbench-gui.png` 和 `workbench-gui.png-drawing.png`，不是效果图。软件渲染日志出现一次地图 shader link 错误，之后地图显示成功；已检查截图。仍应在现场显卡/桌面上复核渲染和人手操作感受。

首次 GUI 探针的 Delete 事件没有正确投递到 RViz RenderPanel；实现补充绘图时的键盘焦点，探针向实际接收键盘的 QWidget 投递后通过。增加预览按钮测试时还发现夹具早于地图到达就点击，后端正确拒绝；夹具改为等待 map_received 后最终通过，相关失败日志保留。

### 真实 DDS / Service / Action 协议测试

独立 domain 92，启动真实安装的工作台后端。TF、健康、IsPathValid 和 FollowRoute **由测试夹具提供**，没有 Nav2 控制器或速度端点：

- 人为延迟目标接收 0.6 s，pending 取消最终确实取消目标；
- pending 暂停成功，重新预览/校验后恢复，再取消；
- 执行接收路径与校验快照序列化字节完全一致；
- 校验返回无效时执行被拒绝；
- 3 次夹具目标均受控结束；无 cmd_vel 话题，后端清理完成。

这验证了真实通信和状态机，**不证明真实 Nav2 footprint 碰撞检测、规划器或车辆跟踪性能**。

## 6. 尚未完成的验收

- 真实 Nav2 静态地图/footprint 碰撞场景（墙、未知区、窄通道）；
- Smac + RPP + 运动学仿真的闭环误差测试；
- 实车的转角、横向误差、制动距离、暂停位置和复杂回环恢复；
- 现场桌面/显卡的人手操作与端到端 GUI 延迟测量；
- 自动平滑及绕障规划没有新增，参考线仍为折线。

下一阶段应先做真实 Nav2 静态校验及隔离闭环仿真，再安排现场低速验收，不直接把本次编辑器测试当作允许发车的证明。

## 7. 证据、变更与回退

证据目录：`/home/yangxuan/ros2_ws/experiments/mcp_rviz_workbench_impl_20260924/`。

主要文件：

- `before-source.tar.gz`、`before-status.txt`、`before.patch`：实施前备份，保留原有未提交工作；
- `final-build.log`、`regression-final.log`、`plugin-test-final.log`；
- `full-pytest.log`：相机环境冲突的原始证据；
- `gui-final.log`、`run_gui_probe.py`、两张真实窗口截图；
- `protocol-final.log`、`protocol-results.json`、`probe_workbench_protocol.py`；
- `performance.json`、`maps-before.sha256`。

回退时按备份恢复本轮修改的原包、配置及 navigation.launch.py，再重新构建相关包；新插件不再被配置引用后可停用。不要使用 `git reset --hard` 或清理整个工作树，以免覆盖此前地图/定位工作。当前未做自动回退。
