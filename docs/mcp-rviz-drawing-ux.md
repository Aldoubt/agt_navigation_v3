# RViz 绘图交互改进：距离、缩放与可见性

日期：2026-09-24。

## 修复内容

1. 路线工作台新增高对比距离栏。草稿显示 XY 折线总长、末段长度和顶点数；拖画／移动顶点时实时更新（最多 10 Hz）。预览成功后另显示含机器人到首点连接段的总长。
2. 原生 Tool 之前没有把滚轮事件传给视图控制器，导致选择绘图工具后无法缩放。现在优先转发滚轮及非绘制状态的中键事件，支持滚轮缩放、中键平移；后端锁定或断连也不应阻止查看地图。
3. 正在绘制的路线从浅蓝改为亮粉红，按当前视图比例近似保持 4 像素线宽。已提交路线改为亮黄色 Billboard，线宽 0.35 m；顶点改为粉红、放大到 0.35 m。路径显示向上偏移 0.08 m，减少与地图面的重叠。

颜色／线宽／显示高度仅用于可视化，不改变执行路径、机器人 footprint、采样间距或碰撞规则；不能把显示线宽当成可通行宽度。距离为地图 XY 平面折线长度，不是三维地形实际里程。

## 实现

- 原生插件新增 route_metrics.hpp 与独立距离 QLabel；绘制指标走 /agt/path_tool/drawing_metrics（轻量 JSON，不提交路线、不发运动目标）。
- 后端在生成预览快照时计算 length_m，通过 editor_state.preview_length_m 发布；执行路径及确认流程不变。
- 离线配置和正式诊断配置同步更新手绘路线显示样式。

## 测试证据

证据目录：`/home/yangxuan/ros2_ws/experiments/mcp_rviz_drawing_ux_20260924/`。

- 构建并安装 agt_rviz_route_tools、agt_rviz_patrol 成功。
- Python 相关测试：37 passed，包含新增“预览长度包含起点连接段”断言。
- C++：2 项通过，含空路线、单点、3+4=7 米折线及末段 4 米的计算断言。
- 真实 RViz/Ogre/Qt，独立 domain 102 / 显示 :193：两种工具均验证滚轮缩放，Scale 8 -> 8.96；反向滚轮方向正确；中键平移不编辑草稿。
- 持续拖画时距离栏实时变化；26 点草稿总长 9.72 米、末段 0.39 米；包含连接段的预览总长 9.87 米。
- 原有拖画、预览、Esc、撤销／重做、航点添加／移动／删除和离线禁止执行仍通过。
- 初次探针错误地假设滚轮正反一格比例完全互逆，等待恢复精确原值超时；RViz 实际按比例增减。夹具改为验证反向缩放方向，并显式恢复测试视图，最终通过。

备份为 before-source.tar.gz；日志见 build-final.log、pytest.log、plugin-test-final.log、gui-final.log。截图为 workbench-gui.png-live-stroke.png（拖画中）及 workbench-gui.png-drawing.png（预览后）。没有关闭用户当前会话，没有执行底盘任务，没有提交或推送代码。

## 加载方式

本次同时更新了插件、RViz 配置和后端距离字段，建议完整重启离线 launch，不要只打开一个 RViz 空窗口。重启后端会清空未持久化的草稿。

先在原离线启动终端 Ctrl+C，确认旧离线节点退出，再使用独立且未被车辆使用的 domain：

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1
ros2 launch agt_rviz_patrol path_tool_offline.launch.py \
  map_registry:=/home/yangxuan/ros2_ws/maps/registry.yaml \
  robot_x:=17.0 robot_y:=8.0
```

不要用旧窗口保存并覆盖新版默认 RViz 配置。若仅重开 RViz 而继续使用旧后端，草稿距离和缩放可用，但新的预览总长字段及顶点样式需要后端重启。
