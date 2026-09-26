# RViz 工作台中文界面

日期：2026-09-24。

## 已完成

- 自定义工具：连续画线、航点编辑。
- 路线工作台面板：操作说明、预览／校验、执行、撤销、重做、清空草稿、清空轨迹、暂停、恢复、取消任务。
- 离线／在线模式、草稿版本、地图、导航健康、定位状态，以及已知的后端状态和拒绝原因。
- 执行确认弹窗及其“确认执行／返回检查”按钮，默认仍为返回检查。
- 离线及正式诊断配置中的工作台、显示项、工具属性、视图、路线顶点、导航地图、预览机器人位置、手绘路线、网格名称。

只改显示层。ROS 话题、服务名、插件标识、后端英文状态协议及执行门控不变；未识别的异常详情保留原文，避免隐藏诊断信息。RViz 本体的 File / Panels / Help、Move Camera 等内置工具及通用属性没有修改。

## 生效方式

已编译安装。运行中的 RViz 不会热更新已加载的 C++ 插件，需要关闭 RViz 窗口后重新打开。

若只关闭 RViz 而保留原后端，使用相同 ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY 可重新接收现有草稿。不要重新启动第二套离线后端。若退出整个 launch，内存中的草稿不会自动保存。

截图标题带星号，表示 RViz 配置有未保存修改。如需保留原布局，请另存到其他文件，不要用旧窗口覆盖安装目录中的新版默认配置。然后重新加载新版默认配置：

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
# 使用与现有后端相同的 ROS_DOMAIN_ID 和 ROS_LOCALHOST_ONLY 设置。
ros2 run rviz2 rviz2 -d \
  /home/yangxuan/ros2_ws/install/agt_rviz_patrol/share/agt_rviz_patrol/config/agt_rviz_path_preview.rviz
```

## 验证

- agt_rviz_route_tools、agt_rviz_patrol 构建安装通过。
- 插件加载及中文按钮、动态草稿状态、拒绝提示和未知诊断保留的断言通过（1 项 gtest）。
- 真实 RViz/Ogre/Qt 图形探针通过：拖画 26 点、中文预览状态、Esc、撤销、重做、航点添加／移动／删除、离线执行禁用。
- 图形测试使用独立 domain 101、虚拟显示 :192、独立 HOME，没有关闭用户窗口或加入其现有离线后端。
- 没有车辆运动测试，没有提交或推送。

证据及备份：`/home/yangxuan/ros2_ws/experiments/mcp_rviz_chinese_20260924/`，包含 before-source.tar.gz、build.log、plugin-test.log、gui-final.log 及实际中文窗口截图。
