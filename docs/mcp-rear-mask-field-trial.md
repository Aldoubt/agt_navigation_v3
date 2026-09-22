> **2026-09-22 现场结果：本页记录的是已完成的旧 10° 窄扇区试验。**
> 该试验确认过滤器生效，但安装杆实际位于后方偏左约 20～30°，仍会进入局部代价图并触发
> controller collision abort。当前显式验证入口已改为 `run_field_stack.sh --rear-pointcloud-mask`，
> 使用 180±35°、0.5–1.0 m 的短距离范围；最新结论和命令见 `导航启动文档.md` 第 9 节。

# FAST-LIO2 + 后方窄扇区过滤：本轮跟踪试验

日期：2026-09-22。用户选择先开启现有后方点云屏蔽，自行进行 FAST-LIO2 跟踪测试，再根据效果调整。

## 本轮唯一感知参数变化

`config/perception.yaml` 中 `rear_filter.enabled: false → true`。

```yaml
rear_filter:
  enabled: true
  center_deg: 180.0
  width_deg: 10.0
  min_range_m: 0.5
  max_range_m: 4.0
```

这是 `base_link` 后方中心左右各 5°、XY 平面距离 0.5–4 m 的窄扇区，**不是屏蔽整个后半圈**。
扇区条件本身不限制 Z。配置由 FAST/Batch 两个模式共享。

保持不变：自车 box、地面过滤、体素参数、射线清除、外参、LIO 原始输入、IMU、控制器、
碰撞检测和定位运动门控。没有顺带修改 LOST 恢复或继续调节其他参数。

过滤只作用于 `/agt/navigation/points_obstacles` 障碍标记支路：

```text
/livox/lidar、/livox/imu → FAST-LIO2              不经过后方过滤
/agt/livox/points → 原始射线清除                  不经过后方过滤
/agt/livox/points → 障碍预处理 → 障碍标记           开启后方窄扇区过滤
```

旧故障运行的实际参数快照仍然是 false，保留作为历史证据；这次源配置变更在**下一次启动**时生效。
当前 C++ 节点只在构造时读取该开关，不应通过 `ros2 param set` 的返回值判断算法成员已热更新。

## 推荐启动：让日志和过滤统计放在同一目录

在宿主机终端执行。先正常退出旧栈，确认遥控器和急停可用、车辆及后方场地安全。

```bash
cd /home/yangxuan/ros2_ws/src/agt_navigation_v3
RUN_DIR="$HOME/.ros/agt_field_stack/fastlio2_rear_$(date +%Y%m%d_%H%M%S)"
AGT_FIELD_LOG_DIR="$RUN_DIR" ./scripts/run_field_stack.sh \
  --mode navigation --lio-backend fastlio2 --rviz \
  --obstacle-debug-base-cloud --obstacle-log-interval 5 \
  --obstacle-stats "$RUN_DIR/obstacle_filter_stats.yaml"
```

此命令会启动正常硬件/导航栈，由现场人员执行；本次代码修改过程没有代为运行它。

就绪后，在已经加载 ROS/工作区环境的另一个终端只读确认：

```bash
ros2 param get /agt_pointcloud_preprocessor rear_filter.enabled
ros2 topic echo /agt/odometry/adapter_status --once
ros2 topic echo /agt/localization/status --once
```

预期后方开关为 `true`，适配器为 FRESH，全局定位有效且 RViz 位姿正确。先静止查看，再发送跟踪目标。

## 看哪些结果

1. 对比原始 `/agt/livox/points`、过滤后的 `/agt/debug/points_obstacles_base` 和局部代价图，
   区分真实点云/致命占用与膨胀软代价，观察是否仍有点或障碍侵入车体轮廓。
2. 运行日志每 5 秒输出累计过滤统计；正常 `Ctrl+C` 退出后，统计文件应包含
   `rear_sector_enabled: true`。如果存在落入扇区且未被前级过滤掉的点，
   `rear_removed_points` 会增长。**零计数不一定表示过滤没开，非零计数也不等于问题已解决。**
3. 保留新运行目录、局部代价图/点云截图或视频，说明车辆当时静止还是转动、侵入发生在前后哪侧。
4. 若需要判断 FAST 轨迹质量，建议额外保留一段短的原始雷达+IMU 录包，并包含
   `/fastlio2/lio_odom`、`/agt/odometry/local`、`/agt/odometry/adapter_status`、
   `/agt/localization/status`、`/tf`、`/tf_static`、`/wheel/odom`、`/mux/cmd_vel` 和过滤前后点云。
   不要只保存文本日志。录包结束先正常停止 recorder，再关闭导航，避免数据库未完成写入。

本轮同时使用新的 FAST 前端与后方过滤，若结果改善，不能仅凭一次运行分离两者的贡献；
后续需要时可固定 FAST，只切换后方开关进行对照。现在不继续扩大扇区或一起调整其他参数。

## 安全与回退

**此扇区中的真实人员、树干或障碍也会被标记支路删除。** 原始 clearing 仍存在，并不等于这些
障碍仍能由 marking 看见。测试保持后方无人、场地受控、人工接管可用，尤其注意转动和后退恢复动作。

如需恢复关闭，将唯一配置源的 `rear_filter.enabled` 改回 `false`，在停止旧栈后更新安装配置并重启：

```bash
cd /home/yangxuan/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --executor sequential --packages-select agt_system_bringup
```

若回退后长期保留关闭状态，也应同步更新对应的试验默认值回归断言和文档；不要放宽碰撞保护来观察效果。

## 验证边界

本轮仅进行配置语义差异检查、构建、离线运行参数合成与相关回归测试，不启动实车或录包回放。
配置开关可正确进入新生成的 Nav2 运行参数，并不等于后方侵入问题已解决；效果以随后现场记录为准。


### 本次已执行的验证结果

- `agt_system_bringup` 构建成功。
- 87 项相关回归测试通过，`git diff --check` 通过。
- 从安装目录调用实际运行参数合成函数（没有启动 launch/节点），确认生成参数为
  `rear_filter.enabled: true`，角度/距离仍是上述值；原始 clearing、碰撞检测和定位门控保持。
- 语义比较确认 `perception.yaml` 的参数值只有这个开关发生变化。
- 没有启动机器人、调用控制服务或回放录包。实际过滤效果仍待现场测试。
