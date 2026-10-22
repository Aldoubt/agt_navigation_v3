# FW-mini 现场精度试验（2026-10-22）

## 完成的统一

- FW-mini 唯一现场几何：~/fwmini_ws/worktrees/fwmini-nav2-agt-cmd-bridge/src/fwmini_bringup/config/geometry.field_trial.yaml
- 车体 0.495 x 0.360 x 0.320 m；轴距 0.350 m；轮距 0.250 m；轮径 0.140 m。
- base_link → livox_frame：xyz=(0.146725, 0.002608, 0.37704)m，rpy=(0, 0.2152, 0)rad（20261021 用户旋转拟合+CAD候选）。
- RobotModel/URDF、导航启动 TF、Nav2 footprint 对应同一组参数；V3 FW-mini 预览模型同步为此候选，未声称外参独立验收通过。
- --trial-motion 显式开放现场速度输出权限，保留原 0.2 s 命令超时、50 Hz 输出、限速和停止服务。
- 自动 Nav2 仅在全局定位状态有效时放行；局部里程计有效但未定位时，允许独立人工搜索话题 /fwmini/search_cmd_vel，限幅 x<=0.10m/s，y=0，yaw<=0.20rad/s。

## 已验证

- 4 个 FW-mini ROS 2 包重新构建 PASS；fwmini_chassis_adapter 单元测试 2/2 PASS。
- xacro 生成的 livox_frame 与标定候选一致，Nav2 footprint 与底盘尺寸一致，V3 预览也一致。
- 隔离 ROS Domain 211 无 CAN 驱动：局部里程计新鲜时人工搜索有速度输出；命令超时停车；无全局定位时普通 Nav2 命令被拒；里程计超时停车。均 PASS。
- 尚未进行实车建图/驱动运动或重定位正确率验收。

## 当前唯一已确认的直接现场故障：MID360 网络未连通

- 当前 ssh 通过 Wi-Fi 192.168.0.117；3 个有线口都是 NO-CARRIER。
- MID360 IP 为 192.168.1.145，Livox JSON 的 host IP 为 192.168.1.5。
- 主机当前没有 192.168.1.5，192.168.1.145 ping 不通，路由经 Mihomo TUN。
- 建图前连接 MID360 网线，恢复物理有线网卡的 192.168.1.5/24，核对 ip route get 192.168.1.145 应显示有线网口，而不是 Mihomo。完成之后运行启动脚本即可，不用重做标定。

## 1. 新建图

在实际 Ubuntu22.04/Humble 终端：
    bash ~/ros2_ws/scripts/fwmini_start_live_mapping.sh --check
    bash ~/ros2_ws/scripts/fwmini_start_live_mapping.sh

正常结束建图（同一 ROS_DOMAIN_ID 的第二终端）：
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/ros2_ws/install_mapping_framework/setup.bash
    ros2 service call /mapping/session/finish std_srvs/srv/Trigger "{}"

用新地图生成 PGM/BBS/候选索引：
    RUN="$(cat ~/ros2_ws/experiments/fwmini_last_live_run.txt)"
    bash ~/ros2_ws/scripts/fwmini_prepare_map.sh "$RUN"

如需先清洗点云，Map Studio 输出 $RUN/cleaned_global_map.pcd 后运行：
    bash ~/ros2_ws/scripts/fwmini_prepare_map.sh "$RUN" "$RUN/cleaned_global_map.pcd"

## 2. 启动新地图导航及运动

分别启动 MID360 和厂家 CAN 驱动（具体命令见 ~/ros2_ws/docs/FWMINI_QUICK_COMMANDS_20261022.md）。

终端 C：
    RUN="$(cat ~/ros2_ws/experiments/fwmini_last_live_run.txt)"
    MAP_VERSION="$(basename "$RUN" | cut -d_ -f3-)"
    bash ~/ros2_ws/scripts/fwmini_start_nav.sh "$MAP_VERSION" --trial-motion

--trial-motion 表示有新鲜的人工/导航指令时可以输出限速运动；不传此选项则默认关闭运动。它不会自动发送非零指令。

还未获得全局定位时，车旁操作者可以用键盘在局部里程计有效的情况下移动搜索（另一个终端）：
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/fwmini/search_cmd_vel

搜索成功后停止键盘遥控，再在 RViz 发短距离的 Nav2 目标。普通 Nav2 速度指令仍需正确的全局定位才会放行。

随时停用软件运动：
    ros2 service call /fwmini/enable_motion std_srvs/srv/SetBool "{data: false}"

观察：
    ros2 topic echo /fwmini/navigation_guard/status
    ros2 topic echo /agt/localization/status
    ros2 topic echo /agt/base/cmd_vel
    ros2 topic echo /ctrl_cmd
    ros2 topic echo /chassis_info_fb

## 3. 验证跟踪与重定位

- 跟踪：直线、弧线、闭环各至少 3 次；比较外部可测位置与 LIO 预测轨迹，统计终点位置、航向和随距离增长的漂移。
- 重定位：固定结构、重复行道、边缘区各做 10 次；记录成功/拒绝/错误接受、耗时、位置和航向误差。独立参考位姿不可使用被测 LIO 自己的结果。
- 特别注意 V3 现有全局定位旧 pose 兼容路径与候选歧义默认参数仍存在历史误接受风险。未修改接受标准，也不能把一次 GICP 成功等同于已验证的全局位置正确。
