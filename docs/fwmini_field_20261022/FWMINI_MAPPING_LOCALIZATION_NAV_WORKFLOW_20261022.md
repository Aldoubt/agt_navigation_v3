# FW-mini：实时建图 → 新地图重定位 → Nav2 实车测试

更新日期：2026-10-22。目标是直接在真实 FW-mini + MID360 上跑通一条可复制的现场流程，不依赖旧 rosbag 回放来建图。

## 0. 当前已经核对的配置

- Humble 工控机：192.168.1.5；MID360：192.168.1.145。
- 实时建图配置：~/ros2_ws/experiments/fwmini_live_preflight_20261021/session_20261021T082953Z/config/mid360_actual_145.json。
- 实时建图使用 agt_mapping_bringup 的 mapping_live_yhs_mid360.launch.py，启动 MID360 + IMU + FAST-LIVO2 LIO-only；不启动 CAN、底盘运动或 Nav2。
- 建图完成后会自动保存 map_package/，并保留 raw_bag/ 作为故障备份。旧录包不是建图前置条件。
- 当前实时建图后端是 LIO-only，没有 PGO/回环优化。第一份地图用于端到端重定位和导航试跑，不代表绝对精度验收通过。
- 当前 FW-mini 雷达外参在 ~/fwmini_ws/worktrees/fwmini-nav2-agt-cmd-bridge/src/fwmini_multi_nav_bringup/config/field_trial/fwmini_mid360_extrinsic.yaml，状态为 SPIN_FIT_CANDIDATE_NOT_FIELD_VERIFIED。首轮沿用昨天记录的候选值，不在本轮重新标定。
- CAN 速度单位链已核对：Nav2 m/s → 适配器 m/s → 厂家 CAN 驱动按 0.001 m/s/bit 编码；Nav2 rad/s → 适配器 °/s → 厂家驱动按 0.01 °/s/bit 编码。适配器单元测试：2 tests，0 failures。
- 已将首轮底盘档位改为手册规定的 ctrl_cmd 档位 06 = 4T4D，并将 Nav2 横向速度采样关闭。首轮只验证前进/转向，不混入横移模式。
- can0 当前已检查为 UP、500000 bit/s。启动时脚本仍会再次检查。

## 1. 启动实时传感器和建图

在 Ubuntu 22.04 + ROS 2 Humble 工控机终端执行：

    bash ~/ros2_ws/scripts/fwmini_start_live_mapping.sh --check
    bash ~/ros2_ws/scripts/fwmini_start_live_mapping.sh

第二条命令会自动：
1. 加载 FAST-LIVO2、ROS 2 工作区和 mapping framework 环境。
2. 检查 MID360 网络配置、主机 IP 和是否已有另一份 Livox driver。
3. 启动 MID360 + IMU + FAST-LIVO2 实时建图。
4. 实时生成地图并保存原始数据备份。

不要同时启动第二份 Livox driver。建图过程中可以正常走一圈覆盖测试区域，尽量回到起始区域；本轮不启动底盘 CAN 控制或 Nav2。

正常结束时，在另一个终端（使用相同 ROS_DOMAIN_ID）执行：

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/ros2_ws/install_mapping_framework/setup.bash
    ros2 service call /mapping/session/finish std_srvs/srv/Trigger "{}"

等待建图终端报告 Verified map 并自动退出。不要用 Ctrl+C 代替正常结束。

本次输出目录会写入：

    cat ~/ros2_ws/experiments/fwmini_last_live_run.txt

## 2. 检查地图、清洗并生成重定位资产

先检查新地图是否存在：

    RUN="$(cat ~/ros2_ws/experiments/fwmini_last_live_run.txt)"
    ls -lh "$RUN/map_package/map.pcd" "$RUN/map_package/poses.txt"

在现有 Map Studio 中查看 $RUN/map_package/map.pcd。如果确实有动态人影、拖影或明显离群点，将清洗后的点云另存为：

    $RUN/cleaned_global_map.pcd

只清理明显动态杂物，不要误删立柱、墙面、行道边缘等稳定结构。原始 map_package/ 不要覆盖。

如果不需要人工清洗：

    RUN="$(cat ~/ros2_ws/experiments/fwmini_last_live_run.txt)"
    bash ~/ros2_ws/scripts/fwmini_prepare_map.sh "$RUN"

如果已经导出清洗后的点云：

    RUN="$(cat ~/ros2_ws/experiments/fwmini_last_live_run.txt)"
    bash ~/ros2_ws/scripts/fwmini_prepare_map.sh "$RUN" "$RUN/cleaned_global_map.pcd"

脚本会自动执行：
- PCD → Nav2 navigation/map.yaml + map.pgm。
- 从所选 PCD 生成 3D-BBS 体素资产。
- 从同一次建图的 poses.txt + patches/ 生成 Polar Context 候选数据库。
- 创建新的不可变地图包，不覆盖已有版本。

新地图包路径为：

    ~/ros2_ws/maps/fwmini_mid360/<MAP_VERSION>/
    ├── metadata.yaml
    ├── localization/global_map.pcd
    ├── localization/relocalization/
    └── navigation/map.yaml

脚本结束时会打印 MAP_VERSION。如果候选数据库构建失败，先检查地图是否太短、有效关键帧是否不足；不要复用旧地图的定位资产。

## 3. 导航前启动硬件驱动

建图 launch 结束后，它拥有的 Livox driver 会一起退出。导航阶段需要重新启动一份 MID360 driver 和一份 YHS CAN driver。分别在两个终端启动。

### 终端 A：MID360 + IMU

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash

    ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args       -p xfer_format:=1       -p multi_topic:=0       -p data_src:=0       -p publish_freq:=10.0       -p output_data_type:=0       -p frame_id:=livox_frame       -p user_config_path:=/home/yangxuan/ros2_ws/experiments/fwmini_live_preflight_20261021/session_20261021T082953Z/config/mid360_actual_145.json

### 终端 B：FW-mini CAN 驱动

    source /opt/ros/humble/setup.bash
    source ~/fwmini_ws/project/install/vendor/setup.bash
    source ~/fwmini_ws/project/install/yhs_driver/setup.bash
    source ~/fwmini_ws/project/install/project/setup.bash

    ros2 launch yhs_can_control yhs_can_control.launch.py

启动前确认急停已释放、can0 为 UP/500000 bit/s，遥控器切到指令控制模式。手册说明遥控器有优先控制权，模式不正确时 CAN 指令可能不会按预期控制底盘。

## 4. 启动新地图重定位 + Nav2

将第二阶段生成的版本号填入命令：

    bash ~/ros2_ws/scripts/fwmini_start_nav.sh <MAP_VERSION>

例如版本号为 20261022_120000：

    bash ~/ros2_ws/scripts/fwmini_start_nav.sh 20261022_120000

该脚本会检查新地图包、MID360 driver、YHS CAN driver 和 can0，然后启动：
- FAST-LIO2 连续跟踪。
- 3D-BBS + GICP 自动全局重定位。
- 新地图的 Nav2 map server。
- Nav2 规划与控制。
- FW-mini 50 Hz 命令适配器及命令看门狗。

启动后先让机器人保持静止，等待自动重定位结束。检查 RViz 中的地图、点云、机器人姿态和 TF 是否一致，再检查状态：

    ros2 topic echo /agt/global_relocalization/status
    ros2 topic echo /agt/localization/status
    ros2 topic echo /fwmini/navigation_guard/status
    ros2 topic echo /agt/odometry/adapter_status

如果没有进入有效定位状态，不要发送运动命令。自动重定位失败时先记录状态和日志；不要直接把失败状态当作定位成功。

## 5. 检查速度输出并开始低速测试

观察 Nav2 输出、门控后的速度、厂家 ROS CAN 指令和底盘反馈：

    ros2 topic echo /cmd_vel
    ros2 topic echo /agt/base/cmd_vel
    ros2 topic echo /ctrl_cmd
    ros2 topic echo /chassis_info_fb

/ctrl_cmd 中的 X/Y 线速度单位是 m/s，角速度字段为 °/s；厂家 CAN 驱动负责按协议缩放和打包。首轮使用 4T4D 模式，因此 Nav2 的横向速度已设为 0，先验证前进与转向。

只有在以下条件满足后才使能底盘：
- 重定位状态有效，RViz 中地图、点云、机器人姿态和 TF 一致。
- Nav2 节点已 active，速度输出在配置限幅内。
- 操作者在车旁，急停可立即触达，周围有足够空旷空间。

使能：

    ros2 service call /fwmini/enable_motion std_srvs/srv/SetBool "{data: true}"

随后在 RViz 发一个短距离、低速目标，观察 /cmd_vel、/ctrl_cmd 和 /chassis_info_fb 是否一致。测试后立即禁用：

    ros2 service call /fwmini/enable_motion std_srvs/srv/SetBool "{data: false}"

本流程不会自动使能运动。FW-mini 手册说明底盘本体不提供完整自主避障安全功能，Nav2 costmap 不能替代现场急停和人工监护。

## 6. 当前未完全确认的两件事

1. base_link → livox_frame 外参仍是昨天的旋转拟合/CAD 候选，尚未完成独立现场验收。本轮先使用它跑通链路，精度差时再单独测量和调整。
2. 本轮 FAST-LIVO2 地图没有 PGO/回环优化。它适合先跑通重定位和低速导航，不应直接作为绝对精度验收地图。若长路线出现地图弯曲或闭环错位，下一步应解决地图优化，而不是先换掉整个导航栈。

本轮没有启动实车运动，也没有宣称重定位精度已通过。
