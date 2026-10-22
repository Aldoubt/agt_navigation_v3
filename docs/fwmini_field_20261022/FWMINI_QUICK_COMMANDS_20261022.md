# FW-mini 快速命令卡

详细说明见 FWMINI_MAPPING_LOCALIZATION_NAV_WORKFLOW_20261022.md。以下命令均在真实 Ubuntu 22.04 + ROS 2 Humble 工控机执行。

## A. 实时建图

    bash ~/ros2_ws/scripts/fwmini_start_live_mapping.sh --check
    bash ~/ros2_ws/scripts/fwmini_start_live_mapping.sh

正常结束建图，在另一个终端执行：

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/ros2_ws/install_mapping_framework/setup.bash
    ros2 service call /mapping/session/finish std_srvs/srv/Trigger "{}"

## B. 地图清洗与资产准备

不手工清洗，直接用同一次建图包的关键帧做持久性过滤：

    RUN="$(cat ~/ros2_ws/experiments/fwmini_last_live_run.txt)"
    bash ~/ros2_ws/scripts/fwmini_prepare_map.sh "$RUN"

手工清洗后，假设清洗结果保存为 $RUN/cleaned_global_map.pcd：

    RUN="$(cat ~/ros2_ws/experiments/fwmini_last_live_run.txt)"
    bash ~/ros2_ws/scripts/fwmini_prepare_map.sh "$RUN" "$RUN/cleaned_global_map.pcd"

## C. 导航阶段驱动

终端 A：MID360 + IMU

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -p xfer_format:=1 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=livox_frame -p user_config_path:=/home/yangxuan/ros2_ws/experiments/fwmini_live_preflight_20261021/session_20261021T082953Z/config/mid360_actual_145.json

终端 B：FW-mini CAN

    source /opt/ros/humble/setup.bash
    source ~/fwmini_ws/project/install/vendor/setup.bash
    source ~/fwmini_ws/project/install/yhs_driver/setup.bash
    source ~/fwmini_ws/project/install/project/setup.bash
    ros2 launch yhs_can_control yhs_can_control.launch.py

终端 C：FAST-LIO2 + 3D-BBS/GICP + Nav2

    RUN="$(cat ~/ros2_ws/experiments/fwmini_last_live_run.txt)"
    MAP_VERSION="$(basename "$RUN" | cut -d_ -f3-)"
    bash ~/ros2_ws/scripts/fwmini_start_nav.sh "$MAP_VERSION"

先静止等待自动重定位。确认 RViz 的地图、点云、机器人姿态和 TF 一致后再使能运动。

## D. 速度观察与运动门控

    ros2 topic echo /cmd_vel
    ros2 topic echo /agt/base/cmd_vel
    ros2 topic echo /ctrl_cmd
    ros2 topic echo /chassis_info_fb

定位和 Nav2 状态确认后，才允许手动使能：

    ros2 service call /fwmini/enable_motion std_srvs/srv/SetBool "{data: true}"

测试后立即禁用：

    ros2 service call /fwmini/enable_motion std_srvs/srv/SetBool "{data: false}"

## 当前首轮约束

- 4T4D 档位值为 06；横移档位值为 08。
- 首轮 Nav2 横向速度关闭，只测前进和转向。
- 速度适配器默认运动禁用；启动命令不会自动开动车辆。
- 新地图是 FAST-LIVO2 LIO-only 候选，没有 PGO/回环优化；先验证闭环，不把第一轮结果宣称为精度验收。
