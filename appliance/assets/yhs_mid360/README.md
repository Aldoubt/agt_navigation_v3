# YHS 履带底盘 + 普通 MID360｜现场使用说明（中文）

> **当前状态：CAD 外参候选 / MID360 内置 IMU 厂家参数已填写 / YHS 实车未验收。**
> 这套配置可以用于检查 URDF、点云和建图流程，**不等于允许底盘运动**。真实 CAN、急停、驱动看门狗、几何尺寸、Nav2 路径与安全测试均须现场验证。

## 先看这里：今天下午怎么用

**如果电脑已经安装过 AGT YHS Control**，在已有 V3 仓库执行：
~~~bash
git switch feature/yhs-field-appliance-v1
git pull --ff-only
~~~

**如果是全新工控机**，先按 [中文安装说明](../../docs/installation.md) 安装：
~~~bash
git clone --branch feature/yhs-field-appliance-v1 https://github.com/Aldoubt/agt_navigation_v3.git
cd agt_navigation_v3
./install.sh
~~~

之后在 **V3 仓库根目录**执行以下命令，把云端文件复制到宿主机持久配置目录（不要在 Docker 镜像里直接编辑）：

~~~bash
mkdir -p "$HOME/agt/profiles/yhs/robot_description/urdf" "$HOME/agt/profiles/yhs/calibration"

# 候选 TF 模型：普通 MID360 + 15° 机械安装
cp appliance/assets/yhs_mid360/yhs_mid360_tf_nominal.urdf \
  "$HOME/agt/profiles/yhs/robot_description/urdf/"

# 官方 IMU 外参（作为 FAST-LIO 的初值）
cp -n appliance/assets/yhs_mid360/fastlio_config_mid360_candidate.yaml \
  "$HOME/agt/profiles/yhs/calibration/fastlio_config.yaml"

# 建图投影参数基线：人工审核后才能作为正式栅格
cp -n appliance/assets/yhs_mid360/projection_review_baseline.yaml \
  "$HOME/agt/profiles/yhs/calibration/projection.yaml"
~~~

注意：URDF 的 cp 会更新同名候选文件；上面两个 YAML 使用 \`cp -n\`，已有配置不覆盖。更换雷达安装姿态以后，不能继续拿原地图的 3D-BBS/Polar Context 资产测试而不重新审核地图版本。

编辑以下**宿主机**配置（只改相关字段，保留其余原字段）：

| 文件 | 需要设置的字段 |
|---|---|
| \`~/agt/profiles/yhs/robot.yaml\` | \`base_frame: base_link\`、\`rotation_frame: base_footprint\`、\`urdf: robot_description/urdf/yhs_mid360_tf_nominal.urdf\`；核查驱动后设置 \`lidar_frame: lidar_link\`、\`imu_frame: imu_link\` |
| \`~/agt/profiles/yhs/localization.yaml\` | \`fastlio_config: calibration/fastlio_config.yaml\` |
| \`~/agt/profiles/yhs/mapping.yaml\` | \`projection_config: calibration/projection.yaml\` |
| \`~/agt/profiles/yhs/sensors.yaml\` | 填真实网卡、电脑/雷达 IP、适配后的 Livox JSON 路径 |
| \`~/agt/profiles/yhs/base.yaml\`、\`topics.yaml\` | 填**真实 YHS ROS1 驱动**、CAN、odom、chassis、estop、超时；严禁照搬 Bunker |

### 要填的机器人配置示例（不是完整 robot.yaml）

~~~yaml
base_frame: base_link
rotation_frame: base_footprint  # V3 固定采用的平面运动参考坐标系名称
lidar_frame: lidar_link         # 先核对 /livox/lidar 的 header.frame_id
imu_frame: imu_link             # 先核对 /livox/imu 的 header.frame_id
urdf: robot_description/urdf/yhs_mid360_tf_nominal.urdf
~~~

**不要手工删掉其他原有字段**，例如 \`profile\`、\`map_frame\`、\`odom_frame\`。实机初次安装后，参数修改无需重新编译镜像，执行 \`./agt down && ./agt up\` 重新加载即可。先把 \`./agt doctor --report\` 中的缺项列出，不要将未测量项目写为 \`VERIFIED\` 来消除报错。

## rotation_frame 与 base_link 高度，究竟哪里还不确定？

简化的 TF 树是：

~~~text
map                       （定位管理器唯一发布 map → odom）
 └── odom
      └── base_footprint  （FAST-LIO adapter 的动态 TF 输出，rotation_frame）
           └── base_link （robot_state_publisher 的固定 TF）
                ├── lidar_link   （CAD：+15° Pitch）
                │    └── imu_link（MID360 厂商内部标定）
                ├── track_visual_link
                └── arm_mount_link
~~~

**① \`rotation_frame: base_footprint\` 是软件约定，名字已确定。** V3 FAST-LIO adapter 会输出 \`odom → base_footprint\`，所以不能随意改成 \`lidar_link\` 或 \`body\`。要验证的是**这个地面参考点是否落在真正适合平面运动的 X/Y 位置**，而不是名字本身。

**② \`base_footprint → base_link\` 当前 Z=+0.1041 m，只是网格最低点推算值。** 它不是人工测得的履带地面接触高度；XY=(0,0) 也只是临时假设。SolidWorks 原始 \`base_link\` 可能只是 CAD 装配原点，并非左右履带的运动学旋转中心。

**③ 如果需要重新定义 base_link，不能只改 Z。** 应保留 SolidWorks 的 \`chassis_cad_link\` 作为原始 CAD 基准，增加新的车体运动基准，按刚体变换重算 \`base_link → lidar_link\`。MID360 的内置 IMU 厂家外参不随底盘原点变化。

现场建议：先检查实际履带触地点、车宽、车长和安装架尺寸；安全条件下通过**物理遥控**做小幅转向，观察车身参考点的运动轨迹。转动中心对滑移履带可能随地面/速度变化，不能仅凭一次原地旋转就当成绝对不变值。测量前保持 CAD 状态，不要为了让 Nav2 启动而猜填数字。

### 当前 CAD + 厂商数据

| 变换 | 平移 xyz（m） | 旋转 rpy（rad） | 可信度 |
|---|---|---|---|
| \`base_footprint → base_link\` | [0, 0, +0.1041] | [0, 0, 0] | **候选，实测待确认** |
| \`base_link → lidar_link\` | [+0.346672506, 0, +0.589682277] | [0, +0.261799388, 0] | SolidWorks 标称，实装待核查 |
| \`lidar_link → imu_link\` | [+0.011, +0.02329, -0.04412] | [0, 0, 0] | 普通 MID360 厂商手册数据 |
| FAST-LIO 的 \`T_imu_lidar\` | [-0.011, -0.02329, +0.04412] | [0, 0, 0] | 上一行的逆变换 |

\`+0.261799388 rad\` 就是 **+15°**。这 15° **只在车体到雷达的 URDF 外参中体现一次**，不要再写进 FAST-LIO 的 \`r_il\`，也不要预先给点云软件调平。雷达点云坐标原点是否等同于 CAD 导出原点，还需通过安装基准或点云坐标方向验证。

## 不启动导航，先验证 TF 和传感器

1. 如果还没有机器环境，先 \`./agt up --mock\` 测 Qt（Mock 成功不代表能运动）。真机检查用 \`./agt up --headless\` 或桌面里的 **AGT YHS Control**。
2. \`./agt doctor --report\` 先看缺什么；\`CALIBRATION_REQUIRED\` 是预期的保护提示。
3. 在 Humble 容器里启动唯一的 \`robot_state_publisher\`（如果 Qt 已经启动它，**不要启动第二个**）：

~~~bash
docker compose -p agt-yhs -f appliance/docker-compose.yml exec runtime bash
source /opt/ros/humble/setup.bash
source /opt/nav_ws/install/setup.bash

# 仅在没有同名 robot_state_publisher 运行时执行：
ros2 run robot_state_publisher robot_state_publisher \
  /data/profiles/yhs/robot_description/urdf/yhs_mid360_tf_nominal.urdf
~~~

4. 在另一个 Humble 终端检查：

~~~bash
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros tf2_echo lidar_link imu_link

ros2 topic list -t
ros2 topic echo /livox/lidar --once --field header
ros2 topic echo /livox/imu --once --field header
~~~

若当前驱动发布的 \`frame_id\` 仍为 \`livox_frame\`，先核对驱动配置和 V3 映射关系；不能只改 \`robot.yaml\` 的字符串就假装 TF 对齐。

## 今天下午的执行顺序（先静态，再移动）

| 顺序 | 执行目标 | 过关条件 |
|---|---|---|
| ① 软件和设备 | \`./install.sh\` / Mock / doctor；检查 ROS1 驱动、CAN、物理急停 | Qt 可开、无危险命令、通讯真实 |
| ② TF 和 LIO | MID360/IMU/URDF、静止点云、FAST-LIO 输出 | 数据非空、时间戳新鲜、无重复 TF |
| ③ 建图 | **物理遥控**行驶；Stop & Build Map | bag 正常关闭，PGO 地图、patches/poses、校验完整 |
| ④ 地图编辑 | MapStudio 审核 PGM / 禁行区 → Confirm & Seal → Activate | Bundle 状态 READY、资产同源且 hash 一致 |
| ⑤ 冷启动重定位 | 入口/中段/末端进行多位置静止定位 | 3D-BBS + GICP 合理、无跳错行、唯一 \`map→odom\` |
| ⑥ 低速导航 | **仅当** watchdog / footprint / 外参 / 定位 / 急停均验收通过 | Nav2 单点→三点停顿→暂停/取消/停机 |

注意：现有 \`START_NAVIGATION\` 入口即使只想静止重定位，也要求完整实车运动配置；不要修改 Guard 或伪造 \`VERIFIED\`。没有达到完整门禁时，可完成建图、地图编辑和离线定位调试，暂停实车导航验证。具体验收见 [中文 R0–R13 检查表](../../docs/real_robot_acceptance.md)。

## 常见报错：先查哪里

| 现象 | 优先检查 |
|---|---|
| \`CONFIG_REQUIRED: urdf/fastlio_config/projection_config\` | 宿主机文件是否已复制、YAML 是否填写了**相对 profile 根目录**的路径 |
| \`CALIBRATION_REQUIRED\` | \`calibration/*.yaml\` 未经实测审核；是正常的安全门禁，不要随意改状态 |
| FAST-LIO adapter 拒绝 odom / \`static mount TF unavailable\` | 是否缺 \`base_footprint → base_link → lidar_link\`，有无另一个 TF 发布者或消息 \`frame_id\` 不符 |
| 没有 LiDAR / IMU | Ethernet 网卡、雷达 IP、Livox JSON、话题名称、ROS_DOMAIN_ID、时间戳 |
| ROS1 底盘有 odom，ROS2 不显示 | YHS vendor 话题类型是否匹配 Gateway，\`~/agt/logs/ros1-gateway.log\` |
| 地图不能激活 | PGO/keyframes、PCD/PGM、MapStudio review、bundle manifest/hash 是否完整 |
| Nav2 不运行 | 先检查 Motion Guard、Localization READY、YHS driver watchdog、footprint，而不是关闭保护 |

查看运行记录：

~~~bash
./agt status
./agt logs
./agt doctor --report
~~~

宿主机持久数据：\`~/agt/maps/\`、\`bags/\`、\`routes/\`、\`logs/\`、\`diagnostics/\`。删除 Docker 容器不会自动删除这些数据。

## 目录里每个文件做什么

| 文件 | 用途 |
|---|---|
| \`yhs_mid360_tf_nominal.urdf\` | ROS2 可读取、**不含 STL** 的 TF/简化外观模型；没有真实导航碰撞几何 |
| \`fastlio_config_mid360_candidate.yaml\` | 普通 MID360 内置 IMU 官方外参逆变换；滤波/噪声等参数仍是软件默认 |
| \`imu_extrinsics.reference.yaml\` | 厂商外参来源记录；不自动将标定状态标为 \`VERIFIED\` |
| \`projection_review_baseline.yaml\` | Mapping 原始地面投影配置；坡地、障碍高度阈值需要人工审核 |

完整 SolidWorks STL 原始压缩包并**未**包含在此目录。没有 STL 不影响本候选发布 TF，但不能用透明示意盒代替实测车体轮廓；机械臂相关 link 也只是刚体参考，不是完整关节驱动模型。

延伸阅读：[中文安装说明](../../docs/installation.md) · [实车待补信息](../../docs/REAL_ROBOT_TODO.md) · [实车 R0–R13 验收](../../docs/real_robot_acceptance.md) · [Qt 中文上位机指南](https://github.com/Aldoubt/agt_robot_hmi/blob/feature/yhs-field-appliance-v1/docs/YHS_FIELD_USER_GUIDE_ZH.md)。

**外参出处：** [Livox 普通 Mid-360 官方下载中心](https://www.livoxtech.com/cn/mid-360/downloads)，Mid-360 用户手册（2024-04）内置 IMU 相对点云坐标信息；安装倾角及平移来自用户 2026-10-08 SolidWorks 导出，尚未完成 YHS 实装验收。
