# YHS Field Appliance：中文安装与启动指南

> 适用于 **YHS ROS1 Noetic 底盘 + Docker 内 ROS2 Humble + 普通 Livox MID360 + Qt5 上位机**。本指南只说明如何安装与验证软件，不能代替实车驱动、机械几何或安全验收。

## 0. 只想知道执行什么命令？

第一次在新的 Linux 工控机安装：

~~~bash
# 主机需预装 Docker Engine / Compose v2，并且 docker info 可用
sudo apt-get update
sudo apt-get install -y git python3-yaml python3-venv xauth iproute2 can-utils

git clone --branch feature/yhs-field-appliance-v1 https://github.com/Aldoubt/agt_navigation_v3.git
cd agt_navigation_v3
./install.sh

# 先测试纯软件。没有显示器时加 --headless。
./agt up --mock
./agt doctor --mock --report
./agt down
~~~

需要接实际传感器和底盘时，先完成 [普通 MID360 URDF / 外参中文说明](../assets/yhs_mid360/README.md) 和 [实车待补参数表](REAL_ROBOT_TODO.md)，然后再执行：

~~~bash
./agt up                   # 有正常 X11 桌面
# 或 ./agt up --headless   # 只有远程终端，没有桌面
./agt doctor --report
~~~

**上述指令不等于授权底盘运动。** 只有真实安全门禁、硬件急停、driver watchdog 通过验证，且定位 READY 后才能进入低速运动验收。

## 1. 系统应该长什么样？

| 部分 | 实际所在位置 | 作用 |
|---|---|---|
| Ubuntu 20.04 + ROS1 Noetic | **工控机宿主机** | 运行 YHS 厂商驱动、ROS1 Master、CAN |
| Ubuntu 22.04 + ROS2 Humble | **Docker 容器** | MID360 驱动、建图、重定位、Nav2、任务管理 |
| ROS1 ↔ ROS2 Gateway | 宿主机进程 + 容器内进程 | 只桥接标准底盘里程计/状态/急停及 Motion Guard 输出速度 |
| Qt5 HMI | Docker 的 hmi 服务，通过宿主 X11 显示 | 点击启动、建图、地图编辑、航点停顿、录包 |

**普通 MID360 是 Ethernet/UDP 设备，不是串口；它直接进入 ROS2，不通过 ROS1 Gateway 转发点云。**

### 安装前检查

~~~bash
lsb_release -a
test -f /opt/ros/noetic/setup.bash && echo "ROS1 Noetic OK"
docker info
docker compose version
df -h
~~~

确保 Docker Engine 与 Compose **v2 插件**已按官方包安装，且当前登录用户能运行 `docker info`。安装器不会自动安装未知 YHS 驱动、修改 CAN 波特率或开放大范围 sudo 权限。若 Ubuntu 版本不是 20.04，请先核对与 Noetic 驱动的兼容性。

## 2. 第一次安装发生了什么？

在 `feature/yhs-field-appliance-v1` 分支执行 `./install.sh`。安装程序将：

1. 创建 `~/agt/{maps,routes,bags,logs,profiles,diagnostics,run,sources}`；
2. 按 `appliance/repos.lock.yaml` 获取锁定版本的 Mapping 和独立 Qt fork；
3. 构建 `agt-yhs-base:v1`、`agt-yhs-field:v1` Docker 镜像；
4. 在桌面应用菜单添加 **AGT YHS Control**；
5. 保留之前的地图、路线、录包、外参和 profile。

第二次安装不会主动覆盖 `~/agt/` 下的现场资料。仅在已有**确定可用且包含本次代码**的镜像时，才能使用 `./install.sh --skip-build`，否则仍应完整构建。源码变更需重新构建镜像；宿主机的 YAML、URDF、标定数据修改后**通常仅需重启 Runtime**。

### 为什么不能单独启动 Qt？

Qt 通过受控的 Runtime API、Map Bundle Manager 和 Mission Executor 调用真实功能。仅运行普通的上游 Ros_Qt5_Gui_App，没有这套 YHS 后端就无法安全完成重定位、Map Bundle 校验与任务停顿。**禁止 Qt 速度话题直连 YHS 驱动**。

## 3. 先跑 Mock，再跑实机

桌面 Mock：

~~~bash
./agt up --mock
./agt status
./agt doctor --mock --report
./agt down
~~~

SSH/无桌面 Mock：

~~~bash
./agt up --mock --headless
PYTHONPATH=appliance python3 appliance/scripts/accept_mock_runtime.py --data-root "$HOME/agt"
./agt doctor --mock --report
./agt down
~~~

注意：Mock 数据只供软件测试，不代表 CAN、Livox、GICP 精度或物理停车通过。若宿主机已接了实际 YHS 厂商驱动，Mock 也应在安全隔离条件下使用，避免干扰实际控制。

## 4. 真机配置要填在哪里？

全部位于**宿主机** `~/agt/profiles/yhs/`，不是 `~/ros2_ws/src` 或 Docker 镜像内部。

| 文件 / 路径 | 配置项目 |
|---|---|
| `robot.yaml` | YHS 型号、`base_link`、`base_footprint`、雷达/IMU frame、URDF 相对路径 |
| `base.yaml` | CAN 网卡/波特率、ROS1 Master、超时/驱动 watchdog、实际停车阈值 |
| `topics.yaml` | ROS1 厂商 cmd_vel、odom、chassis、estop 与 ROS2 录制话题 |
| `sensors.yaml` | MID360 的网卡、Host IP、LiDAR IP、外部 Livox JSON |
| `localization.yaml` | FAST-LIO 配置相对路径及定位状态约束 |
| `mapping.yaml` | PCD2Grid 投影参数路径、建图 Domain 和导出超时 |
| `navigation.yaml` | 实测 footprint、自过滤盒、地面参考、限速/加减速 |
| `calibration/*.yaml` | LiDAR/IMU/底盘几何、校验人员/版本/证据 |
| `robot_description/urdf`、`meshes` | URDF 与 STL 外观网格 |

普通 MID360 的 CAD 15° 外参与厂家 IMU 参数已经在云端：[点这里按中文说明复制](../assets/yhs_mid360/README.md)。

**`null`、`CONFIG_REQUIRED`、`CALIBRATION_REQUIRED` 是未验收状态。** 不能复制旧 Bunker 几何、猜 CAN 参数或只把 status 改成 `VERIFIED`。

配置变更后：

~~~bash
./agt down
./agt up             # 或 ./agt up --headless
./agt doctor --report
~~~

## 5. 启动、停止、日志、CAN

| 命令 | 作用 |
|---|---|
| `./agt up` | 启动实体 Runtime 和 Qt（要求 X11 DISPLAY） |
| `./agt up --headless` | 无显示器启动实体 Runtime |
| `./agt up --mock` | 软件模拟模式 |
| `./agt down` | 先尝试 STOP_ALL，再关闭容器和宿主相关进程 |
| `./agt status` | 查询 Runtime 状态与 Compose 容器 |
| `./agt logs` | 查看最近容器日志 |
| `./agt doctor --report` | 生成含配置/运行信息的诊断 ZIP，不打包大型 rosbag |
| `./agt can up` / `./agt can down` | 使用已验证的 CAN 接口和 bitrate 做受控操作 |

**CAN 操作需要宿主权限**。程序默认使用固定 `ip link` 参数、`sudo -n`，不会在 Qt 内提示输入 sudo 密码。首次需要管理员授权时，在终端完成，或者按实际运维要求配置只允许特定 `ip link` 命令的最小 sudo 权限。不要给 Docker 容器特权或挂载 Docker socket 来省事。

## 6. Qt 双击没反应、启动报错怎么办？

按顺序运行：

~~~bash
./agt status
./agt logs
./agt doctor --report
cat "$HOME/agt/logs/desktop-launch.log"
~~~

重点查看：

- `DISPLAY` / `XAUTHORITY`：实际登录桌面需要可用的 X11 cookie，Launcher 不调用不安全的 `xhost +`；
- `docker info`：操作员是否有权限使用 Docker；
- `~/agt/profiles/yhs/`：路径是否存在，URDF、Livox JSON、FAST-LIO YAML 是否指向真实文件；
- `ros1-gateway.log`：ROS1 driver 是否正常启动，标准消息类型是否匹配；
- `CALIBRATION_REQUIRED`：属于预期安全阻断，不能当成编译错误；
- 底盘或雷达不存在：诊断应显示 OFFLINE/CONFIG_REQUIRED，而不是假装在线。

## 7. 路线/地图的数据保存在哪里？

| 数据 | 路径 |
|---|---|
| 原始/封存地图及定位资产 | `~/agt/maps/` |
| 航点路线 | `~/agt/routes/` |
| 录包 | `~/agt/bags/` |
| 程序与驱动日志 | `~/agt/logs/` |
| 标定和底盘参数 | `~/agt/profiles/yhs/` |
| 故障报告 | `~/agt/diagnostics/` |

这些目录挂载在宿主机，删除或重建容器不会自动删除。**不要直接修改已封存地图包**；MapStudio/Qt 编辑需要创建并校验新版本。

## 8. 现场完整验收顺序

先 [实车待补参数](REAL_ROBOT_TODO.md)，再按 [R0–R13 中文验收表](real_robot_acceptance.md) 逐项进行。实现功能、运行 Mock、完成一次 GICP 对齐都不等于可以直接运行真实底盘。长时间站人、动态遮挡、温室重复行道、坡地地面参数都建议在实际测试时保留录包。

## 9. 版本与开发备注

代码版本固定在 `appliance/repos.lock.yaml`。导航集成取当前 feature checkout，Mapping、HMI 由锁定 commit 构建；记录 `git rev-parse HEAD`、Docker image ID 和校验报告以便追溯。系统 apt 软件包尚未全部 snapshot 固定，因此不能承诺每次安装获得字节级相同的镜像。

云端构建/Mock 已有证据，但**工控机真实 X11、ROS1 YHS 驱动、CAN、MID360、物理急停和完整导航均仍待现场验证**。

[Qt 中文操作指南](https://github.com/Aldoubt/agt_robot_hmi/blob/feature/yhs-field-appliance-v1/docs/YHS_FIELD_USER_GUIDE_ZH.md) · [普通 MID360 外参与 TF](../assets/yhs_mid360/README.md) · [实机验收 R0–R13](real_robot_acceptance.md)
