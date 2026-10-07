# YHS Field Appliance V1 — 交接

软件交付入口是 Navigation feature branch。Mapping 仍是独立地图生产仓库，Qt 使用独立可写 fork + 固定 commit；没有复制算法源码形成新的大一统仓库。真实硬件项目全部 PENDING。

## 明天第一步：按这 12 步执行

1. 在现场 Linux/Noetic 主机取得集成分支：
   ```bash
   git clone --branch feature/yhs-field-appliance-v1 https://github.com/Aldoubt/agt_navigation_v3.git
   cd agt_navigation_v3
   ```
   已有干净 checkout 则 fetch 后切换同名分支。Mapping 分支为 `Aldoubt/agt-lio-pgo-mapping:feature/yhs-field-appliance-v1`，安装器会取得锁定 commit。Qt 安装器取得独立 fork `Aldoubt/agt_robot_hmi` 的固定 commit `5ab45cc2cd4415e1d41329c23ab02877559340a9`；无需修改 Qt upstream。中文操作说明见 [Qt 中文使用说明](https://github.com/Aldoubt/agt_robot_hmi/blob/feature/yhs-field-appliance-v1/docs/YHS_FIELD_USER_GUIDE_ZH.md)。
2. 首次运行 `./install.sh`。先确保 Docker Engine/Compose 可用、操作员能执行 docker info，且宿主机有 python3-yaml/xauth。安装器保留已有数据/profile，写安装日志并建立桌面入口。详见 [安装说明](appliance/docs/installation.md)。
3. 启动：`./agt up`。先验纯软件可用 `./agt up --mock`，无显示服务器用 `./agt up --mock --headless`。
4. Qt：双击 **AGT YHS Control**，或从 Linux 桌面终端执行 `./agt up`；launcher 封装 X11 cookie、Compose 与运行检查。失败看 `~/agt/logs/desktop-launch.log`，不需要手工 xhost/source。
5. YHS 实际型号/帧定义填 `~/agt/profiles/yhs/robot.yaml`；底盘超时与 ROS1 master 填 base.yaml；速度、加减速、footprint、自过滤尺寸填 navigation.yaml。所有 null 是 CONFIG_REQUIRED，不能以旧 Bunker 数值替代。
6. URDF 放 `~/agt/profiles/yhs/robot_description/urdf/`，meshes 放相邻 meshes/；robot.yaml urdf 填相对路径。真实 URDF 不进入镜像，替换不需要重建 image。
7. MID360/IMU 外参填 calibration/lidar_extrinsics.yaml、imu_extrinsics.yaml、base_geometry.yaml、calibration_version.yaml；只有测量后才填 VERIFIED/verified_by/values。FAST-LIO 内部标定使用 templates/ 中说明，设置 localization.yaml fastlio_config。Livox JSON 和 projection YAML 也放 mounted profile，具体来源见 [模板说明](profiles/yhs/templates/README.md)。
8. CAN bitrate/interface 填 base.yaml can_bitrate/can_interface。CAN 按钮通过固定 host backend 操作；未知值禁止 CAN up。首次管理员授权可在宿主机终端 sudo -v 后执行 `./agt can up`，或由管理员配置窄范围 CAN 权限。
9. YHS topics 填 topics.yaml 的 ros1_cmd_vel/ros1_odom/ros1_chassis/ros1_estop。**当前没有 YHS vendor driver**：须先取得实际 ROS1 driver、记录 repo/commit、审计 watchdog/TF；自定义 chassis/estop 消息须在 ROS1 驱动侧适配为 String JSON/Bool。gateway 不猜 CAN 协议，也不移植驱动。
10. 首个诊断：`./agt doctor --report`。参数未填齐时 FAIL/CONFIG_REQUIRED 是预期，报告保存在 ~/agt/diagnostics。配置更新后 `./agt down`、`./agt up`；profiles 不需要 rebuild。
11. 首个无运动测试：物理急停有效、轮子不动，依次 R0/R1/R2：GUI/容器 → CAN → chassis/estop/odom。确认没有第二个 map→odom 或竞争 odom/base TF；不向驱动直接发送 cmd_vel。
12. 首个低速运动测试：先用物理遥控确认 odom/轮向/角速度符号；V3 Guard 要求定位 READY，因此软件 Qt teleop 延后到静态定位成功后，选择 **Manual Control (Motion Guard)**，在空旷区域使用已测量的低速限制测试；然后 Return to Navigation Control。严格按 [R0–R13](appliance/docs/real_robot_acceptance.md) 执行，不关闭 safety gate。

## 操作流程

System → Start MID360 → Preflight。Mapping 页填写新 map ID/version → Start Mapping → 用物理遥控行驶 → Stop & Build Map。后端调用 Mapping 正式 live wrapper，清洁 finish，等待 PGO/export/verify；随后调用 V3 原生资产 builders 和已有 PCD2Grid。不会从最终 map.pcd 切块伪造 keyframe。

Review Map 打开已有 MapStudio；在 MapStudio Confirm & Save 后关闭窗口，再在 Qt Confirm & Seal Bundle。3D 源资产与 localization patches/poses 不受 2D 编辑影响。READY 后 Activate。已有地图可 Refresh Map Bundles 选择 ID/version，再 Activate。

Navigation → Start Localization / Navigation Mode，等待 automatic 3D-BBS+GICP READY。现有 2D Pose Estimate 保留为 fallback/debug；路线第一个点不改变定位。任务表编辑 ID/X/Y/Yaw(rad)/Wait(s)/Action，添加/删除/上移/下移；Action 仅 wait。Save Route 后 Start Mission，支持 Pause/Resume/Cancel/STOP ALL。路线绑定 bundle ID/version/hash，错误地图拒绝运行。Recording topics 来自 profile，数据在宿主机 ~/agt/bags。

## 数据与合同

每个版本在 `~/agt/maps/<id>/<version>/`；mapping/ 保存校验通过的原始 map package，localization/ 是原生数据库/index及逐 keyframe provenance，navigation/ 是 grid/review。manifest 哈希覆盖文件和 review，root metadata 兼容 V3 Map Manager。每个 localization patch 保持原始 body cloud，pose 使用 PGO 最终优化 T_map_body。冻结文件只读；编辑必须新建版本。

V3 Motion Guard、Localization Manager、连续 tracking、Nav2 保留。Mission Executor 只调用 NavigateToPose，不发布速度或 TF。MID360 直接进 Humble runtime，不经过 ROS1 bridge。ROS1 gateway 只传 odom/chassis/estop/guarded cmd_vel，不发布 TF；断连/超时归零，进程死亡的最终停机还依赖真实 driver watchdog。

## 审计、验证和版本

[初始仓库审计](appliance/docs/P0_AUDIT.md) · [Localization 资产合同](appliance/docs/CURRENT_LOCALIZATION_ASSET_CONTRACT.md) · [第三方许可证事实](appliance/THIRD_PARTY_NOTICES.md) · [Gateway 决策与验证范围](appliance/docs/ros1_ros2_gateway.md)。软件证据在 appliance/evidence/，最终 Git/镜像记录见下方最终验收记录。完整机器证据可用 doctor --report 导出，不默认打包 rosbag。

已验证：appliance unit/mock、实际 ROS2 Action + Odometry/covariance 传输、原生 BBS/descriptor builder 合成 smoke、V3 相关回归、Mapping workflow、原生 PCD2Grid/MapStudio tests、完整 Qt5 ROS2 编译和 offscreen startup、Qt waypoint logic、pinned source patch repeat-apply、wheel、launcher/install/doctor smoke。探索性全 workspace tests 的未构建包和上游第三方 lint 失败单独记录，未伪装 PASS。

真实 YHS/CAN/MID360/TF/外参/3D-BBS+GICP 精度/Nav2/物理安全/30分钟 endurance 均 **PENDING**；下一步事项仅集中在 [REAL_ROBOT_TODO](appliance/docs/REAL_ROBOT_TODO.md)。

## 维护

AGENTS.md 是后续 AI 修改规则。开发模式使用 Dockerfile.dev、独立 source mounts、incremental colcon/Qt；不需要每改一行重建巨大 image。详见 [开发说明](appliance/docs/development.md)。不要绕过 Guard、制造第二个 map→odom、混 bundle、修改冻结 3D 数据、硬编码硬件参数，或把用户数据留在容器 writable layer。

## 最终验收状态

| 项目 | 状态与证据 |
|---|---|
| SOFTWARE_BUILD | PASS：Navigation 18 packages、Mapping 17 packages、完整 ROS2 Qt5、Python wheel |
| UNIT_TESTS | PASS：45 appliance tests（含 fork pin/migration/dirty preservation）；Qt CTest 1；V3 map regression 32；Mapping workflow 109 passed/4 skipped/18 subtests |
| MOCK_INTEGRATION | PASS：建图到三点 dwell 完成，定位丢失/Nav2失败/断连/取消 fail safely；最终 Docker RPC 记录见 evidence/docker_mock_runtime.txt |
| DOCKER_RUNTIME | PASS：最终镜像/CLI 验收结果及 image ID 见 evidence/docker_runtime_acceptance.txt |
| HMI | PASS 软件：Qt 完整编译、waypoint logic CTest、offscreen startup；云端没有实际 X11 桌面点击验收 |
| MAP_BUNDLE | PASS：hash/version/review/缺失资产 fail closed，原始 optimized keyframe provenance；真实地图建图质量 PENDING |
| MISSION_EXECUTOR | PASS：实际 ROS2 Action fixture、暂停/恢复/取消、dwell、failure/localization loss；真实 Nav2 planner/controller PENDING |
| GATEWAY | PASS 软件 Noetic→Humble 50.08 Hz，cmd transfer/timeout/monitor-only；CAN 延迟及 driver crash watchdog PENDING |
| DIAGNOSTICS | PASS 软件：doctor/report smoke；硬件状态如实 OFFLINE/CONFIG_REQUIRED/PENDING |
| REAL_YHS / REAL_CAN / REAL_MID360 | PENDING |
| REAL_TF / REAL_EXTRINSICS / REAL_3DBBS_GICP / REAL_NAV2 / REAL_SAFETY | PENDING |

Native runtime 完整 clean 构建日志来自 Dockerfile 的两阶段 candidate 构建。云端 Docker 使用 VFS、磁盘仅 32GB，最终镜像复用相同 native 编译产物后重新安装本分支 adapter、重建 ROS2 executor 和最终 Qt patch，并以已验证 wheel 修复增量旧 setuptools 打包结果、规范代码访问权限和持久化日志。最后扁平化来降低 VFS 磁盘占用；没有把所有本地改动声称为一次未经缓存的 clean build。现场正常 Docker overlay2 使用 install.sh 执行正式 Dockerfile。源码 commit/归档 SHA256 已锁定；apt package 内容尚非 snapshot，不能承诺 bit-for-bit image 重现。没有推送 container registry，也没有实车数据。

## Git 交付记录

所有初始 checkout 均 CLEAN，没有覆写用户 dirty 工作内容；三个仓库均建立 feature/yhs-field-appliance-v1。Navigation/Mapping 推送到用户可写 origin；后续 GitHub canonical URL 检查确认已有可写 fork `Aldoubt/agt_robot_hmi`。Qt 保留 upstream remote，在 fork 新增本任务 branch；未改动其 master/main/既有分支。安装器直接锁定 fork commit，历史 upstream 补丁保留作为 provenance，不重复应用。

| Repository | Remote | BASE_HEAD | 验证过的实现 HEAD |
|---|---|---|---|
| agt_navigation_v3 | https://github.com/Aldoubt/agt_navigation_v3.git | e27abeb3fd63280ee8e48a47966e06588f8eebd8 | 7ac545cf5179e01857aaf40389ca454a67df24e9，后续 commit 补验收脚本/证据/本交接文档 |
| agt-lio-pgo-mapping | https://github.com/Aldoubt/agt-lio-pgo-mapping.git | 6c40ff8bedab472b5cb0b914088fec1ecd566630 | 539352d309deef4f0acfe5f2bd783d5e72623a88 |
| Ros_Qt5_Gui_App | https://github.com/Aldoubt/agt_robot_hmi.git（origin；upstream 保留） | b0825e3cba3e7186cba8a6b83ff230be37c8b1fb | 035b6ac7bfab24be63abea979741f1a6a7185206 |

Navigation commits：81e4054 audit；a6b5b22 bundle/native assets；1bc4bb4 mission/gateway contracts；113177e lifecycle/review/recording；44aa33a calibrated localization/native tests；2442b4c Docker/CLI/Qt/doctor；eaa366f motion gates/ROS overlay；8239582 archive pins/native layers；7ac545c ordinary-UID startup/package validation。Qt：75e6d7e integration/dwell；8c53ede archive pins/controlled manual mode；5ab45cc 中文现场说明。Mapping：539352d external FAST-LIO mounted calibration。

最终文档与证据 commit 本身不可能在自身内容中写入自己的 SHA；pull 后用 git rev-parse HEAD / git log --oneline BASE_HEAD..HEAD 查看最终交付 HEAD 与全部提交，最终回复同时给出实际 HEAD。所有仓库最终 dirty 状态见最终回复；appliance/evidence 是保留的可审查输出。HANDOFF 以现场步骤为入口，审计与详细结构后置。

## 交付组件

| 组件 | 交付状态 |
|---|---|
| MAPPING_BACKEND | 独立 pinned Mapping wrapper、clean finish/PGO/export/verify 复用，mounted FAST-LIO calibration 支持；大型实测建图 PENDING |
| MAP_BUNDLE | 已实现 sealed identity/hash/version/review 与 fail-closed activation |
| LOCALIZATION_ASSETS | 原生 V3 descriptor + BBS builder、原始 body patches 与最终 optimized poses、逐 keyframe provenance |
| NAVIGATION_V3 | 原有 3D-BBS/GICP/tracker/LocalizationManager/Nav2/Guard 保留，YHS launch/profile composition |
| QT_HMI | 独立 writable fork + 固定 commit；upstream attribution/历史 patch 保留；5 tabs、runtime/device/mapping/map/mission/recording 控制 |
| WAYPOINT_DWELL | Qt add/delete/reorder/yaw/dwell/save/load，绑定 bundle，ROS2 executor 等待 Nav2 success 后 dwell |
| MISSION_EXECUTOR | 状态/命令、action cancellation barrier、定位/断连失败停止推进，不发布速度/TF |
| ROS1_GATEWAY | 标准 typed ROS1/ROS2 endpoints，50Hz 软件验证、freshness/estop/timeout、monitor-only |
| YHS_PROFILE | 全部未知物理参数 null/CONFIG_REQUIRED，URDF/外参/footprint/driver watchdog 门禁与 mount |
| DOCKER | 正式 release/dev Dockerfile、Compose、独立版本源、host-volume 数据、普通 UID 启动验证 |
| CLI | install/up/down/restart/status/can/doctor/logs/RPC、配置保留和启动错误显示 |
| DESKTOP_LAUNCHER | AGT YHS Control 已生成、固定 launcher；真实桌面 X11/双击作为 R0 验收 |
| DOCTOR | runtime + host probes、节点/topics/TF/CAN/profile/bundle/hash、诊断 zip（不带大型 bag） |
| MOCK_ACCEPTANCE | Python unit/mock + 实际最终 Docker RPC 完整链路及异常、Qt offscreen、实际 Noetic/Humble fixture |

## Qt fork 初次独立发布记录（历史）

独立仓库：https://github.com/Aldoubt/agt_robot_hmi/tree/feature/yhs-field-appliance-v1 。中文使用说明涵盖安装、Mock、真实配置、建图、MapStudio、激活、定位、多航点停顿、任务控制、录制、诊断与 R0–R13。Qt 新 commit 仅增加文档，C++ 与前述编译/CTest/offscreen 验证内容相同。

repos.lock.yaml 现在直接 pin fork `5ab45cc`，不把已包含的 patch 再应用一次。旧的、精确匹配已验证补丁的 sources/hmi 会保留到 sources/hmi.upstream-patch-<base> 再取得 fork；未知修改不覆盖。不影响 maps/routes/bags/profiles。源准备的直 pin、重复运行、旧缓存迁移和 dirty 拒绝已自动测试。

当前云端已运行镜像仍是前述验收版本；此更新未重建巨大 native image。安装器准备 fork 已实测，CLI wheel 构建/45 unit tests 已验证。现场首次 install.sh 使用新 fork pin；新 fork 没有改变已验证的 Qt C++ 行为。apt snapshot / 实车验收边界不变。

## 中文 Qt 界面更新

控制面板与航点表已中文化，并与原上位机浅色、蓝色按钮、圆角边框风格一致。五页为系统、建图、导航、录制、诊断。底部“停止全部任务”固定显示；长页面滚动，旧 dock 布局过窄时启动自动修复。实际主程序截图与独立 fork 中文使用说明已更新。

Qt 完整主程序增量构建、Qt CTest 与 45 appliance tests 通过。实际 UID 1000 offscreen 截图通过；真实 X11 交互与实车验收仍为 PENDING。具体证据与镜像限制见 [中文界面验证](appliance/evidence/HMI_CHINESE_STYLE.md)。

取得最新集成分支后执行 `./install.sh` 再 `./agt up --mock`；正式安装会编译新的固定 Qt fork 并安装中文字体。当前旧镜像未重建，不能用 `--skip-build` 获取这次界面更新。当前运行容器已增量编译新界面用于验证。

当前 Qt fork pin：`035b6ac7bfab24be63abea979741f1a6a7185206`（中文界面与建图阶段标签测试）。
