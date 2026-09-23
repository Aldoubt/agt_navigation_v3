# Navigation Runtime V4 迁移与验收报告（R9，SSH 复核更新）

**更新：**2026-09-23T23:45:01+08:00（Asia/Shanghai）

## 环境、基线与保护

- 直接 ShunCode MCP 终端缺少 ROS；本次以**已记录主机密钥、严格校验、非交互公钥 SSH** 登录宿主机 `yangxuan-Default-string`。宿主机为 Ubuntu 22.04、ROS 2 Humble、Python 3.10，具备 `colcon`、`rclpy`、`pytest`、PyYAML。测试设置 `ROS_DOMAIN_ID=201`（新增 SDK 构建使用 203）、`ROS_LOCALHOST_ONLY=1`；未启动硬件驱动、未向机器人发送目标、未发布正式地图。
- SSH 宿主机核对到原始 R0 基线 `/home/yangxuan/ros2_ws_audit/navigation_v4/baseline.md`，SHA-256 `7f944def2e7eb8dac529e4e1ee36da656d678aa469e0fa2b8be4f86c5fb44b49`；原始 74 包及进入改造前的八项 Navigation 改动、三项 HMI 已跟踪删除均有记录。当前 `colcon list --names-only --base-paths src` 为 **80 行、80 个唯一包**。与 R0 集合对比，原 `tracked_chassis_description` 按 R2 迁为 `agt_robot_description`，另新增 6 个 V4 包，净增 6；详见审计目录 `package_inventory_diff.txt`。旧包名的直接调用不再存在，不能把“80 个唯一包”写作旧包完全未丢；其兼容性应单独核对。先前“基线不可见”仅指隔离的 MCP 文件系统，现已由 SSH 核实，不是重造基线。
- Navigation 仓库分支 `refactor/navigation-runtime-v4`，基线 HEAD `e27abeb`；未运行 `git clean`、`reset` 或提交。`src/agt_robot_bringup`、`src/agt_mission` 尚未处于可恢复的 Git 根内；R3 的 `map_promotion.py` 原本就是未跟踪文件，本次编辑前备份至审计目录。受保护的 `agt_robot_hmi/install/{CMakeLists.txt,linux/bin/start.sh,windows/bin/start.bat}` 仍为 `D`，不恢复或清理。

**2026-09-24 仓库边界补记：**上述 `src/agt_robot_bringup`、`src/agt_mission` 是 2026-09-23 审计时的位置。现已分别成为公开的 `agt_robot_platform` 与 `agt_mission` Git 仓库；本机原本无 Git 根的 `bunker_base`、`bunker_msgs`、`ugv_sdk` 源码及 Apache 2.0 许可证归入 Robot Platform。Mission 启动编排也移到外部 `agt_mission_bringup` 包，当前包数为 **81 个唯一包**；Navigation 不存放这些实现。两个新仓库与 Robot Description V4 提交的精确 SHA 由 `dependencies/navigation_v4_external.repos` 锁定，已从远端干净导入核对。完整 V4 工作空间仍有其他仓库未提交、未做新机器恢复。详见 [跨电脑迁移清单](docs/v4_migration/portable_workspace.md)。
- 正式地图目录 `/home/yangxuan/ros2_ws/maps/` 未用于故障注入。测试前后 `registry.yaml` SHA-256 均为 `f8b8d4cddb1afd797320f6123276e8223dd95e908bb24d048665df743d0c37d2`，`active_map.yaml` 均为 `ac59d2d4ad6b2053afee4b9afc8bb9b5eb555600a96834a5c00f5fac0417e8d8`。registry 的 `active: ''` 与已有的历史 `active_map.yaml` 是两个不同事实，不可推断现场已按 V4 激活。

## R0–R9 状态

| 阶段 | 本轮结论 | 证据及边界 |
| --- | --- | --- |
| R0–R1 基线与安全清理 | `BASELINE_VERIFIED` | SSH 核对原始文件、SHA、74 包与受保护状态；当前 80 唯一包，R2 有 1 个描述包改名、7 个新增包（净增 6），未清理现场文件。 |
| R2 Robot Profile / 硬件 owner | `BUILD_PASS；现场 NOT_RUN` | 新 overlay 中 `agt_robot_description`、`agt_robot_bringup`、`agt_system_bringup` 构建通过；冷启动、驱动与 TF 唯一发布者未验证。 |
| R3 candidate / registry / resolver | `EXCEPTION_FAULT_TEST_PASS；CRASH_RECOVERY NOT_RUN` | `map_promotion.py` 的激活异常补偿已修复：包一经成功登记仍为 `VALIDATED`、candidate 为 `PROMOTED` 并记录错误；失败时保留上一 `active` 和旧指针。临时目录故障注入修复前 3 失败/2 通过，修复后新增边界用例共 7 个通过；`agt_map_manager` 修改后重构建通过。跨文件断电窗口与独立选图入口的一致性仍须复核。 |
| R4 Supervisor / Navigation Capability | `BUILD_PASS；ROS Action 集成 NOT_RUN` | 相关 Python 包构建通过，Health/策略回归属于下述 pytest；尚无本轮同一 ROS graph 的零转发、取消及故障 Action 证据。 |
| R5 Mission | `BUILD_PASS；V4 Route/TaskGroup 集成 NOT_RUN` | Mission 接口、BT、runtime 在无 Livox 依赖的 clean 子集中构建；本轮单元回归通过，不把旧巡检 Mission 的测试算作 V4 Action 验收。 |
| R6 RViz | `PARTIAL` | 画线仍走 V4 `FollowRoute`；旧 `rviz_patrol.py` 仍调用 `/agt/mission/execute`。修正了两条已过时的静态断言，使测试准确检查 `mission.launch.py` 的显式 `enable_legacy_inspection`，**未迁移旧巡检实现**。 |
| R7 HMI | `BUILD_PASS；界面交互 NOT_RUN` | 依赖原工作区已安装驱动的隔离 overlay 构建 HMI 通过；三项 Git 跟踪删除保持原状。真实界面与 ROS Action 状态/取消一致性未运行。 |
| R8 benchmark / pointcloud / map generation | `AUDITED；parity NOT_RUN` | 三份既有 R8 审计仍有效；此前日志记录 benchmark 7、工具 10、点云 core GTest 3 项通过。本轮未对同 bag/同 PCD 做数值 parity，不删除两套实现。 |
| R9 架构与验收 | `DOCUMENTED；RELEASE NOT_READY` | SDK 1.4.3 配对后 12 包核心 clean 闭包已通过；全工作区 clean build、仿真/回放与实机门尚未通过。 |

## 2026-09-23 宿主机无硬件验证

所有原始日志、退出码、JUnit XML、初始工作树状态与编辑前备份位于 `/home/yangxuan/ros2_ws_audit/navigation_v4/ssh_validation_20260923/`；构建输出放在该审计目录下，**未覆盖工作区原有 `build/`、`install/`、`log/`**。这些运行之间有覆盖，不应将测试数相加当成独立覆盖率。

| 验证 | 实际结果 | 可复核文件 / 说明 |
| --- | --- | --- |
| 包清单 | `80/80` 唯一，集合差异已记录 | `package_inventory_diff.txt`：基线原包 `tracked_chassis_description` 已迁名，新增 `agt_robot_description` 及 6 个 V4 包；不是 74 包的严格超集。 |
| clean 核心闭包，首次尝试 | `FAIL`，7 包完成 | `build_core.full.log`，退出 1。未给外部 Livox 驱动传其 `build.sh` 要求的 `-DROS_EDITION=ROS2 -DDISTRO_ROS=humble`；保留失败，不算源码缺陷。 |
| clean 核心闭包，补齐参数 | **整体 `FAIL`**，10 包完成 | `build_core_humble.full.log`，退出 2；Livox 驱动编译引用 `kLivoxLidarDoubleEchoData`、`LivoxLidarDoubleEchoRawPoint`，当前 `/usr/local/include/livox_lidar_def.h` 中不存在；Supervisor 未处理。没有修改驱动/SDK。 |
| SDK 1.4.3 成对绑定的驱动 | `PASS 1/1`（构建） | `build_sdk143_driver.full.log`、退出 0；CMake cache 明确指向 `.agt_native/include` 和 `.agt_native/lib`，不触及正式 `install/`。 |
| SDK 1.4.3 成对绑定的核心 clean 闭包 | `PASS 12/12`（构建） | `build_core_sdk143.full.log`、退出 0；包含 Livox 与 Supervisor，不等于全 80 包 clean build、ROS graph 或实机 PASS。 |
| SDK 1.4.3 普通安装与加载路径 | `PASS 1/1`（构建 + 静态 ELF 核对） | `build_sdk143_rpath.full.log`、退出 0；明确 `CMAKE_INSTALL_RPATH` 后普通安装的 `ldd` 默认解析 `.agt_native/lib`，只证明本机该环境的静态链接解析。 |
| 依赖已有驱动的 V4 overlay | `PASS 7/7` | `build_overlay.full.log`、退出 0：Robot Description/Bringup、System Bringup、Navigation Runtime/Supervisor、Mapping Bringup、HMI。**不是全量 clean build。** |
| R3 修复后单包重构建 | `PASS 1/1` | `build_mapmanager_after.full.log`、退出 0；使用原工作区 underlay，提示包覆盖警告，未声称独立 clean。 |
| `colcon test` 探测 | 命令退出 0，但 **0 tests** | `test_core.full.log`、`test_core.results.txt`：8 包的默认 unittest 发现机制未发现 pytest 文件；不可将命令退出 0 记成 A2 PASS。 |
| 显式 pytest，初次 | `PASS 47/47` | `pytest_core.log`、`pytest_core.xml`：Map、Mission BT、Navigation Capability、RViz 的原有用例。 |
| R3 故障注入，修复前/后 | 修复前 `3 FAILED, 2 PASSED`；修复后 `5 PASSED`，另补无 active/不激活两例 | `pytest_map_before.{log,xml}`、`pytest_map_after.{log,xml}`；均使用 pytest 临时目录，未触碰正式地图。 |
| 扩大范围回归 | 最终 `PASS 106/106` | 首次 `104 PASSED, 2 FAILED` 是旧断言仍指向已迁移的 `enable_inspection`；仅更新测试合同后 `pytest_regression_after.{log,xml}`、退出 0。包含 R3 新增 7 例及 Navigation Runtime 的不启动节点用例。 |

### Livox SDK 双版本根因与修复边界

- 宿主机并存两套 SDK：`$WS/.agt_native/include/livox_lidar_def.h` 是 **1.4.3**（与 `src/external/Livox-SDK2` 源头 SHA 一致），包含双回波枚举/结构体；`/usr/local/include/livox_lidar_def.h` 是 **1.3.1**，缺少这两项。完整头/库 SHA、三轮成功构建与 loader 路径见审计目录 `sdk_pairing_diagnosis.txt`。
- 旧驱动构建从额外 `CXX_FLAGS=-I$WS/.agt_native/include` 得到 1.4.3 头文件，但旧 `CMakeCache.txt` 的 SDK 库实际是 `/usr/local/lib/liblivox_lidar_sdk_shared.so`（1.3.1）；旧正式安装的 ELF RUNPATH 指向 `/usr/local/lib`，本轮普通环境 `ldd` 也解析到 1.3.1。旧构建能成功**不表示头文件与运行库配对正确**，更不表示点云行为已验收。
- 在**全新审计输出目录**显式绑定 1.4.3 头文件与 1.4.3 库后，Livox 单包及核心 12 包 clean 构建通过；普通安装试验同时设置 `CMAKE_INSTALL_RPATH` 后，`readelf` 和 `ldd` 指向 `.agt_native/lib` 的 1.4.3。`--symlink-install` 在开发目录保留的 RUNPATH 不能替代对最终普通安装产物的核验。
- 已修订 `scripts/field_build_smoke.sh`：同一前缀的双回波头与共享库预检、成对 CMake 绑定、安装 RPATH 及安装产物 `ldd` 防串版检查。该脚本 `bash -n` 和 `git diff --check` 已通过，但**未直接执行**：它仍会改动 Livox ROS 链接并使用原工作区 `build/install`/清理测试目录；上述构建 PASS 来自隔离目录内的等效 SDK 参数实验，不是该脚本端到端 PASS。
- 本次未修改第三方驱动/SDK 或生产启动环境，未把审计 install 覆盖进原工作区。绝对 RPATH 与本机目录绑定，且实际加载仍可能受启动进程的 `LD_LIBRARY_PATH` 影响；部署前需对**真实启动环境**重做 `ldd`/版本核对，并在受控现场验证 MID360 点云类型、时戳和频率。

此前直连 MCP 的 AST `13/13`、XML `13/13`、shell `2/2`、纯逻辑断言 `10` 和文档格式/链接检查是另一组**非 ROS**证据；不要与以上 pytest 数字混算。构建通过不等于已启动 ROS 图，更不等于底盘或传感器工作正常。

## 待收口项与发布门

1. **生产部署 SDK 版本/加载路径仍是阻断。** 本机配对 1.4.3 后，核心 12 包 clean 构建已通过，但完整工作区 clean build 未运行；旧正式驱动安装在默认环境下仍解析 `/usr/local/lib` 的 1.3.1。选择并固定 SDK 1.4.3 的头/库、普通安装的 RUNPATH 或受控节点环境，检查最终 `ldd` 与 ROS 启动环境，再做传感器现场验收；勿直接删改双回波逻辑或原地覆盖已发布程序。
2. **R3 异常补偿已测，跨文件断电一致性未验。** `registry.yaml` 与 `active_map.yaml` 无法作为单个文件原子提交；还需启动时的指针核对/恢复演练，并审视旧 `select_map_package.select()` 只写 active state、不更新 registry 的路径。任何回滚不完整会显式报“manual reconciliation required”，不能自动继续现场。
3. **测试发现与 V4 集成。** `colcon test` 本次发现 0 用例，应接入 pytest/CI 或固定显式 pytest 命令；Navigation Capability、Mission Route/TaskGroup 的真实 ROS Action fake-backend、Health fail-closed、暂停/取消及 UI 端到端尚需隔离环境验证。
4. **R6 与版本恢复。** 旧 RViz 巡检仍是 `/agt/mission/execute`；完成语义对照前保持显式兼容开关。R2 原描述包名已迁走，若仍有调用 `tracked_chassis_description` 的外部入口需另做兼容核对。Mission 与 Robot Platform 已转入独立远端，Navigation 仍有未跟踪的 R3 源码和新增测试，须纳入后续提交与完整工作空间锁文件。
5. **B/C 验收仍为 `NOT_RUN`。** 仿真/回放、同 bag/PCD parity、现场 Cold Start、传感器频率、TF 唯一发布者、自动重定位、`NAV_READY`、真实 HMI/RViz 任务及低速受控运动均未由本次 SSH 测试签发；须由具备急停和现场看护的人员按矩阵执行。

**当前结论：`NOT_READY_FOR_FIELD_ACCEPTANCE`。** 本轮通过的是明确列出的软件构建与临时目录测试，不更改现场 Profile 标定、正式地图或已发布运动参数。
