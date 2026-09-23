# Navigation Runtime V4 验收矩阵（R9）

本矩阵是**验收计划**，不是一次执行结果。实际结论记录在仓库根目录 [V4_MIGRATION_REPORT.md](../../V4_MIGRATION_REPORT.md)。离线、仿真与现场证据必须分别记录；真实硬件测试只能由现场人员在急停、隔离区和明确授权下进行。

## 状态与证据规则

- `PASS`：有可定位的本次运行日志/报告、时间、代码版本、地图版本及明确的通过结果。
- `FAIL`：有复现步骤或静态代码证据表明合同不成立；不能以“不影响当前演示”代替修复。
- `OBSERVED`：源码或已有文件可见，但未验证运行结果；**不是 PASS**。
- `NOT_RUN`：本环境未执行或没有足够证据；尤其不可把无硬件的 mock 写作现场 PASS。
- `NEEDS_REVIEW`：设计/一致性风险待故障注入或人工决定。

每份验收证据应同时记录各 Git 仓库的分支、HEAD 和脏工作树摘要、ROS 发行版及 overlay、Robot Profile、registry/map 版本、运行命令、退出码、日志路径和执行人。原始 R0 基线若不在当前机器可见，先找回它，**不要用当前状态冒充改造前基线**。三个已被 Git 跟踪且处于删除状态的 `agt_robot_hmi/install` 文件不得按生成物清理。

## A. 不接硬件的门

| ID | 检查和判据 | 最低证据 |
| --- | --- | --- |
| A0 工作树 | 记录各仓库 `git status --short`、HEAD 与原始 R0 基线；不得重置现场更改；`git diff --check` 无新问题；在工作空间根运行 `python3 src/agt_navigation_v3/scripts/check_v4_source_ownership.py --source-root src`，发布时不得有未跟踪的 ROS 包源码。 | 基线文件、diff、仓库清单、源码归属输出；新包提交推送后再做干净克隆。 |
| A1 包与构建 | `colcon list --names-only` 无重复且无**未经说明的**丢包；按 R0 集合逐名记录 R2 描述包迁名及新增包；分包构建 Robot、Map、Navigation、Mission、RViz、HMI 和 R8 保留包，再做干净 overlay 构建。 | 基线包集合差异、旧入口兼容结果、colcon 日志与退出码；不能仅凭包总数判断 PASS。 |
| A2 单元与静态 | 相关 `colcon test` 与显式 Python pytest 无失败；有 pytest 文件的包若显示 `0 tests`，即使命令退出 0 也不能算 PASS；新 Python/manifest/shell 通过语法检查。 | JUnit/XML、命令输出、实际测试数；AST/纯逻辑检查只能证明其覆盖的部分。 |
| A3 地图生命周期 | `auto/latest/active/精确版本`、未知地图、错误 Profile、包篡改、拒绝覆盖、候选质量不合格都按预期处理。激活前/后失败、第二次 registry 写失败、无历史 active 时须保持包、registry、candidate 和指针可解释；另验跨文件断电/重启恢复。 | 临时目录的故障注入、前后 SHA 与重启核对；不可在正式 `maps/` 做破坏性注入。 |
| A4 启动和 TF 静态合同 | 一个 Robot Bringup 硬件 owner；定位独占 `map → odom`；底盘 `publish_odom_tf=false`；模拟和实机入口不重复启动驱动。 | launch ownership 测试、参数快照、静态 TF 合同核对。 |
| A5 Health/Navigation | Health 过期、定位丢失、地图不兼容、Nav2 拒绝/失败/取消时，`NavigateTo` 和 `FollowRoute` 不报告成功；目标不会在未就绪时转给 Nav2。 | mock Action/Health 集成日志、结果码、零转发计数。 |
| A6 Mission | P01 → Task A → P02 → Task B → P03；仅任务成功才推进；超时、重试上限、暂停/恢复、停止、取消与不支持 handler 均有确定结果。 | 可复现的 mock 场景、状态序列和 Action 结果。 |
| A7 UI | RViz 画线走 `FollowRoute`；巡检 RViz 与 HMI 应共用 V4 Mission contract。旧 `/agt/mission/execute` 入口保留期间，应明确标注和隔离，不能把旧链路测试算作新链路 PASS。 | 两个 UI 的请求/状态/取消对照，ROSBridge 拒绝导航控制的验证。 |
| A8 R8 职责 | Benchmark/点云包迁移前后可发现和构建；生产障碍点云只有既定预处理器；两套投影器和 benchmark 指标在做同输入 parity 前不删改。 | 依赖/引用扫描、包清单、测试及同 bag/同 PCD 对照记录。 |

直接 MCP 终端仍缺少 ROS；宿主机可经已校验的 SSH 使用 Humble/colcon。已有证据表明 SDK 1.4.3 的**头文件与库成对绑定**可使核心 12 包 clean 构建通过，但全工作区及生产运行时尚未验收。以下示例必须使用**新建的隔离输出目录**，不能覆盖正式 `build/install/log`；普通安装还要验证安装 RUNPATH 与最终 `ldd`：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
N="$PWD/.agt_native"
AUDIT="$(mktemp -d "$HOME/ros2_ws_audit/navigation_v4/clean_recheck_XXXXXX")"
colcon list --names-only
colcon --log-base "$AUDIT/log" build --build-base "$AUDIT/build" --install-base "$AUDIT/install" \
  --packages-up-to agt_system_bringup agt_mission_bringup agt_rviz_patrol agt_robot_hmi \
  --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  "-DLIVOX_LIDAR_SDK_INCLUDE_DIR:PATH=$N/include" \
  "-DLIVOX_LIDAR_SDK_LIBRARY:FILEPATH=$N/lib/liblivox_lidar_sdk_shared.so" \
  "-DCMAKE_INSTALL_RPATH:STRING=$N/lib"
colcon test --build-base "$AUDIT/build" --install-base "$AUDIT/install" \
  --packages-select agt_robot_description agt_system_bringup agt_mission_bringup agt_mapping_bringup agt_map_manager agt_navigation_capability agt_navigation_supervisor agt_mission_bt agt_mission_runtime agt_rviz_patrol
colcon test-result --verbose --test-result-base "$AUDIT/build"
# 同时运行显式 pytest；如果上一步显示 0 tests，不能记为 PASS。
```

上述**完整清单**仍待执行，不代表本轮已通过；本次 12 包 clean 子集、普通安装 RPATH 试验和 `106` 项显式 pytest 的原始证据见报告与审计目录。部署前对真正交付的 `install/livox_ros_driver2/lib/liblivox_ros_driver2.so` 执行 `readelf -d` 与 `ldd`，确认 SDK 路径是预期的 1.4.3，且启动环境没有覆盖；若任何阶段失败，不跳过到现场门。

## B. 仿真／回放门

| ID | 条件 | 通过标准 |
| --- | --- | --- |
| B1 受控启动 | 离线地图 + Robot Profile + `map:=auto`，以及 `active` 空/有效两种状态。 | 选图可解释；错图失败闭锁；Nav2/Health 状态与日志一致。 |
| B2 任务闭环 | 固定 Route/TaskGroup，与相机 mock 对接。 | “到点”与“拍照完成”分别计数；失败、超时、取消不会误发下一目标。 |
| B3 兼容与回滚 | 旧巡检入口保持原有行为，V4 新入口独立验证。 | 两套入口不重复派单；任何回退都有指明的版本和操作证据。 |
| B4 地图/感知 parity | 相同 bag/PCD、相同参数比较点云链、投影器和基准指标。 | 显式记录容差、帧/时间戳、差异；没有 parity 不做算法等价声明。 |

## C. 实机门（只能现场执行；默认 NOT_RUN）

1. 冷启动前记录急停、场地隔离、权限与独立看护；不通过任何自动化脚本直接发送运动目标。
2. 检查 MID360/IMU/底盘的频率、时戳、驱动实例数、静态/动态 TF 发布者；确认 `map → odom` 与 `odom → base_footprint` 各唯一。
3. 只在登记且适配 Profile 的不可变地图上验证自动重定位、`NAV_READY` 连续稳定、失定位降级及制动路径；记录实际地图版本。
4. 低速受控执行单航点、画线路径、任务拍照、暂停/取消/故障注入；核对 RViz 与 HMI 的同一后端状态。失败即停止，保留 ROS graph、bag 和日志。
5. 单独签发现场结果；软件构建、mock、历史回放和地图 registry 的 `VALIDATED` 标记都**不等于**现场通过。
