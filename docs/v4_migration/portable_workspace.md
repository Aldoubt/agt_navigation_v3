# V4 跨电脑迁移清单

## 仓库边界

| Git 仓库 | ROS 包与职责 |
| --- | --- |
| `agt_navigation_v3` | 定位、Nav2、Navigation Health/Capability、RViz 调试和现场导航编排 |
| [`agt_mission`](https://github.com/Aldoubt/agt_mission) | `agt_mission_interfaces`、`agt_mission_bt`、`agt_mission_runtime`、`agt_mission_bringup`；任务语义与执行 |
| [`agt_robot_platform`](https://github.com/Aldoubt/agt_robot_platform) | `agt_robot_bringup`、`bunker_base`、`bunker_msgs`、`ugv_sdk`；硬件入口与本机使用的底盘源码 |
| [`agt_robot_description`](https://github.com/Aldoubt/agt_chassis_description) | `bunker_v1` Profile 与标定；V4 提交已固定在外部清单中 |
| 其他独立仓库 | `agt_mapping_framework`、`agt_robot_hmi`、`shared/agt_pointcloud_pipeline` 等 |

Mission 与 Robot Platform 都是公开的独立 Git 仓库；导航代码通过 ROS 包名和 Action/消息接口依赖它们，没有 Git 子模块或源码绝对路径依赖。Robot Platform 保留 Bunker/UGV SDK 原 Apache 2.0 许可证。不要再导入旧的 `src/drivers/agt_bunker_base`、`src/drivers/bunker_ros2`、`src/drivers/ugv_sdk` 或手工复制一份 Mission 包，否则会产生重复包。

`dependencies/navigation_v4_external.repos` 固定两个新仓库和 Robot Description V4 版本的**精确提交 SHA**。`scripts/bootstrap_humble.sh` 在原第三方清单之外导入并检查该清单。旧 `dependencies/field_demo.repos` 使用迁名前的 description 路径与底盘仓库，不是 V4 完整依赖清单。

## 发布与恢复边界

两个新仓库和 Robot Description 的 V4 提交已推送；**完整 V4 工作空间仍未锁定**：Navigation、Mapping、HMI 等仓库尚有本地未提交改动。将它们分别提交推送后，用精确 SHA 制作完整工作空间 `.repos` 清单。仅指定 `main` 或 V4 分支名不能保证两台电脑构建相同代码。执行只读源码归属审计可发现仍未被 Git 跟踪的 ROS 包：

```bash
cd ~/ros2_ws
python3 src/agt_navigation_v3/scripts/check_v4_source_ownership.py --source-root src
```

地图、registry、bag、日志和 `.agt_native` 不属于源码提交。正式 Map Package 连同 `maps/registry.yaml` 单独传输到新工作空间的 `maps/`。`active_map.yaml` 含旧机器的绝对路径，不要原样用作新机器的激活指针。新机用 registry 精确版本或 `latest_validated` 解析，重新核验整包 SHA 与 Robot Profile。地图生产和 Map Manager 的部分 CLI/YAML 默认值仍含旧机路径；在新机生成或编辑地图前须显式设置目录并复核实验目录约束。

Livox SDK 1.4.3 头文件与运行库必须成对构建；新机检查最终驱动安装的 `ldd`/RUNPATH，不能混用旧 1.3.1 库。具体构建门见 [V4 验收矩阵](../acceptance/V4_ACCEPTANCE_MATRIX.md)。

## 新机器最小源码检查

在 Ubuntu 22.04 / ROS 2 Humble 的新工作空间中，先克隆最终发布的 Navigation 提交，再按其锁文件导入依赖。当前可单独核对两个新增仓库及 Robot Description：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
# 先克隆已发布的 agt_navigation_v3 V4 提交。
vcs import --skip-existing src \
  < src/agt_navigation_v3/dependencies/navigation_v4_external.repos
source /opt/ros/humble/setup.bash
colcon list --names-only --base-paths src | sort | uniq -d
```

最后一条命令应无输出。完成全工作空间干净构建并 `source install/setup.bash` 后，`ros2 pkg prefix agt_mission_bringup` 和 `ros2 pkg prefix agt_robot_bringup` 应指向新工作空间。`run_field_stack.sh` 默认读取 `<工作空间>/maps/registry.yaml`；若地图在其他目录，用 `--map-registry /绝对路径/registry.yaml`。直接运行 launch 时传 `map_registry:=/绝对路径/registry.yaml` 或设置 `AGT_MAP_REGISTRY`。

还需在新机执行无硬件 Action 集成、回放与现场门。只有**完整精确提交清单、正式地图数据、干净构建和新机器验收日志**齐备，才能宣称整体 V4 迁移可复现。
