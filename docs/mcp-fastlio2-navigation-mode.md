# FAST-LIO2 第二导航里程计模式与后方扇区检查

日期：2026-09-22。

> **后续试验更新（同日）：**按用户要求，当前 `config/perception.yaml` 已将后方扇区过滤开启，
> 仍为后方左右各 5°、0.5–4 m，仅影响障碍标记支路。下文是此前 FAST 接入阶段的检查记录，
> 其中“未开启”描述的是当时配置和原实车运行，不代表当前试验配置。
> 新试验步骤见 [后方窄扇区试验说明](mcp-rear-mask-field-trial.md)。

## 范围

新增 `--lio-backend batch_lio|fastlio2`，默认仍是 Batch-LIO；与既有
`--mode navigation|inspection` 独立。保留已有巡检、相机、地图、控制和感知参数改动，
没有 reset/checkout 覆盖现有工作区，也没有开启后方屏蔽。

本次只做代码接入、构建和离线/隔离消息验证。未运行实车导航，未给底盘发送命令，
未回放实车录包，也未修改 FAST-LIO2 原生算法、Batch-LIO 原生算法、车体外参或速度限制。

## 使用

```bash
# 原模式
/home/yangxuan/ros2_ws/src/agt_navigation_v3/scripts/run_field_stack.sh \
  --mode navigation --lio-backend batch_lio --rviz

# 第二导航里程计模式
/home/yangxuan/ros2_ws/src/agt_navigation_v3/scripts/run_field_stack.sh \
  --mode navigation --lio-backend fastlio2 --rviz

# 不启动任何 ROS 节点的入口检查
/home/yangxuan/ros2_ws/src/agt_navigation_v3/scripts/run_field_stack.sh \
  --mode navigation --lio-backend fastlio2 --dry-run
```

切换前应先正常退出旧栈，重新启动后保持静止并重新全局定位；不能热替换 odom 原点。
完整现场步骤与拍照模式说明见 [导航启动文档](../导航启动文档.md)。

## 接入合同

- `localization.launch.py` 只构造选中的一个前端分支。
- FAST 原始 `/fastlio2/lio_odom` 经适配后输出 `/agt/odometry/local`，消息为
  `odom→base_link`；适配器仅发布 `odom→base_footprint`，不发布 map→odom。
- Localization Manager 仍是 map→odom 唯一所有者，RSP 仍负责静态车体 TF。
- FAST 使用自己的冻结 `r_il/t_il`，与 URDF 组合得到 body→base；全局重定位共享
  同一标定源。保留旧 `batch_lio_config_file` 参数作为兼容回退。
- 拒绝 FAST world/body frame 合同不匹配、启用在线外参估计、标定覆盖值与前端不一致等情况。
- 上游未填写角速度时，通过连续、已经转换的 base 位姿生成一致的线/角速度。
  不跨长间断或时间回退差分；备用上游 twist 转换包含 `omega × r` 参考点项。
- 原始 header 时间保留；200 ms 过期门限保留，静态 TF 不全时不发布半套 odom/TF。
- FAST 发布与 Batch 相同的 AdapterStatus，原脚本的三次 FRESH 就绪检查可直接复用。
- `use_sim_time` 贯通 FAST 前端和适配器；原始 Livox CustomMsg/IMU 不经过障碍过滤。
- 每轮保存 `launch_selection.txt`、`lio_config_input.yaml`；FAST 另外使用临时运行快照。

## 已执行验证

### 修改前基线

35 项原有相关 pytest 测试通过。

### 修改后

87 项相关 pytest 测试通过，包括：

- 原模式/巡检模式合同与原有标定/查询帧测试；
- 后端选择互斥和非法值拒绝；四种 task/backend 组合的无节点 dry-run；
- 先拒绝已有运行栈，再保存新的 LIO 快照，避免重复启动覆盖原栈的标定输入；
- FAST 配置快照、独立标定、错误标定/坐标系拒绝、时钟参数传递；
- pose 差分角速度、参考点偏移、无效值/时间戳/陈旧数据拒绝；
- TF 不可用时不得部分发布；就绪/陈旧诊断；
- 后方扇区保持 opt-in，障碍标记/原始清除支路不混用。

宿主机以下 6 个包构建成功：

```text
fastlio2
agt_batch_lio_adapter
agt_fastlio_adapter
agt_global_relocalization
agt_navigation_runtime
agt_system_bringup
```

`fastlio2` 构建有非致命 underlay 覆盖提示及 CMake CMP0074/PCL_ROOT 提示；构建退出码为 0。

额外显式运行隔离合成消息检查：

```bash
AGT_RUN_ISOLATED_FASTLIO_TEST=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=217 \
python3 navigation/state_estimation/agt_fastlio_adapter/test/isolated_adapter_smoke.py
```

实际结果：6 条有效里程计与 6 条导航 TF 成对发布；角速度非零，原始时间戳保留；
12 条陈旧输入被拒绝，最后状态为 STALE、输出率为零。数据与 TF 话题额外重映射到
`/test/fastlio_adapter/*`。该检查只启动测试夹具与适配器，**没有启动 FAST 前端本体、
硬件、Nav2 或速度命令发布者**。

另通过：两个 shell 脚本 `bash -n`、实际 FAST dry-run、非法 `both` 后端退出码 2、
`git diff --check`。安装目录的 Batch launch 已核对实际节点名为 `/laserMapping`。

## 后方扇区检查

同时核对了源码 `config/perception.yaml` 和宿主机本轮实际
`/tmp/agt_navigation_params_dsg5zp1f/runtime.yaml`：

```yaml
rear_filter:
  enabled: false
  center_deg: 180.0
  width_deg: 10.0
  min_range_m: 0.5
  max_range_m: 4.0
```

能力确实存在：`obstacle_cloud_node.cpp::inside_rear_sector` 在 base 坐标系中按
XY 平面距离及后方夹角删除点；总宽 10°，即后方左右各 5°。该条件本身不限制 Z。
只有 `rear_enabled_` 为真时才生效，并计入 `rear_removed_points`。

它只影响 `/agt/navigation/points_obstacles` 标记支路；clearing 仍订阅原始
`/agt/livox/points`，LIO 输入保持原始 CustomMsg。当前关闭，不能将本轮问题直接
归因为“已开启后方扇区导致盲区”。反之，关闭也不保证所有车体回波都已被其他过滤覆盖。

当前自车 box 含 padding 的 base 范围：X ±0.5615 m、Y ±0.439 m、Z −0.05～0.45 m。
底盘低处、较高支架和 TF 偏差导致的回波可能在 box 外，需点云/实物核对；不是已经
证明这些区域产生了入侵。先区分膨胀软代价与致命占用，并同时检查已知位姿跳变、时间延迟
及 voxel Z 越界。没有通过开启整片后方屏蔽来掩盖这些问题。

## 未验证与未包含

- 没有新的原始雷达+IMU 录包，因此没有完成两个前端的同数据 A/B，也没有实车定位精度、
  延迟或导航成功率验收。现有两段探针包没有原始雷达点云，不能重算前端。
- FRESH 不是位姿质量证明；现有 LOST 受控恢复缺口和独立 pose 质量检测未在本次修改。
- 沿用现有 FAST 原生 IMU 处理：回调代码中有加速度乘 10.0，内部重力为 9.81。
  本次没有未经专门校准就修改共享建图前端；实车验收前应检查 MID360 单位、静止偏置与
  当前前端的加速度倍率，并保留原始传感器数据。
- 没有改动 self/rear/ground/voxel 感知配置，也没有放宽陈旧数据、碰撞或速度门控。
