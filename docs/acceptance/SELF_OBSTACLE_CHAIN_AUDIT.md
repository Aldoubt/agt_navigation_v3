# Navigation Self-Obstacle Chain Audit

范围：当前 field launch 的 navigation obstacle 支路；只读 YAML、launch 与 C++ 源码，并关联 `NAV_TEST_002` rosbag evidence。未改变任何过滤或 Nav2 参数。

## 实际数据链

```text
/livox/lidar                         livox_ros_driver2/CustomMsg, frame=livox_frame
  -> agt_livox_tools custom_to_pointcloud2 bridge
/agt/livox/points                    PointCloud2
  -> agt_obstacle_cloud_preprocessor
/agt/navigation/points_obstacles     PointCloud2, filtered after base_link-frame classification
  -> Nav2 local_costmap VoxelLayer lidar3d observation source
/local_costmap/costmap
  -> InflationLayer
  -> controller collision checking
```

`NAV_TEST_002` 仅录制了最上游 `/livox/lidar`，没有录制 bridge 输出或 `/agt/navigation/points_obstacles`；故可证明输入可用，不可把 local costmap 内的高代价 cell 归因到某个具体 LiDAR 点。

## Nav2 costmap 输入合同

`nav2_params.yaml` 与 `nav2_acceptance_params.yaml` 均配置：

| 项目 | 值 |
|---|---|
| local global frame / robot frame | `odom` / `base_link` |
| local layers | `VoxelLayer`, `InflationLayer` |
| observation source | `lidar3d` |
| topic | `/agt/navigation/points_obstacles` |
| marking / clearing | true / true |
| obstacle range | 0.30–12.0 m |
| raytrace range | 0.30–15.0 m |
| obstacle height | 0.10–2.0 m |
| inflation radius | 0.55 m |

global costmap 不接收 LiDAR obstacle source；它是 `StaticLayer + InflationLayer`，因此本轮自障碍问题的首要对象是 **local costmap**。

## Self mask 与 rear filter 合同

Obstacle preprocessor 会用 `base_link <- cloud_frame` TF 将每个点转换到 `base_link` **用于过滤判定**，然后保留未过滤点的原始 cloud frame 坐标和 header 发布：

1. range filter：0.5–120.0 m；
2. self box（enabled）：center `(0,0,0.20)`，size `(1.023,0.778,0.400)` m，padding `0.05 m`；
3. rear sector（仅在 `rear_sector.enabled=true` 时）；
4. 0.20 m voxel downsample。

等效 self box 边界为 x `[-0.5615, +0.5615]`，y `[-0.439, +0.439]`，z `[-0.05, +0.45]` m。它不会自动覆盖更高、更远后方或不在该盒内的安装件反射。

## rear_filter / rear_sector 不一致

配置中声明了：

```yaml
rear_filter:
  enabled: false
  type: box
  box: {x_min: -2.0, x_max: 0.0, y_min: -0.8, y_max: 0.8, z_min: -0.5, z_max: 2.0}
```

但当前 `obstacle_cloud_node.cpp` 从未声明或读取 `rear_filter.*`。它只读取并执行：

```text
rear_sector.enabled
rear_sector.center_deg
rear_sector.width_deg
rear_sector.min_range_m
rear_sector.max_range_m
```

当前 YAML 中 `rear_sector.enabled=false`，所以运行时没有 rear-sector 排除，YAML 的 `rear_filter` box 也不会生效。这是**配置—实现 contract defect**；本文件只记录事实，不建议在未采集输出点云前直接调整过滤区域。

## 与 nav_test_002 的关系

在 694 个 local costmap 样本中，15 个样本的 padded footprint 内出现 `OccupancyGrid=100`，集中于停止阶段。该结果与 self-obstacle 假设一致，但亦可能来自真实近场障碍、机器人贴近静态地图结构或 footprint/inflation 几何；当前 bag 缺少下游点云，不能在三个假设间定责。

## 现场验证（不改参数）

在安全静止状态下，以 `base_link` 为固定 frame 在 RViz 同时显示：

- `/agt/livox/points`；
- `/agt/navigation/points_obstacles`；
- `/local_costmap/costmap`；
- `/local_costmap/published_footprint`。

录制同四项加 `/tf`、`/agt/odometry/local`。若后方杆的点仍出现在 obstacle output 并落入 footprint/inflation 范围，才足以把该硬件反射确定为根因。验证时还应读取实际参数 `rear_sector.enabled` 而不是只查看 YAML。
