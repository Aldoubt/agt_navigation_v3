# Qt 地图编辑、保存与导航校验审计

范围：feature/yhs-field-appliance-v1；独立 Qt fork Aldoubt/agt_robot_hmi。

## 审计结论

原 Qt 橡皮擦、障碍物画笔、线段会修改显示地图的 QImage，GetOccupancyMap 再转回栅格。导航可以使用这些编辑后的 PGM。原保存流程存在兼容问题，不应通过关闭整个地图校验掩盖：

1. 栅格值为 -1/0–100，原保存却直接与 0–1 free/occupied threshold 比较。
2. MapConfig 默认 free/occupied threshold 反置，mode 没有初始化；保存没有输出 mode。
3. 输出浮点精度不足可能改变 resolution/origin。PGM 205 在 free_thresh > 50/255 时会被解释为空闲。
4. Save 不返回失败；主窗体即使 fopen 失败也弹“保存成功”。
5. 保存直接覆盖当前 map.pgm/yaml，另外写入 topology，却没有更新已封存 bundle 的哈希和版本；封存只读文件也不能直接覆写。
6. 半透明黑色与 Qt::black 的完整 RGBA 相等判断会丢失中间占用概率。

## 本分支修复

- OccupancyMap::Save 返回 bool；概率按 100 缩放，明确导出 Nav2 trinary P5，negate=0、free_thresh=0.196、occupied_thresh=0.65，未知仍为 205。写入错误不会报告成功。
- 保存 origin/resolution 使用 17 位浮点精度，输出 mode；统一处理 .yaml/.pgm 后缀。
- Qt 现场模式“保存地图”和“另存为”均先导出到 run/map_edits 的唯一草稿目录，确认后调用 SAVE_NAVIGATION_EDIT。
- 当前显示地图必须来自已激活 bundle，且源 binding 一致。独立打开的其他 PGM/YAML 不允许替换当前定位地图。
- backend 拒绝活动任务、未完成建图、未确认、错误源 binding、尺寸/原点/分辨率变化、损坏 PGM、路径越界与 unknown->free 配置。
- 新版本 `<旧版本>.qt-<唯一标识>` 复用冻结 mapping 与 localization 数据文件；更新 localization 身份元数据、Qt 编辑 provenance、review_status、V3 metadata 和 bundle manifest/hash。
- 原版本保持不变。保存后停止定位/Nav2，激活新版本并回到 IDLE；操作员重新启动定位导航，为新版本重新创建/保存路线。旧路线 hash/version 绑定继续拒绝运行。
- 没有取消 Map Bundle、三维资产、校准、Motion Guard 或 map->odom 所有权检查。原 main/V3 核心没有修改。

Qt 的拓扑点、连线与区域保存在 map.topology，属于 HMI/拓扑数据；不能仅凭在 Qt 画区域就声称 Nav2 keepout 已生效。需要禁止通行时，可用地图画笔把对应二维栅格画成占用，或使用现有 MapStudio/实际已配置的 keepout 流程。本次不新增高级区域到 Nav2 mask 转换器。Qt 场景坐标工具主要按轴对齐地图使用；没有新增旋转地图编辑支持。

## 操作

1. 激活要编辑的 Map Bundle；有任务时先取消并等待结束。
2. 在 Qt 主地图点击“编辑地图”，使用橡皮擦、画笔、线段。
3. 点击“保存地图”，阅读“新版本/停止导航/旧路线”提示，确认编辑结果。
4. 只有 backend 校验与激活成功，Qt 才显示保存成功。失败草稿保留用于诊断。
5. 重新启动定位 / 导航，确认 READY，保存绑定新版本的路线，再开始任务。

建图初次人工审核仍可使用“审核地图（MapStudio）→确认并封存→激活”。Qt 编辑已激活地图不需要先解除只读权限或手工改 manifest。

## 验证

- 56 appliance tests PASS：原 45 项与新增 11 项，含修改后的新 bundle 通过真实 V3 validate_package、原始文件不变、旧路线拒绝、几何变化/截断/错误 hash/路径/symlink/未知格配置拒绝，以及任务期间保存禁止。
- Qt CTest 2/2 PASS：航点逻辑、PGM/YAML 导出与重新加载，包含 0/100/-1/中间概率、行方向、尺寸、浮点精度、缺目录及无效栅格失败。
- Humble 容器内 Qt ROS2 主程序增量构建、Python wheel 构建。
- 真实 Qt 工具栏保存的 Mock RPC 集成验证通过：确认对话框 → 新地图版本 → 完整 V3 校验 → 激活，未知格保持 205，拓扑文件落盘，导航回到 IDLE/STOPPED；详见 appliance/evidence/qt_map_save_acceptance.txt；没有将 Mock 地图声明为真实 BBS/GICP 或实车 PASS。

当前 Mock 运行容器已增量更新 Qt 和经过验证的 Python wheel，重启后 healthy；本次不重建巨大完整 Docker image。现场更新集成 feature 分支并运行 ./install.sh，将自动取得固定 Qt fork。Qt 版本由 repos.lock.yaml 固定。实车与 X11 桌面交互仍待现场验收。
