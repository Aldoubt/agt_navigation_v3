# 中文 Qt 面板验证

范围：AGT YHS 控制面板与现场航点表。独立 Qt fork 的 feature/yhs-field-appliance-v1 分支；没有复制导航/建图核心，没有修改协议命令、地图身份或 Motion Guard 门禁。

- 原有主窗体浅色/蓝色 #1976d2/圆角按钮风格应用到控制面板；标签页、输入框和状态卡片统一。
- 中文标题、系统/建图/导航/录制/诊断、动作、设备/任务/定位/建图状态；航点朝向/停留/动作表头。
- 航点动作显示“等待”，JSON 仍为 action: wait。未知状态保留原文，防止诊断被隐藏。
- 长页面使用 QScrollArea。修复旧 dock perspective 导致控制面板被挤到 100px 的布局；用户仍可调整停靠位置与大小。
- 状态区悬停查看原始最近时间戳。接口、地图/路线标识和诊断 JSON 保留真实值。
- Dockerfile.base 加入 fonts-noto-cjk/fontconfig；主窗体已有字体选择中加入 Noto Sans CJK 回退。

验证环境：实际运行的 agt-yhs-runtime-1 Humble 容器，Qt 5.15.3；使用已有 native 编译依赖进行 cmake --build /opt/hmi_build -j2，主程序链接成功。不是新的完整 Docker image build。

Qt CTest /opt/hmi_field_tests：1/1 PASS，包含 add/delete/reorder/yaw/dwell/save/load/map binding，并新增中文显示、动作 wire value 不变、所有实际建图阶段中文映射与未知状态回退检查。

UID 1000:1000 运行实际 Qt 主程序，QT_QPA_PLATFORM=offscreen，以 QWidget::grab 保存系统/建图/导航面板及完整主窗体。截图由真实程序生成；没有人工绘图或图像修改。检查中文字体正常、按钮可见、设备信息不截断、停靠面板宽度恢复。截图中的设备状态为 Mock；场景使用现有微型测试地图，没有真实环境地图。原始截图记录在本次执行 workspace/artifacts/yhs-hmi；Qt 中文使用说明中包含系统面板截图。

45 appliance tests PASS。真实 X11 显示与鼠标操作、YHS/CAN/MID360、TF/外参、定位导航与安全均继续 PENDING。

旧本地镜像 agt-yhs-field:v1 保持前一交付镜像；当前容器已增量编译新 GUI，并安装与 fonts-noto-cjk 一致的 NotoSansCJK-Regular.ttc 验证。部署时 git pull 后重新 ./install.sh，将由正式 Dockerfile 取得锁定 fork、编译 GUI 并安装字体。不能用旧镜像的 --skip-build 来获取本次界面更新。
