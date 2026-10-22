"""Qt5 single-window operator entry point for Bunker and FW-mini.

The window controls only existing high-level ROS services. It never
publishes cmd_vel or changes motor enable. ROS failures remain visible.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

from PyQt5.QtCore import Qt, QProcess, QTimer
from PyQt5.QtGui import QColor, QImage, QPainterPath, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout,
    QFrame, QGraphicsScene, QGraphicsView, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton, QScrollArea,
    QSplitter, QTextEdit, QVBoxLayout, QWidget,
)

from .field_console_core import (
    DISTANCES_M, PROFILES, discover_map_packages, health_gate,
    validate_distance,
)


class FieldConsoleWindow(QMainWindow):
    def __init__(self, *, map_root, offline_demo=False, default_robot="fwmini"):
        super().__init__()
        self.offline_demo = bool(offline_demo)
        self.map_root = Path(map_root).expanduser()
        self.health = self.editor = self.guard = None
        self.last_reason = None
        self.bridge = None
        self.current_map = None
        self.route_points = []
        self.map_image = None
        self.children_processes = []
        self.setWindowTitle("AGT Field Console · 农业机器人操作台")
        self.resize(1460, 860)
        self.setMinimumSize(960, 630)
        self._build()
        if default_robot not in PROFILES:
            raise ValueError(f"unknown robot: {default_robot}")
        self.robot.setCurrentIndex(list(PROFILES).index(default_robot))
        self.refresh_maps()
        if not self.offline_demo:
            try:
                from .field_console_bridge import FieldROSBridge
                self.bridge = FieldROSBridge(self._on_ros_event)
                self._log("ROS 已连接：等待定位、路线后端和底盘状态")
            except Exception as exc:
                self._log(f"无法连接 ROS：{exc}。所有运动入口保持禁用。")
        else:
            self._log("离线演示模式：只有路线示意，绝不发送车辆命令。")
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(100)
        self._tick()

    def _build(self):
        self.setStyleSheet("""
          QMainWindow,QWidget {background:#101820;color:#eaf2f5;font:13px 'Noto Sans CJK SC';}
          QGroupBox {border:1px solid #344451;border-radius:10px;margin-top:14px;
                     padding:15px 10px 10px;font-weight:600;}
          QGroupBox::title{subcontrol-origin:margin;left:13px;color:#c7dbe4;}
          QPushButton {background:#23323e;border:1px solid #476071;border-radius:7px;
                       min-height:33px;padding:4px 10px;font-weight:600;}
          QPushButton:hover{background:#304a59;}
          QPushButton:disabled{background:#1c2831;color:#758996;border-color:#263642;}
          QPushButton#primary{background:#007f84;border-color:#0eb3ac;color:#fff;}
          QPushButton#primary:hover{background:#09979a;}
          QPushButton#danger{background:#663036;border-color:#a94d56;color:#fff;}
          QComboBox,QDoubleSpinBox {background:#1d2933;border:1px solid #476071;
             border-radius:6px;padding:6px;min-height:28px;}
          QComboBox QAbstractItemView{background:#1b2831;color:white;}
          QTextEdit {background:#121f28;border:1px solid #314654;border-radius:7px;}
          QGraphicsView {background:#172530;border:1px solid #40515b;border-radius:9px;}
          QLabel#banner{font-size:17px;font-weight:700;}
          QLabel#muted{color:#9db3bf;}
          QLabel#notice{color:#ffcb84;}
        """)
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(16, 12, 16, 12)
        head = QHBoxLayout()
        title = QLabel("AGT  /  农业机器人导航操作台")
        title.setObjectName("banner")
        head.addWidget(title)
        head.addStretch()
        self.mode_label = QLabel("离线演示｜禁止运动" if self.offline_demo else
                                 "连接中｜禁止运动")
        self.mode_label.setObjectName("notice")
        head.addWidget(self.mode_label)
        layout.addLayout(head)

        split = QSplitter(Qt.Horizontal)
        left = QWidget()
        left.setMinimumWidth(270)
        left.setMaximumWidth(390)
        left_l = QVBoxLayout(left)
        left_l.setContentsMargins(1, 0, 8, 0)
        select = QGroupBox("01  机器人与地图")
        sel_l = QFormLayout(select)
        self.robot = QComboBox()
        self.robot.setObjectName("robot_select")
        for key, (label, _) in PROFILES.items():
            self.robot.addItem(label, key)
        self.robot.currentIndexChanged.connect(self._changed_context)
        sel_l.addRow("底盘", self.robot)
        self.map_select = QComboBox()
        self.map_select.setObjectName("map_select")
        self.map_select.currentIndexChanged.connect(self._changed_map)
        sel_l.addRow("地图", self.map_select)
        buttons = QHBoxLayout()
        refresh = QPushButton("刷新地图")
        refresh.clicked.connect(self.refresh_maps)
        choose = QPushButton("选择目录")
        choose.clicked.connect(self.choose_map_root)
        buttons.addWidget(refresh)
        buttons.addWidget(choose)
        sel_l.addRow(buttons)
        self.activate = QPushButton("校验并激活所选地图")
        self.activate.setObjectName("activate_map")
        self.activate.clicked.connect(self.activate_map)
        self.activate.setToolTip("仅导航停止时激活地图包；校验资产哈希。禁止运行中热切图。")
        sel_l.addRow(self.activate)
        self.map_note = QLabel("地图尚未选择")
        self.map_note.setWordWrap(True)
        self.map_note.setObjectName("muted")
        sel_l.addRow(self.map_note)
        left_l.addWidget(select)

        mission = QGroupBox("02  农科院定距测试")
        ml = QVBoxLayout(mission)
        choices = QHBoxLayout()
        for d in DISTANCES_M:
            b = QPushButton(f"{d:g}m")
            b.setObjectName(f"distance_{int(d)}")
            b.clicked.connect(lambda _, value=d: self.distance.setValue(value))
            choices.addWidget(b)
        ml.addLayout(choices)
        self.distance = QDoubleSpinBox()
        self.distance.setObjectName("distance_custom")
        self.distance.setRange(0.1, 30.0)
        self.distance.setDecimals(2)
        self.distance.setSuffix(" m")
        self.distance.setValue(5.0)
        ml.addWidget(self.distance)
        self.prepare = QPushButton("生成并校验路线")
        self.prepare.setObjectName("prepare_distance")
        self.prepare.setProperty("kind", "prepare")
        self.prepare.clicked.connect(self.prepare_route)
        ml.addWidget(self.prepare)
        self.execute = QPushButton("确认执行路线")
        self.execute.setObjectName("primary")
        self.execute.clicked.connect(self.execute_route)
        ml.addWidget(self.execute)
        pair = QHBoxLayout()
        self.pause = QPushButton("暂停")
        self.pause.clicked.connect(lambda: self._request("pause"))
        self.cancel = QPushButton("取消任务")
        self.cancel.setObjectName("danger")
        self.cancel.clicked.connect(lambda: self._request("cancel"))
        pair.addWidget(self.pause)
        pair.addWidget(self.cancel)
        ml.addLayout(pair)
        left_l.addWidget(mission)

        mapping = QGroupBox("03  建图与地图编辑")
        mp = QVBoxLayout(mapping)
        self.map_studio = QPushButton("打开 Map Studio 地图编辑器")
        self.map_studio.clicked.connect(self.open_map_studio)
        mp.addWidget(self.map_studio)
        self.start_mapping = QPushButton("开始建图（暂不可用）")
        self.start_mapping.setEnabled(False)
        self.start_mapping.setToolTip(
            "现有建图流程含 PGO；无回环建图和资产发布尚未完成，不允许伪装成可用。")
        mp.addWidget(self.start_mapping)
        self.acceptance = QPushButton("查看验收报告")
        self.acceptance.clicked.connect(self.open_report)
        mp.addWidget(self.acceptance)
        note = QLabel("大棚自动覆盖和返回起点仍在验收中，不会开放未经验证的运动功能。")
        note.setWordWrap(True)
        note.setObjectName("muted")
        mp.addWidget(note)
        left_l.addWidget(mapping)
        left_l.addStretch()
        split.addWidget(left)

        center = QWidget()
        cl = QVBoxLayout(center)
        cl.setContentsMargins(4, 0, 4, 0)
        self.map_title = QLabel("二维栅格地图 / 路线预览")
        self.map_title.setObjectName("banner")
        cl.addWidget(self.map_title)
        self.scene = QGraphicsScene(self)
        self.view = QGraphicsView(self.scene)
        self.view.setRenderHints(self.view.renderHints())
        self.view.setDragMode(QGraphicsView.ScrollHandDrag)
        cl.addWidget(self.view, 1)
        self.route_label = QLabel("暂无路线")
        self.route_label.setObjectName("muted")
        cl.addWidget(self.route_label)
        split.addWidget(center)

        right = QWidget()
        right.setMinimumWidth(270)
        right.setMaximumWidth(380)
        rl = QVBoxLayout(right)
        rl.setContentsMargins(8, 0, 1, 0)
        status = QGroupBox("运行状态")
        sl = QVBoxLayout(status)
        self.summary = QLabel("等待连接")
        self.summary.setWordWrap(True)
        self.summary.setObjectName("notice")
        sl.addWidget(self.summary)
        self.status_values = {}
        for label, key in (
                ("机器人定位", "loc"), ("Nav2", "nav"),
                ("车体连接", "base"), ("安全状态", "safety"),
                ("路线校验", "route")):
            row = QHBoxLayout()
            row.addWidget(QLabel(label))
            v = QLabel("未连接")
            v.setAlignment(Qt.AlignRight)
            self.status_values[key] = v
            row.addWidget(v)
            sl.addLayout(row)
        rl.addWidget(status)
        rl.addWidget(QLabel("事件日志"))
        self.logs = QTextEdit()
        self.logs.setReadOnly(True)
        rl.addWidget(self.logs, 1)
        tip = QLabel("软件取消任务不等于物理急停。\n实车应保证遥控器和硬件急停可用。")
        tip.setObjectName("notice")
        tip.setWordWrap(True)
        rl.addWidget(tip)
        split.addWidget(right)
        split.setStretchFactor(1, 1)
        layout.addWidget(split, 1)
        self.setCentralWidget(root)

    def _log(self, message):
        self.logs.append(time.strftime("%H:%M:%S  ") + str(message))

    def selected_profile(self):
        return self.robot.currentData()

    def selected_map(self):
        return self.current_map

    def choose_map_root(self):
        choice = QFileDialog.getExistingDirectory(self, "选择地图根目录",
                                                   str(self.map_root))
        if choice:
            self.map_root = Path(choice)
            self.refresh_maps()

    def refresh_maps(self):
        previous = self.current_map.label if self.current_map else ""
        packages, warnings = discover_map_packages(self.map_root)
        self.map_select.blockSignals(True)
        self.map_select.clear()
        self.map_select.addItem("请选择已发布地图", None)
        chosen = 0
        for index, pack in enumerate(packages, 1):
            self.map_select.addItem(pack.label, pack)
            if pack.label == previous:
                chosen = index
        self.map_select.setCurrentIndex(chosen)
        self.map_select.blockSignals(False)
        self._changed_map()
        self._log(f"地图目录 {self.map_root}：找到 {len(packages)} 个地图包")
        for warning in warnings[:3]:
            self._log("地图跳过：" + warning)

    def _changed_context(self, *_):
        self.route_points = []
        self.health = self.editor = self.guard = None
        self._draw_route()
        if hasattr(self, "summary"):
            self._log("已切换车型；重新核对后端身份和地图")
        self._tick()

    def _changed_map(self, *_):
        self.current_map = self.map_select.currentData()
        self.route_points = []
        self._display_map()
        self._tick()

    def _display_map(self):
        self.route_items = []
        self.scene.clear()
        self.map_image = None
        p = self.current_map
        if p is None:
            self.map_note.setText("未选择地图。可以选择现有地图包的根目录。")
            self.map_title.setText("二维栅格地图 / 路线预览")
            self.scene.addText("请选择二维地图。").setDefaultTextColor(QColor("#dbe7ed"))
            return
        self.map_note.setText(f"地图：{p.map_id} · 版本：{p.version}\n仅选择用于预览，导航后端须使用同一版本。")
        self.map_title.setText(f"PGM 底图 · {p.map_id} / {p.version}")
        image = QImage(str(p.image))
        if image.isNull():
            self._log("地图图像加载失败：" + str(p.image))
            return
        self.map_image = image
        self.scene.addPixmap(QPixmap.fromImage(image))
        self.scene.setSceneRect(0, 0, image.width(), image.height())
        self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
        self._draw_route()

    def _pixel(self, x, y):
        p = self.current_map
        if p is None or self.map_image is None:
            return (x, y)
        dx, dy = x - p.origin_x, y - p.origin_y
        c, s = math.cos(p.origin_yaw), math.sin(p.origin_yaw)
        mx, my = c * dx + s * dy, -s * dx + c * dy
        return mx / p.resolution, self.map_image.height() - 1 - my / p.resolution

    def _draw_route(self):
        if not hasattr(self, "scene"):
            return
        for item in getattr(self, "route_items", []):
            if item.scene() is self.scene:
                self.scene.removeItem(item)
        self.route_items = []
        if not self.route_points:
            self.route_label.setText("暂无路线 · 请选择距离生成预览")
            return
        pen = QPen(QColor("#19c6b4"), 3)
        path = QPainterPath()
        a = self._pixel(*self.route_points[0])
        path.moveTo(*a)
        length = 0.0
        for previous, point in zip(self.route_points, self.route_points[1:]):
            path.lineTo(*self._pixel(*point))
            length += math.dist(previous, point)
        item = self.scene.addPath(path, pen)
        self.route_items.append(item)
        self.route_label.setText(
            f"预览路径 · 长度 {length:.2f} m · {len(self.route_points)} 个轨迹点")

    def _on_ros_event(self, event, payload):
        if event in ("health", "editor", "guard"):
            setattr(self, event, payload)
        elif event == "route":
            if payload.get("frame") == "map":
                self.route_points = payload.get("points", [])
                self._draw_route()
        elif event == "result":
            self._log(f"{payload['action']}：{payload['message']}")
            if not payload["success"]:
                QMessageBox.warning(self, "操作未通过", payload["message"])
        self._tick()

    def _tick(self):
        if not hasattr(self, "summary"):
            return
        if self.bridge:
            try:
                self.bridge.spin()
            except Exception as exc:
                self._log("ROS 通信异常：" + str(exc))
                self.bridge.close()
                self.bridge = None
        if self.offline_demo:
            self.summary.setText("离线交互演示｜不能控制车辆")
            self.prepare.setEnabled(True)
            self.activate.setEnabled(False)
            self.execute.setEnabled(False)
            self.pause.setEnabled(False)
            self.cancel.setEnabled(False)
            self.map_studio.setEnabled(False)
            self.mode_label.setText("DEMO · NO MOTION")
            return
        reason, may_prepare, may_execute = health_gate(
            self.selected_profile(), self.current_map,
            self.health, self.editor, self.guard)
        self.summary.setText(reason)
        active = self._active_navigation_present()
        self.activate.setEnabled(bool(self.bridge) and self.current_map is not None
                                 and not active)
        self.prepare.setEnabled(bool(self.bridge) and may_prepare)
        self.execute.setEnabled(bool(self.bridge) and may_execute)
        self.pause.setEnabled(bool(self.bridge) and self.editor is not None)
        self.cancel.setEnabled(bool(self.bridge))
        self.map_studio.setEnabled(True)
        self.mode_label.setText("导航 READY" if may_prepare else "运动禁止 / 等待就绪")
        h = self.health["data"] if self.health else {}
        e = self.editor["data"] if self.editor else {}
        self.status_values["loc"].setText(str(h.get("state", "未连接")))
        self.status_values["nav"].setText(
            "ACTIVE" if h.get("nav2_active") else "未就绪")
        self.status_values["base"].setText(
            "在线" if h.get("base_alive") else "未确认")
        self.status_values["safety"].setText(
            self.guard["data"].get("state", "Bunker 外部安全链")
            if self.selected_profile() == "fwmini" and self.guard else
            "由平台守卫负责" if self.selected_profile() == "bunker" else "未连接")
        self.status_values["route"].setText(str(e.get("status", "待预览")))
        if reason != self.last_reason:
            self.last_reason = reason
            self._log("状态：" + reason)

    def prepare_route(self):
        try:
            distance = validate_distance(self.distance.value())
            if self.offline_demo:
                # Synthetic preview is never sent to ROS/CAN.
                self.route_items = []
                self.scene.clear()
                self.map_image = None
                self.scene.addRect(0, 0, 580, 350, QPen(QColor("#425764")))
                self.route_points = [(80, 180), (80 + distance * 12, 180)]
                self._draw_route()
                self.route_label.setText(
                    f"离线示意：{distance:g} 米 · 不是真实 map 位姿 · 禁止执行")
                self._log(f"生成 {distance:g}m 离线示意路线")
                return
            reason, allowed, _ = health_gate(
                self.selected_profile(), self.current_map,
                self.health, self.editor, self.guard)
            if not allowed:
                raise ValueError(reason)
            p = self.current_map
            self.bridge.request("prepare", distance=distance,
                                map_id=p.map_id, version=p.version)
            self._log(f"请求 {distance:g}m 路线草稿；等待后端碰撞校验")
        except (ValueError, RuntimeError) as exc:
            QMessageBox.warning(self, "无法生成路线", str(exc))

    def execute_route(self):
        _, allowed, confirmed = health_gate(
            self.selected_profile(), self.current_map,
            self.health, self.editor, self.guard)
        if not self.bridge or not (allowed and confirmed):
            QMessageBox.warning(self, "执行已禁止", "地图、定位、安全或路线校验状态不满足条件。")
            return
        result = QMessageBox.question(
            self, "确认真实车辆运动",
            "确认已检查路线、现场人员、硬件急停和遥控器；"
            "按下【确认】将让真实机器人执行路线。",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if result == QMessageBox.Yes:
            self._request("execute")

    def _request(self, action):
        if not self.bridge:
            return
        try:
            self.bridge.request(action)
            self._log("发送高层任务请求：" + action)
        except (ValueError, RuntimeError) as exc:
            QMessageBox.warning(self, "服务不可用", str(exc))

    def _active_navigation_present(self):
        if self.health is not None and time.monotonic() - self.health["at"] <= 2.5:
            h = self.health["data"]
            if h.get("status") in (1, 2) or h.get("goal_active"):
                return True
        if self.bridge is not None:
            try:
                return self.bridge.navigation_nodes_present()
            except Exception:
                return True  # Unobservable ROS graph -> no hot activation.
        return False

    def activate_map(self):
        if self.offline_demo or not self.bridge or not self.current_map:
            return
        if self._active_navigation_present():
            QMessageBox.warning(
                self, "无法切换地图", "检测到导航正在运行或无法确认已停止，请先停止导航。")
            return
        m = self.current_map
        if QMessageBox.question(
                self, "确认激活地图",
                f"校验并激活地图 {m.label}？\n只准备下一次导航加载；不会热切换正在运行的节点。",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
            return
        proc = getattr(self, "activation_process", None)
        if proc is not None and proc.state() != QProcess.NotRunning:
            return
        proc = QProcess(self)
        proc.setProgram("ros2")
        proc.setArguments([
            "run", "agt_map_manager", "select_map_package",
            "--map-root", str(self.map_root),
            "--active-state-file", str(self.map_root / "active_map.yaml"),
            "--map-id", m.map_id, "--map-version", m.version,
        ])
        proc.finished.connect(
            lambda exit_code, _status, p=proc, name=m.label:
            self._map_activation_finished(p, name, exit_code))
        proc.errorOccurred.connect(
            lambda _: self._log("无法调用 Map Manager，请检查导航包是否已安装。"))
        self.activation_process = proc
        self.children_processes.append(proc)
        proc.start()
        self._log(f"正在检查地图资产并激活：{m.label}")

    def _map_activation_finished(self, process, name, exit_code):
        output = (bytes(process.readAllStandardOutput()) +
                  bytes(process.readAllStandardError())).decode(
                      "utf-8", "replace").strip()
        if exit_code == 0:
            self._log(f"地图激活完成：{name}。等待导航加载同一版本。")
            self.map_note.setText(
                f"已准备 {name}\n导航加载并确认该地图版本后才能执行。")
        else:
            self._log("地图激活失败：" + output[:700])
            QMessageBox.warning(
                self, "地图未通过校验",
                "地图包哈希或内容不符合要求。\n" + output[:700])

    def open_map_studio(self):
        if self.offline_demo:
            return
        if self.bridge is None:
            QMessageBox.warning(self, "ROS 未连接", "无法确认导航是否停止，禁止打开地图编辑。")
            return
        if self._active_navigation_present():
            QMessageBox.warning(self, "无法编辑活动地图",
                                "导航运行期间不允许从此界面启动地图编辑。请先独立停止导航。")
            return
        process = QProcess(self)
        process.setProgram("ros2")
        process.setArguments(["run", "agt_map_studio", "map_viewer"])
        process.readyReadStandardError.connect(
            lambda: self._log(bytes(process.readAllStandardError()).decode("utf-8", "replace")[:500]))
        process.errorOccurred.connect(
            lambda _: self._log("Map Studio 启动失败：请确认建图工作区已安装。"))
        process.start()
        self.children_processes.append(process)
        self._log("尝试打开 Map Studio；只启动地图编辑器")

    def open_report(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "查看已生成的验收报告", str(Path.home()),
            "验收报告 JSON (*.json)")
        if not path:
            return
        try:
            report = json.loads(Path(path).read_text(encoding="utf-8"))
            if report.get("schema_version") != 1 or report.get("result") not in (
                    "PASS", "FAIL", "INCOMPLETE"):
                raise ValueError("不是受支持的验收报告")
            nav = report.get("navigation", {}).get("measurements", [])
            details = "\n".join(
                f"{row.get('distance_m')}m × {row.get('count')}/{row.get('required_count')}："
                f"均值 {row.get('mean_error_m')}m → {row.get('result')}"
                for row in nav)
            speeds = report.get("speed", {})
            QMessageBox.information(
                self, "农科院验收结果",
                f"总体结果：{report['result']}\n{details}\n"
                f"30m 速度：{speeds.get('observed_gate_average_mps', [])}\n"
                "注意：仅包含定距和测速，不代表爬坡、续航和防撞验收。")
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            QMessageBox.warning(self, "报告读取失败", str(exc))

    def closeEvent(self, event):
        running = [p for p in self.children_processes if p.state() != QProcess.NotRunning]
        if running and QMessageBox.question(
                self, "关闭操作台",
                "已启动的地图编辑进程仍在运行。关闭操作台将结束本界面启动的进程，继续吗？",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
            event.ignore()
            return
        for process in running:
            process.terminate()
        if self.bridge:
            self.bridge.close()
        event.accept()


def main(argv=None):
    parser = argparse.ArgumentParser(description="AGT Qt5 统一导航操作台")
    parser.add_argument("--maps", default="~/agt_field_ws/maps")
    parser.add_argument("--robot", choices=tuple(PROFILES), default="fwmini")
    parser.add_argument("--offline-demo", action="store_true",
                        help="show interaction only: no ROS services or motion")
    args = parser.parse_args(argv)
    app = QApplication([sys.argv[0]])
    window = FieldConsoleWindow(map_root=args.maps, offline_demo=args.offline_demo,
                                default_robot=args.robot)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
