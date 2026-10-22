"""Safe preview GUI for the FW-mini experimental profile.

The GUI owns only preview processes that it starts itself. It deliberately has
no CAN driver, chassis adapter, /cmd_vel publisher, localization, or Nav2 start
command because the FW-mini profile and map compatibility are not validated.
"""

from __future__ import annotations

import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time
from typing import Any

import yaml

from .fwmini_profile import (
    default_candidate_path,
    default_profile_path,
    has_complete_lidar_transform,
    load_profile,
    optional_vector,
    save_candidate,
)


MAP_NODE_NAMES = {
    '/agt_academy_preview/academy_map_preview_server',
    '/agt_academy_preview/academy_map_preview_lifecycle',
    '/agt_academy_preview/academy_map_preview_rviz',
}
MODEL_NODE_NAMES = {
    '/fwmini_profile_preview_state_publisher',
    '/fwmini_profile_preview_rviz',
}
MAP_YAML = Path(
    '/home/yangxuan/ros2_ws/maps/bunker_mid360/'
    '20260924-trav-integration-v1/navigation/bunker_v1/map.yaml'
)


def _ament_share(package: str) -> Path:
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory(package))


def _ros2_nodes(timeout_sec: float = 4.0) -> tuple[set[str] | None, str]:
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list', '--no-daemon'], check=False, text=True,
            capture_output=True, timeout=timeout_sec,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, f'ROS graph query failed: {exc}'
    if result.returncode != 0:
        return None, (result.stderr.strip() or f'ros2 node list exit {result.returncode}')
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}, ''


class FieldTestGui:
    def __init__(self, root: Any, *, profile_path: Path | None = None) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.ttk = ttk
        self.root = root
        self.profile_path = profile_path or default_profile_path()
        self.profile = load_profile(self.profile_path)
        self.candidate_path = default_candidate_path()
        if self.candidate_path.is_file():
            try:
                self.profile = load_profile(self.candidate_path)
            except ValueError:
                # Keep the shipped template intact if an old candidate is malformed.
                self.profile = load_profile(self.profile_path)

        self.children: dict[str, subprocess.Popen[str]] = {}
        self.output_queue: queue.Queue[tuple[str, str]] = queue.Queue()
        self._close_after_stop = False
        self._refresh_after: str | None = None
        self.status_vars = {
            'map': tk.StringVar(value='未启动'),
            'model': tk.StringVar(value='未启动'),
            'lidar': tk.StringVar(value='BLOCKED：缺少 FW-mini 专用雷达驱动/IP 配置'),
            'localization': tk.StringVar(value='BLOCKED：雷达外参和地图兼容性未验证'),
            'nav2': tk.StringVar(value='BLOCKED：舵轮 Nav2 参数尚未实现'),
            'base': tk.StringVar(value='BLOCKED：档位与标准底盘状态门控未验证'),
        }
        mount = self.profile.get('lidar_mount', {})
        self.xyz_vars = [tk.StringVar(value=self._display_optional(v)) for v in mount.get('xyz_m', [None] * 3)]
        self.rpy_vars = [tk.StringVar(value=self._display_optional(v)) for v in mount.get('rpy_rad', [None] * 3)]
        self.domain_var = tk.StringVar(value=os.environ.get('ROS_DOMAIN_ID', '0 (ROS 默认值)'))
        self.env_var = tk.StringVar(value='ROS 环境尚未检查')
        self.profile_var = tk.StringVar(value=str(self.candidate_path))
        self._build()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.after(100, self._drain_output)
        self.root.after(700, self.refresh_status)

    @staticmethod
    def _display_optional(value: Any) -> str:
        return '' if value is None else str(value)

    def _build(self) -> None:
        tk, ttk = self.tk, self.ttk
        self.root.title('AGT 测试启动台 · FW-mini 舵轮预览')
        self.root.geometry('1000x780')
        self.root.minsize(900, 680)

        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill='both', expand=True)
        ttk.Label(outer, text='FW-mini 测试启动台', font=('TkDefaultFont', 18, 'bold')).pack(anchor='w')
        ttk.Label(
            outer,
            text='当前可启动：隔离的车体模型预览、农科院地图静态预览。'
                 '传感器、定位、Nav2 和底盘入口会在配置/验收不足时保持阻塞。',
            wraplength=940,
        ).pack(anchor='w', pady=(2, 8))

        env = ttk.LabelFrame(outer, text='运行环境', padding=8)
        env.pack(fill='x', pady=(0, 8))
        ttk.Label(env, text='ROS_DOMAIN_ID：').grid(row=0, column=0, sticky='w')
        ttk.Label(env, textvariable=self.domain_var).grid(row=0, column=1, sticky='w')
        ttk.Label(env, textvariable=self.env_var, wraplength=780).grid(row=1, column=0, columnspan=2, sticky='w')
        ttk.Button(env, text='刷新状态', command=self.refresh_status).grid(row=0, column=2, rowspan=2, padx=6)

        profile = ttk.LabelFrame(outer, text='FW-mini 车体档案与雷达外参候选', padding=8)
        profile.pack(fill='x', pady=(0, 8))
        body = self.profile['body']
        ttk.Label(
            profile,
            text=(f"车体外形预览：长 {body['length_m']:.3f} m × 宽 {body['width_m']:.3f} m "
                  f"× 高 {body['height_m']:.3f} m　|　来源：{body['source']}（未实测）"),
            wraplength=940,
        ).grid(row=0, column=0, columnspan=7, sticky='w', pady=(0, 5))
        ttk.Label(
            profile,
            text='参考原点：base_link 底面中心。位置用米，姿态 RPY 用弧度；留空时不发布 livox_frame。',
            wraplength=940,
        ).grid(row=1, column=0, columnspan=7, sticky='w', pady=(0, 8))
        for row, label, variables in ((2, 'XYZ (m)', self.xyz_vars), (3, 'RPY (rad)', self.rpy_vars)):
            ttk.Label(profile, text=label).grid(row=row, column=0, sticky='w')
            for col, (axis, variable) in enumerate(zip(('X', 'Y', 'Z'), variables), start=1):
                ttk.Label(profile, text=axis).grid(row=row, column=2 * col - 1, padx=(5, 0))
                ttk.Entry(profile, textvariable=variable, width=13).grid(row=row, column=2 * col, padx=(2, 5))
        ttk.Button(profile, text='保存外参候选', command=self.save_profile_candidate).grid(
            row=2, column=7, rowspan=2, padx=(8, 0), sticky='ns')
        ttk.Label(profile, textvariable=self.profile_var, wraplength=930).grid(
            row=4, column=0, columnspan=8, sticky='w', pady=(7, 0))

        actions = ttk.LabelFrame(outer, text='静态预览启动', padding=8)
        actions.pack(fill='x', pady=(0, 8))
        ttk.Button(actions, text='启动车体模型预览', command=self.start_model_preview).grid(row=0, column=0, padx=4)
        ttk.Button(actions, text='启动农科院地图预览', command=self.start_map_preview).grid(row=0, column=1, padx=4)
        ttk.Button(actions, text='启动静态演示序列', command=self.start_preview_sequence).grid(row=0, column=2, padx=4)
        ttk.Button(actions, text='停止本工具启动的预览', command=self.stop_all).grid(row=0, column=3, padx=4)
        ttk.Label(
            actions,
            text='模型 TF 使用 fwmini_preview/ 前缀；地图发布到 /agt_academy_preview/map，避免覆盖现场 /tf 和 /map。',
            wraplength=930,
        ).grid(row=1, column=0, columnspan=4, sticky='w', pady=(7, 0))

        stages = ttk.LabelFrame(outer, text='启动门控状态', padding=8)
        stages.pack(fill='x', pady=(0, 8))
        rows = [
            ('地图预览', 'map'), ('车体模型', 'model'), ('MID360/传感器', 'lidar'),
            ('LIO/重定位', 'localization'), ('Nav2', 'nav2'), ('底盘控制', 'base'),
        ]
        for index, (label, key) in enumerate(rows):
            ttk.Label(stages, text=label, width=20).grid(row=index, column=0, sticky='w', pady=1)
            ttk.Label(stages, textvariable=self.status_vars[key], wraplength=820).grid(
                row=index, column=1, sticky='w', pady=1)

        safety = ttk.LabelFrame(outer, text='安全边界', padding=8)
        safety.pack(fill='x', pady=(0, 8))
        ttk.Label(
            safety,
            text='本工具不启动 CAN、不启用 /fwmini/enable_motion、不发布 /cmd_vel，'
                 '也不停止它未创建的节点。FW-mini 当前没有接入 AGT 的经过验证 Nav2/底盘链。',
            wraplength=940,
        ).pack(anchor='w')

        log_frame = ttk.LabelFrame(outer, text='操作日志', padding=6)
        log_frame.pack(fill='both', expand=True)
        self.log = tk.Text(log_frame, height=10, wrap='word', state='disabled')
        scrollbar = ttk.Scrollbar(log_frame, orient='vertical', command=self.log.yview)
        self.log.configure(yscrollcommand=scrollbar.set)
        self.log.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')
        self._append_log('启动台已就绪。先填写并保存雷达相对 base_link 的 XYZ/RPY 候选值。')

    def _append_log(self, message: str) -> None:
        self.log.configure(state='normal')
        self.log.insert('end', message.rstrip() + '\n')
        self.log.see('end')
        self.log.configure(state='disabled')

    def refresh_status(self) -> None:
        if self._refresh_after is not None:
            try:
                self.root.after_cancel(self._refresh_after)
            except Exception:
                pass
            self._refresh_after = None
        try:
            nodes, reason = _ros2_nodes()
        except Exception as exc:  # UI should report status rather than crash.
            nodes, reason = None, str(exc)
        if nodes is None:
            self.env_var.set(f'ROS graph 不可读：{reason}')
            self.status_vars['map'].set('UNKNOWN：无法读取 ROS graph')
            self.status_vars['model'].set('UNKNOWN：无法读取 ROS graph')
        else:
            self.env_var.set(f'ROS graph 可读；当前 ROS_DISTRO={os.environ.get("ROS_DISTRO", "未设置")}')
            self._set_external_or_owned_status('map', MAP_NODE_NAMES, nodes)
            self._set_external_or_owned_status('model', MODEL_NODE_NAMES, nodes)
        self.domain_var.set(os.environ.get('ROS_DOMAIN_ID', '0 (ROS 默认值)'))
        if has_complete_lidar_transform(self.profile):
            self.status_vars['localization'].set('BLOCKED：候选外参已填写但未实测，地图兼容仍未验证')
        else:
            self.status_vars['localization'].set('BLOCKED：雷达 XYZ/RPY 未完整填写，地图兼容性未验证')
        self._refresh_after = self.root.after(4000, self.refresh_status)

    def _set_external_or_owned_status(self, key: str, expected: set[str], nodes: set[str]) -> None:
        child = self.children.get(key)
        live = child is not None and child.poll() is None
        matching = expected.intersection(nodes)
        if live:
            if expected.issubset(nodes):
                self.status_vars[key].set('READY：本工具管理的 launch 和预期节点均存活')
            else:
                self.status_vars[key].set(
                    f'STARTED：等待节点；当前 {len(matching)}/{len(expected)} 个预期节点可见')
        elif matching:
            self.status_vars[key].set(f'EXTERNAL：检测到节点 {", ".join(sorted(matching))}；不会重复启动或停止')
        else:
            self.status_vars[key].set('READY：可启动静态预览')

    def _read_transform_fields(self) -> tuple[list[float | None], list[float | None]]:
        return optional_vector([v.get() for v in self.xyz_vars]), optional_vector([v.get() for v in self.rpy_vars])

    def save_profile_candidate(self, *, quiet: bool = False) -> Path | None:
        try:
            xyz, rpy = self._read_transform_fields()
            path = save_candidate(self.profile, xyz, rpy, self.candidate_path)
            self.profile = load_profile(path)
            self.profile_var.set(f'候选档案：{path}（field_verified=false，不会覆盖正式配置）')
            self._append_log(f'已保存候选档案：{path}')
            if not quiet:
                from tkinter import messagebox

                messagebox.showinfo('已保存', f'雷达外参候选已保存到：\n{path}\n\n没有修改正式配置或生产 TF。')
            return path
        except (OSError, ValueError, yaml.YAMLError) as exc:
            self._append_log(f'保存失败：{exc}')
            from tkinter import messagebox

            messagebox.showerror('无法保存', str(exc))
            return None

    def _precheck_packages(self, packages: tuple[str, ...]) -> tuple[bool, str]:
        try:
            for package in packages:
                result = subprocess.run(
                    ['ros2', 'pkg', 'prefix', package], check=False, text=True,
                    capture_output=True, timeout=4.0,
                )
                if result.returncode != 0:
                    return False, f'缺少 ROS package {package}；请 source 对应 overlay。'
        except (OSError, subprocess.TimeoutExpired) as exc:
            return False, f'ROS 环境检查失败：{exc}'
        return True, ''

    def _start(self, key: str, command: list[str], expected: set[str], packages: tuple[str, ...]) -> None:
        child = self.children.get(key)
        if child is not None and child.poll() is None:
            self._append_log(f'{key}: 已由本工具启动，跳过重复启动。')
            return
        ready, reason = self._precheck_packages(packages)
        if not ready:
            self._append_log(f'{key}: BLOCKED：{reason}')
            from tkinter import messagebox

            messagebox.showerror('启动被阻止', reason)
            return
        nodes, reason = _ros2_nodes()
        if nodes is None:
            self._append_log(f'{key}: BLOCKED：无法确认 ROS graph，{reason}')
            from tkinter import messagebox

            messagebox.showerror('启动被阻止', f'无法确认节点是否重复：\n{reason}')
            return
        collision = expected.intersection(nodes)
        if collision:
            self._append_log(f'{key}: BLOCKED：已有同名节点，未启动：{", ".join(sorted(collision))}')
            from tkinter import messagebox

            messagebox.showwarning(
                '检测到已有节点',
                '发现同名节点。为避免重复发布，本工具不会启动第二份，也不会停止外部节点：\n'
                + '\n'.join(sorted(collision)),
            )
            return
        self._append_log('$ ' + ' '.join(command))
        try:
            process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True, env=os.environ.copy(),
            )
        except OSError as exc:
            self._append_log(f'{key}: 启动失败：{exc}')
            return
        self.children[key] = process
        self.status_vars[key].set('STARTED：等待节点和话题 READY')
        threading.Thread(target=self._read_output, args=(key, process), daemon=True).start()

    def _read_output(self, key: str, process: subprocess.Popen[str]) -> None:
        stream = process.stdout
        if stream is None:
            return
        try:
            for line in stream:
                self.output_queue.put((key, line.rstrip()))
        finally:
            self.output_queue.put((key, f'[process exit={process.poll()}]'))

    def _drain_output(self) -> None:
        try:
            while True:
                key, line = self.output_queue.get_nowait()
                self._append_log(f'[{key}] {line}')
        except queue.Empty:
            pass
        for key, process in list(self.children.items()):
            code = process.poll()
            if code is not None:
                self.status_vars[key].set(f'STOPPED：进程退出码 {code}')
        if self._close_after_stop and all(p.poll() is not None for p in self.children.values()):
            self.root.destroy()
            return
        self.root.after(100, self._drain_output)

    def start_model_preview(self) -> None:
        candidate = self.save_profile_candidate(quiet=True)
        if candidate is None:
            return
        command = [
            'ros2', 'launch', 'agt_operator_console', 'fwmini_profile_preview.launch.py',
            f'profile_file:={candidate}',
        ]
        self._start('model', command, MODEL_NODE_NAMES, ('agt_operator_console', 'robot_state_publisher', 'rviz2'))

    def start_map_preview(self) -> None:
        if not MAP_YAML.is_file():
            self._append_log(f'地图文件不存在：{MAP_YAML}')
            return
        command = ['ros2', 'launch', 'agt_system_bringup', 'academy_map_preview.launch.py', f'map:={MAP_YAML}']
        self._start('map', command, MAP_NODE_NAMES, ('agt_system_bringup', 'nav2_map_server', 'rviz2'))

    def start_preview_sequence(self) -> None:
        self.start_model_preview()
        self.root.after(800, self.start_map_preview)

    def stop_all(self) -> None:
        for key, process in reversed(list(self.children.items())):
            if process.poll() is None:
                self._append_log(f'停止本工具管理的 {key} 预览进程组（SIGINT）')
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        self.root.after(2500, self._finish_stop)

    def _finish_stop(self) -> None:
        for key, process in list(self.children.items()):
            if process.poll() is None:
                self._append_log(f'{key} 未响应 Ctrl-C；发送 SIGTERM 到本工具创建的进程组。')
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        if self._close_after_stop:
            self.root.after(500, self._finish_stop)

    def _on_close(self) -> None:
        active = [p for p in self.children.values() if p.poll() is None]
        if not active:
            self.root.destroy()
            return
        self._close_after_stop = True
        self.stop_all()


def main() -> None:
    try:
        import tkinter as tk
    except ImportError as exc:
        raise SystemExit('Tkinter is missing. Install python3-tk, then source ROS 2 Humble.') from exc
    root = tk.Tk()
    FieldTestGui(root)
    root.mainloop()


if __name__ == '__main__':
    main()
