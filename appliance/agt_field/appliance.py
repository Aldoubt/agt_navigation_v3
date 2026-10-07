"""Operator lifecycle and controlled commands; Qt contains no business state machine."""

from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess
import threading
import time
from datetime import datetime, timezone

from .bundle import activate, build_localization_assets, seal, validate_bundle, validate_mapping
from .contracts import ContractError, atomic_yaml, identifier, read_yaml, route
from .mission import Mission
from .mock import mapping_fixture, localization_fixture, navigation_fixture
from .processes import Processes, overlay_command
from .profile import load_profile, calibration_errors, require_real


class Appliance:
    def __init__(self, data, profile, *, mock=False, versions=None):
        self.data = Path(data).resolve()
        self.profile = load_profile(profile)
        self.mock = mock
        if mock != (self.profile["robot"]["profile"] == "mock_yhs"):
            raise ContractError("mock flag/profile mismatch")
        for name in ["maps", "routes", "bags", "logs", "profiles", "diagnostics", "run"]:
            (self.data / name).mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.processes = Processes(self.data / "logs")
        self.mode = "IDLE"
        self.mapping = "STOPPED"
        self.localization = "STOPPED"
        self.recording = "STOPPED"
        self.bundle = None
        self.identity = None
        self.gateway_ready = mock
        self.chassis_rx = 0
        self.versions = versions or {}
        self.mapping_history = ["STOPPED"]
        self.last_error = ""
        self.worker = None
        self.record_started = 0
        self.record_path = ""
        self.mapping_started = 0
        self.run_dir = None
        self.runtime = None
        self.mock_goal = None
        self.mission = Mission(self.send_goal, self.cancel_goal)
        if not mock:
            from .runtime import RosRuntime

            self.runtime = RosRuntime(self)
        self.stop_event = threading.Event()
        if mock:
            self.mock_thread = threading.Thread(target=self.mock_tick, daemon=True)
            self.mock_thread.start()

    def send_goal(self, point, epoch):
        if self.mock:
            self.mock_goal = (epoch, time.monotonic() + 0.05)
        else:
            self.runtime.send(point, epoch)

    def cancel_goal(self, epoch):
        if self.mock:
            self.mock_goal = None
            self.mission.cancelled(epoch)
        else:
            self.runtime.cancel(epoch)

    def mock_tick(self):
        while not self.stop_event.wait(0.02):
            with self.lock:
                self.mission.readiness(self.localization == "READY", self.gateway_ready)
                if self.mock_goal and time.monotonic() >= self.mock_goal[1]:
                    epoch = self.mock_goal[0]
                    self.mock_goal = None
                    self.mission.result(epoch, True)
                self.mission.tick()

    def busy(self):
        return self.worker is not None and self.worker.is_alive()

    def task(self, function):
        if self.busy():
            raise ContractError("lifecycle operation already running")

        def execute():
            try:
                function()
            except Exception as exc:
                with self.lock:
                    self.last_error = str(exc)
                    self.mapping = "ERROR"

        self.worker = threading.Thread(target=execute, daemon=True)
        self.worker.start()

    def preflight(self):
        disk = shutil.disk_usage(self.data)
        errors = calibration_errors(self.profile)
        if disk.free < 1024**3:
            errors.append("insufficient free disk (<1 GiB)")
        sensors = self.devices()
        if not self.mock:
            for name, key in [("LiDAR", "lidar_min_hz"), ("IMU", "imu_min_hz")]:
                row = sensors[name]
                if (
                    row["state"] != "ONLINE"
                    or row.get("frequency", 0) < self.profile["sensors"][key]
                    or abs(row.get("timestamp_age", 1e9)) > self.profile["sensors"]["stale_sec"]
                ):
                    errors.append(name + " frequency/timestamp not ready")
            for frame in [
                self.profile["robot"].get("lidar_frame"),
                self.profile["robot"].get("imu_frame"),
            ]:
                if frame and frame not in self.runtime.tf:
                    errors.append("TF frame unavailable: " + frame)
            if self.mapping not in {"RUNNING", "PROCESSING"}:
                if not Path("/opt/mapping_ws/install/setup.bash").is_file():
                    errors.append("mapping runtime missing")
        return dict(
            pass_=not errors,
            errors=errors,
            warnings=[]
            if self.gateway_ready
            else ["wheel odom/CAN not ready; mapping does not depend on wheel odom"],
            disk_free=disk.free,
            devices=sensors,
        )

    def devices(self):
        if self.mock:
            return {
                k: dict(
                    state="ONLINE" if k != "CAN" else "ACTIVE",
                    frequency=50 if k == "Wheel Odom" else 100,
                    last_timestamp=time.time(),
                )
                for k in ["LiDAR", "IMU", "CAN", "YHS", "Wheel Odom"]
            }
        result = {k: self.runtime.sensor_status(k) for k in ["LiDAR", "IMU", "Wheel Odom"]}
        result["CAN"] = dict(state="ACTIVE" if self.gateway_ready else "DOWN")
        result["YHS"] = dict(
            state="ONLINE" if time.monotonic() - self.chassis_rx < 0.5 else "OFFLINE"
        )
        return result

    def status(self):
        with self.lock:
            return dict(
                robot="YHS",
                mock=self.mock,
                mode=self.mode,
                mapping=self.mapping,
                localization=self.localization,
                recording=self.recording,
                recording_duration=time.monotonic() - self.record_started
                if self.recording == "RUNNING"
                else 0,
                bag_path=self.record_path,
                map=self.mission.binding or {},
                bundle_path=str(self.bundle or ""),
                mapping_duration=time.monotonic() - self.mapping_started
                if self.mapping == "RUNNING"
                else 0,
                mission=self.mission.status(),
                mapping_history=self.mapping_history,
                devices=self.devices(),
                disk_free=shutil.disk_usage(self.data).free,
                sensors=self.profile["sensors"],
                base=self.profile["base"],
                last_error=self.last_error,
                profiles=str(self.profile["root"]),
                busy=self.busy(),
            )

    def no_mission(self):
        if (
            self.mission.state in {"RUNNING", "NAVIGATING", "DWELLING", "PAUSED"}
            or self.mission.cancel_pending
        ):
            raise ContractError("cancel mission and wait for cancellation first")

    def validate_active(self):
        if not self.bundle:
            raise ContractError("no active map")
        binding = validate_bundle(self.bundle, allow_mock=self.mock)
        if binding != self.mission.binding:
            raise ContractError("active bundle changed")
        return binding

    def mapping_start(self, request):
        self.no_mission()
        if self.busy() or self.mode == "NAVIGATION" or self.mapping == "RUNNING":
            raise ContractError("mapping mode conflicts with current lifecycle")
        check = self.preflight()
        if not check["pass_"]:
            raise ContractError("; ".join(check["errors"]))
        map_id = identifier(request.get("map_bundle_id"))
        version = identifier(str(request.get("map_version", "1")))
        target = self.data / "maps" / map_id / version
        if target.exists():
            raise ContractError("map/version already exists")
        self.identity = dict(map_bundle_id=map_id, map_version=version)
        self.bundle = target
        self.run_dir = (
            self.data
            / "bags"
            / ("mapping_" + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f"))
        )
        self.mode = "MAPPING"
        self.mapping = "RUNNING"
        self.mapping_history.extend(["START", "MAPPING"])
        self.mapping_started = time.monotonic()
        self.localization = "STOPPED"
        self.mission.binding = None
        self.mission.data = None
        if self.mock:
            self.run_dir.mkdir()
        else:
            self.processes.stop("sensor")
            p = self.profile
            cfg = p["root"] / p["localization"]["fastlio_config"]
            cmd = [
                "bash",
                "/opt/mapping_ws/src/agt-lio-pgo-mapping/scripts/run_mid360_live_mapping.sh",
                str(self.run_dir),
                "--robot",
                "yhs_v1",
                "--livox-config",
                str(p["root"] / p["sensors"]["livox_config"]),
                "--fastlio-config",
                str(cfg),
                "--setup",
                "/opt/mapping_ws/install/setup.bash",
                "--domain-id",
                str(p["mapping"]["domain_id"]),
                "--no-rviz",
            ]
            self.processes.start("mapping", cmd)

    def mapping_finish(self):
        if self.mapping != "RUNNING":
            raise ContractError("mapping not running")
        self.mapping = "FINISH"
        self.task(self.build_mapping)

    def build_mapping(self):
        if self.mock:
            mapping_fixture(self.bundle / "mapping")
            self.mapping = "PROCESSING"
            time.sleep(0.05)
        else:
            # Existing supervisor accepts a unique output-root STOP_MAPPING file.
            # Its normal finish path closes bag, drains LIO/PGO, exports and verifies.
            (self.run_dir / "STOP_MAPPING").touch(exist_ok=False)
            process = self.processes.owned["mapping"]
            code = process.wait(timeout=self.profile["mapping"]["export_timeout_sec"])
            if code != 0:
                raise ContractError(f"mapping lifecycle failed: {code}")
            state = json.loads((self.run_dir / "session.json").read_text())
            if state.get("artifact_verified") is not True:
                raise ContractError("mapping session did not verify artifact")
            source = self.run_dir / "map_package"
            validate_mapping(source)
            self.bundle.mkdir(parents=True, exist_ok=False)
            shutil.copytree(source, self.bundle / "mapping")
        self.mapping_history.extend(["FINISH", "PROCESSING", "VERIFY"])
        self.mapping = "VERIFY"
        validate_mapping(self.bundle / "mapping")
        self.mapping = "LOCALIZATION_ASSET_BUILD"
        self.mapping_history.append(self.mapping)
        if self.mock:
            localization_fixture(
                self.bundle / "localization", self.bundle / "mapping", self.identity
            )
        else:
            build_localization_assets(
                self.bundle / "mapping",
                self.bundle / "localization",
                self.identity,
                self.versions,
                self.profile["localization"],
            )
        self.mapping = "GRID_BUILD"
        self.mapping_history.append(self.mapping)
        if self.mock:
            navigation_fixture(self.bundle / "navigation", self.bundle / "mapping", confirmed=False)
        else:
            subprocess.run(
                overlay_command(
                    "/opt/mapping_ws/install/setup.bash",
                    [
                        "ros2",
                        "run",
                        "agt_pcd2grid_exporter",
                        "pcd2grid_exporter",
                        "--package",
                        str(self.bundle / "mapping"),
                        "--config",
                        str(self.profile["root"] / self.profile["mapping"]["projection_config"]),
                        "--output",
                        str(self.bundle / "navigation_base"),
                    ],
                ),
                check=True,
            )
        self.mapping = "REVIEW_REQUIRED"
        self.mapping_history.append(self.mapping)

    def review(self):
        if self.mapping != "REVIEW_REQUIRED" or self.busy():
            raise ContractError("map not ready for review")
        if self.mock:
            return dict(
                message="Mock review requires explicit Confirm; no editor is simulated as real"
            )
        self.processes.start(
            "review",
            overlay_command(
                "/opt/mapping_ws/install/setup.bash",
                [
                    "ros2",
                    "run",
                    "agt_map_studio",
                    "map_viewer",
                    "--review-package",
                    str(self.bundle / "mapping"),
                    "--review-map",
                    str(self.bundle / "navigation_base/map.yaml"),
                    "--review-output",
                    str(self.bundle / "navigation"),
                ],
            ),
        )

    def confirm(self):
        if self.mapping != "REVIEW_REQUIRED" or self.busy():
            raise ContractError("map not reviewable")
        if self.mock:
            atomic_yaml(
                self.bundle / "navigation/review_status.yaml",
                dict(
                    status="confirmed",
                    source_mapping_package=str((self.bundle / "mapping").resolve()),
                ),
            )
        # MapStudio output must attest the same source; builder never modifies mapping/localization.
        if not self.mock and self.processes.running("review"):
            raise ContractError("close MapStudio after Confirm & Save")
        # Base grid is a build artifact, not a runtime consumer. Keep it outside sealed bundle.
        if (self.bundle / "navigation_base").exists():
            shutil.move(str(self.bundle / "navigation_base"), str(self.run_dir / "navigation_base"))
        seal(self.bundle, self.identity, self.versions, mock=self.mock)
        self.mapping = "READY"
        self.mapping_history.extend(["REVIEW_CONFIRMED", "READY"])

    def activate(self, request):
        self.no_mission()
        if self.busy() or self.mapping in {"RUNNING", "FINISH", "PROCESSING"}:
            raise ContractError("mapping/lifecycle still active")
        target = (
            self.data
            / "maps"
            / identifier(request["map_bundle_id"])
            / identifier(str(request["map_version"]))
        )
        self.processes.stop("navigation")
        self.processes.stop("localization")
        binding = activate(target, self.data / "run/active_map.yaml", allow_mock=self.mock)
        self.bundle = target
        self.identity = {k: binding[k] for k in ["map_bundle_id", "map_version"]}
        self.mission.bind(binding)
        self.localization = "STOPPED"
        self.mode = "IDLE"

    def navigation_start(self):
        self.no_mission()
        self.validate_active()
        require_real(self.profile, motion=True)
        if self.busy() or self.mode == "MAPPING" and self.mapping != "READY":
            raise ContractError("mapping unfinished")
        self.mode = "NAVIGATION"
        self.localization = "RELOCALIZING"
        if self.mock:
            self.localization = "READY"
            return
        from .launch_config import write_navigation_params

        params = write_navigation_params(self.profile, self.data / "run/nav2_params.yaml")
        self.start_sensor()
        if not self.processes.running("robot_description"):
            self.processes.start(
                "robot_description",
                [
                    "ros2",
                    "run",
                    "robot_state_publisher",
                    "robot_state_publisher",
                    str(self.profile["root"] / self.profile["robot"]["urdf"]),
                ],
            )
        self.processes.start(
            "localization",
            [
                "ros2",
                "launch",
                "agt_system_bringup",
                "localization.launch.py",
                "global_map:=" + str(self.bundle / "mapping/map.pcd"),
                "relocalization_assets:=" + str(self.bundle / "localization"),
                "lio_backend:=fastlio2",
                "fastlio_config:="
                + str(self.profile["root"] / self.profile["localization"]["fastlio_config"]),
                "enable_map_tracking:=true",
                "auto_relocalize:=true",
            ],
        )
        # Nav2 retains V3 configuration/controller. Start action server without sending goals.
        self.processes.start(
            "navigation",
            [
                "ros2",
                "launch",
                "agt_mission_executor",
                "field_navigation.launch.py",
                "map:=" + str(self.bundle / "navigation/map.yaml"),
                "params_file:=" + str(params),
            ],
        )

    def start_sensor(self):
        if self.mock:
            return
        require_real(self.profile)
        if self.processes.running("sensor"):
            return
        if not self.processes.running("robot_description"):
            self.processes.start(
                "robot_description",
                [
                    "ros2",
                    "run",
                    "robot_state_publisher",
                    "robot_state_publisher",
                    str(self.profile["root"] / self.profile["robot"]["urdf"]),
                ],
            )
        self.processes.start(
            "sensor",
            [
                "ros2",
                "run",
                "livox_ros_driver2",
                "livox_ros_driver2_node",
                "--ros-args",
                "-p",
                "xfer_format:=1",
                "-p",
                "multi_topic:=0",
                "-p",
                "publish_freq:=" + str(self.profile["sensors"]["driver_rate_hz"]),
                "-p",
                "user_config_path:="
                + str(self.profile["root"] / self.profile["sensors"]["livox_config"]),
                "-p",
                "frame_id:=livox_frame",
            ],
        )

    def recording_start(self):
        if self.recording == "RUNNING":
            raise ContractError("recorder already running")
        self.record_path = str(
            self.data
            / "bags"
            / ("record_" + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f"))
        )
        if self.mock:
            Path(self.record_path).mkdir()
        else:
            self.processes.start(
                "recorder",
                [
                    "ros2",
                    "bag",
                    "record",
                    "--output",
                    self.record_path,
                    *self.profile["topics"]["recording"],
                ],
            )
        self.recording = "RUNNING"
        self.record_started = time.monotonic()

    def check_processes(self):
        if self.mode == "NAVIGATION" and any(
            name in self.processes.owned and not self.processes.running(name)
            for name in ["navigation", "localization", "sensor"]
        ):
            self.localization = "ERROR"
            self.mission.stop("ERROR", "critical runtime process exited")
        if self.recording == "RUNNING" and not self.processes.running("recorder"):
            self.recording = "ERROR"
        if self.mapping == "RUNNING" and not self.processes.running("mapping"):
            self.mapping = "ERROR"

    def command(self, request):
        if request.get("command") == "DOCTOR":
            from .diagnostics import diagnostic_zip

            return dict(report_path=str(diagnostic_zip(self.data, self.profile, self.status())))
        with self.lock:
            command = request.get("command")
            if command == "STATUS":
                return self.status()
            if command == "PREFLIGHT":
                return self.preflight()
            if command == "START_MAPPING":
                self.mapping_start(request)
            elif command == "STOP_MAPPING":
                self.mapping_finish()
            elif command == "REVIEW_MAP":
                self.review()
            elif command == "CONFIRM_MAP":
                self.confirm()
            elif command == "ACTIVATE_MAP":
                self.activate(request)
            elif command == "START_NAVIGATION":
                self.navigation_start()
            elif command == "START_SENSOR":
                self.start_sensor()
            elif command == "STOP_SENSOR":
                self.no_mission()
                self.processes.stop("sensor")
            elif command == "TEST_SENSOR":
                return self.preflight()
            elif command == "SAVE_ROUTE":
                self.validate_active()
                d = route(request["route"], self.mission.binding)
                atomic_yaml(self.data / "routes" / (identifier(d["route_id"]) + ".yaml"), d)
                self.mission.load(d)
            elif command == "LOAD_ROUTE":
                self.validate_active()
                d = route(
                    read_yaml(self.data / "routes" / (identifier(request["route_id"]) + ".yaml")),
                    self.mission.binding,
                )
                self.mission.load(d)
                return d
            elif command == "START":
                self.validate_active()
                self.mission.start()
            elif command == "PAUSE":
                self.mission.pause()
            elif command == "RESUME":
                self.validate_active()
                self.mission.resume()
            elif command == "CANCEL":
                self.mission.stop()
            elif command == "START_RECORDING":
                self.recording_start()
            elif command == "STOP_RECORDING":
                self.processes.stop("recorder")
                self.recording = "STOPPED"
            elif command == "STOP_ALL":
                self.mission.stop()
                self.processes.shutdown()
                self.localization = "STOPPED"
                self.mapping = "CANCELLED"
                self.recording = "STOPPED"
                self.mode = "IDLE"
            elif command == "IDLE":
                self.no_mission()
                self.processes.stop("navigation")
                self.processes.stop("localization")
                self.mode = "IDLE"
                self.localization = "STOPPED"
            elif command in {"CAN_UP", "CAN_DOWN"}:
                if self.mock:
                    self.gateway_ready = command == "CAN_UP"
                else:
                    from .host_control import request as host_request

                    return host_request(self.data, command)
            elif command == "DOCTOR":
                from .diagnostics import diagnostic_zip

                return dict(report_path=str(diagnostic_zip(self.data, self.profile, self.status())))
            elif command == "MOCK_FAULT":
                if not self.mock:
                    raise ContractError("fault injection is mock-only")
                fault = request["fault"]
                if fault == "localization":
                    self.localization = "LOST"
                    self.mission.readiness(False, self.gateway_ready)
                elif fault == "gateway":
                    self.gateway_ready = False
                    self.mission.readiness(self.localization == "READY", False)
                elif fault == "nav":
                    self.mock_goal = None
                    self.mission.result(self.mission.epoch, False, "mock Nav2 failure")
                else:
                    raise ContractError("unknown mock fault")
            else:
                raise ContractError("unknown command")
            return self.status()

    def shutdown(self):
        with self.lock:
            self.mission.stop()
            self.stop_event.set()
        self.processes.shutdown()
        if self.runtime:
            self.runtime.shutdown()
        if self.worker:
            self.worker.join(timeout=5)
