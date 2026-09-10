from __future__ import annotations

import os
import tempfile
import json
import shutil
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
import yaml
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from agt_robot_interfaces.action import GenerateMapPackage
from agt_robot_interfaces.msg import MapEditSession, MapPackage, MapStatus
from agt_robot_interfaces.srv import (
    CancelMapEdit,
    ListMapPackages,
    LoadMapPackage,
    PublishMapEdit,
    StartMapEdit,
    ValidateMap,
    ActivateMap,
    DiscardMap,
)
from std_msgs.msg import String

from .edit_session import (
    EditSessionInfo,
    create_edit_session,
    load_edit_session,
    set_edit_session_state,
    validate_edit_session_for_publish,
)
from .map_package import PackageInfo, discover_packages, validate_package
from .create_map_package import build_package
from .map_pipeline import generate_map_package
from .promote_hmi_navigation_edit import promote
from .map_events import MAP_ACTIVATED, MAP_GENERATED, MAP_VALIDATED, decode_event, encode_event
from .map_registry import MapRegistry, MapState
from .map_validator import validate_map_package, write_validation_report


class MapManager(Node):
    """Own validated Map Package selection and HMI map-edit lifecycle.

    A released Map Package is immutable.  The HMI receives an edit session that
    contains only a staging copy of the navigation map/topology.  Publication is
    accepted only if the session still matches the base package's immutable grid
    geometry contract; a successful publication creates a new map version.

    Selection remains intentionally separate from runtime application: writing
    active_map.yaml is not the same thing as atomically reloading Nav2 and the
    3D localization backend. Runtime consumers use typed MapStatus generations
    so every backend can apply one validated package generation together.
    """

    def __init__(self) -> None:
        super().__init__('agt_map_manager')
        self.declare_parameter('map_root', '/home/yangxuan/ros2_ws/agt_data/maps')
        self.declare_parameter(
            'active_state_file', '/home/yangxuan/ros2_ws/agt_data/maps/active_map.yaml')
        self.declare_parameter('edit_root', '/home/yangxuan/ros2_ws/agt_data/map_edits')
        self.declare_parameter('verify_hashes_on_discovery', False)
        self.declare_parameter('verify_hashes_on_load', True)
        self.declare_parameter('status_topic', '/agt/map/status')
        self.declare_parameter('list_service', '/agt/map/list')
        self.declare_parameter('load_service', '/agt/map/load')
        self.declare_parameter('start_edit_service', '/agt/map/edit/start')
        self.declare_parameter('publish_edit_service', '/agt/map/edit/publish')
        self.declare_parameter('cancel_edit_service', '/agt/map/edit/cancel')
        self.declare_parameter('generate_action', '/agt/map/generate')
        self.declare_parameter('registry_file', '/home/yangxuan/ros2_ws/agt_data/maps/map_registry.yaml')
        self.declare_parameter('events_topic', '/agt/map/events')
        self.declare_parameter('validate_service', '/agt/map/validate')
        self.declare_parameter('activate_service', '/agt/map/activate')
        self.declare_parameter('discard_service', '/agt/map/discard')

        self._root = Path(str(self.get_parameter('map_root').value)).expanduser().resolve()
        self._state_file = Path(
            str(self.get_parameter('active_state_file').value)).expanduser().resolve()
        self._edit_root = Path(
            str(self.get_parameter('edit_root').value)).expanduser().resolve()
        self._root.mkdir(parents=True, exist_ok=True)
        self._state_file.parent.mkdir(parents=True, exist_ok=True)
        self._edit_root.mkdir(parents=True, exist_ok=True)
        self._registry = MapRegistry(Path(str(self.get_parameter('registry_file').value)))

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(
            MapStatus, self.get_parameter('status_topic').value, qos)
        self._events_pub = self.create_publisher(
            String, self.get_parameter('events_topic').value, 10)
        self.create_subscription(
            String, self.get_parameter('events_topic').value, self._on_map_event, 10)

        self._packages: List[PackageInfo] = []
        self._active: Optional[PackageInfo] = None
        self._generation = 0
        self._reason = 'no_active_map'

        self.create_service(
            ListMapPackages,
            self.get_parameter('list_service').value,
            self._on_list,
        )
        self.create_service(
            LoadMapPackage,
            self.get_parameter('load_service').value,
            self._on_load,
        )
        self.create_service(
            StartMapEdit,
            self.get_parameter('start_edit_service').value,
            self._on_start_edit,
        )
        self.create_service(
            PublishMapEdit,
            self.get_parameter('publish_edit_service').value,
            self._on_publish_edit,
        )
        self.create_service(
            CancelMapEdit,
            self.get_parameter('cancel_edit_service').value,
            self._on_cancel_edit,
        )
        self.create_service(ValidateMap, self.get_parameter('validate_service').value, self._on_validate)
        self.create_service(ActivateMap, self.get_parameter('activate_service').value, self._on_activate)
        self.create_service(DiscardMap, self.get_parameter('discard_service').value, self._on_discard)
        self._generate_action = ActionServer(
            self,
            GenerateMapPackage,
            self.get_parameter('generate_action').value,
            execute_callback=self._on_generate,
        )

        self._refresh()
        self._restore_active_state()
        self._sync_registry()
        self._publish_status()
        self.get_logger().info(
            f'Map Manager root={self._root}; edit_root={self._edit_root}; '
            f'discovered {len(self._packages)} metadata files')

    def _refresh(self) -> None:
        verify = bool(self.get_parameter('verify_hashes_on_discovery').value)
        self._packages = list(discover_packages(self._root, verify_hashes=verify))

    def _sync_registry(self) -> None:
        active = None
        if self._active is not None:
            active = self._active.key
        self._registry.sync(self._packages, active=active)

    def _record(self, info: PackageInfo):
        return self._registry.get(info.map_id, info.map_version)

    def _to_package_msg(self, info: PackageInfo) -> MapPackage:
        out = MapPackage()
        out.map_id = info.map_id
        out.map_version = info.map_version
        out.package_path = str(info.package_path)
        out.valid = bool(info.valid)
        out.reason = info.reason
        out.navigation_map_yaml = info.asset_path('navigation_map')
        out.localization_map_pcd = info.asset_path('localization_map')
        out.relocalization_assets_path = info.asset_path('relocalization_assets')
        out.rtk_origin_yaml = info.asset_path('rtk_origin')
        record = self._record(info)
        if record is not None:
            out.status = record.status
            out.active = record.status == MapState.ACTIVE
            out.created_time = record.created_time
            out.package_hash = record.package_hash
        return out

    def _to_edit_session_msg(self, info: EditSessionInfo) -> MapEditSession:
        out = MapEditSession()
        out.stamp = self.get_clock().now().to_msg()
        out.active = info.state == 'open'
        out.session_id = info.session_id
        out.state = info.state
        out.base_map_id = info.base_map_id
        out.base_map_version = info.base_map_version
        out.session_path = str(info.session_path)
        out.navigation_map_yaml = str(info.navigation_map_yaml)
        out.geometry_fingerprint = info.geometry_fingerprint
        out.contract_fingerprint = info.contract_fingerprint
        out.reason = info.reason
        return out

    def _status_msg(self) -> MapStatus:
        out = MapStatus()
        out.stamp = self.get_clock().now().to_msg()
        out.active = self._active is not None
        out.generation = int(self._generation)
        out.reason = self._reason
        if self._active is not None:
            out.map_id = self._active.map_id
            out.map_version = self._active.map_version
            out.package_path = str(self._active.package_path)
            out.navigation_map_yaml = self._active.asset_path('navigation_map')
            out.localization_map_pcd = self._active.asset_path('localization_map')
            out.relocalization_assets_path = self._active.asset_path('relocalization_assets')
            out.rtk_origin_yaml = self._active.asset_path('rtk_origin')
        return out

    def _publish_event(self, event_type: str, **fields) -> None:
        msg = String()
        msg.data = encode_event(event_type, **fields)
        self._events_pub.publish(msg)

    def _find_info_or_raise(self, map_id: str, map_version: str) -> PackageInfo:
        self._refresh()
        self._sync_registry()
        info = self._find_exact(str(map_id).strip(), str(map_version).strip())
        if info is None:
            raise ValueError('exact_map_id_and_version_not_uniquely_found')
        return info

    def _validate_info(self, info: PackageInfo) -> dict:
        record = self._record(info)
        if record is not None and record.status in (MapState.CANDIDATE, MapState.FAILED):
            self._registry.transition(info.map_id, info.map_version, MapState.VALIDATING)
        report = validate_map_package(info.metadata_path)
        write_validation_report(info.metadata_path, report)
        current = self._registry.get(info.map_id, info.map_version)
        if current is not None:
            target = MapState.VALIDATED if report['result'] == 'PASS' else MapState.FAILED
            if current.status != target:
                self._registry.transition(info.map_id, info.map_version, target, report.get('reason', ''))
        return report

    def _on_validate(self, request, response):
        try:
            info = self._find_info_or_raise(request.map_id, request.map_version)
            report = self._validate_info(info)
            response.success = report['result'] == 'PASS'
            response.message = report.get('reason', '') or report['result']
            response.report = json.dumps(report, sort_keys=True)
            self._publish_event(MAP_VALIDATED, map_id=info.map_id,
                                map_version=info.map_version, result=report['result'])
        except (OSError, ValueError, TypeError) as exc:
            response.success = False
            response.message = str(exc)
            response.report = json.dumps({'result': 'FAIL', 'reason': str(exc)})
        return response

    def _on_activate(self, request, response):
        try:
            info = self._find_info_or_raise(request.map_id, request.map_version)
            report = self._validate_info(info)
            if report['result'] != 'PASS':
                raise ValueError('map_validation_failed')
            status = self._activate_candidate(info, 'validated_and_activated')
            self._registry.set_active(info.map_id, info.map_version)
            response.success = True
            response.message = 'validated map activated; runtime consumers must apply the new generation'
            response.status = status
            self._publish_event(MAP_ACTIVATED, map_id=info.map_id, version=info.map_version,
                                map_version=info.map_version, generation=status.generation)
        except (OSError, ValueError, TypeError) as exc:
            response.success = False
            response.message = str(exc)
            response.status = self._status_msg()
        return response

    def _on_discard(self, request, response):
        try:
            info = self._find_info_or_raise(request.map_id, request.map_version)
            record = self._registry.get(info.map_id, info.map_version)
            if record is None or record.status not in (MapState.CANDIDATE, MapState.FAILED):
                raise ValueError('only_candidate_or_failed_maps_can_be_discarded')
            if self._active is not None and info.key == self._active.key:
                raise ValueError('active_map_cannot_be_discarded')
            info.package_path.relative_to(self._root)
            shutil.rmtree(info.package_path)
            self._registry.records.pop(info.key, None)
            self._registry.save()
            self._refresh()
            response.success = True
            response.message = 'map package discarded'
        except (OSError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _on_map_event(self, msg: String) -> None:
        try:
            event = decode_event(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'Ignoring invalid map event: {exc}')
            return
        if event.get('type') != MAP_GENERATED:
            return
        try:
            map_id = str(event.get('map_id', '')).strip()
            version = str(event.get('map_version', event.get('version', ''))).strip()
            source = Path(str(event.get('source_pcd', ''))).expanduser()
            root = Path(str(event.get('path', event.get('package_path', '')))).expanduser()
            if source.is_file():
                package = build_package(
                    self._root, map_id, version, source,
                    Path(str(event['navigation_dir'])),
                    relocalization_assets_dir=Path(str(event['relocalization_assets_dir']))
                    if event.get('relocalization_assets_dir') else None)
            else:
                metadata = root / 'metadata.yaml' if root.is_dir() else root
                package = validate_package(metadata, verify_hashes=False)
                if not package.valid:
                    raise ValueError(f'generated_package_invalid:{package.reason}')
            self._refresh(); self._sync_registry()
            self.get_logger().info(f'Map generated candidate: {map_id}/{version}')
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(f'MAP_GENERATED processing failed: {exc}')

    def _publish_status(self) -> MapStatus:
        msg = self._status_msg()
        self._status_pub.publish(msg)
        return msg

    def _on_list(self, request, response):
        del request
        self._refresh()
        self._sync_registry()
        response.packages = [self._to_package_msg(info) for info in self._packages]
        return response

    def _find_exact(self, map_id: str, map_version: str) -> Optional[PackageInfo]:
        matches = [
            package for package in self._packages
            if package.map_id == map_id and package.map_version == map_version
        ]
        if len(matches) != 1:
            return None
        return matches[0]

    def _activate_candidate(self, candidate: PackageInfo, reason: str) -> MapStatus:
        if not candidate.valid:
            raise ValueError(f'cannot_activate_invalid_map:{candidate.reason}')
        next_generation = self._generation + 1
        state = {
            'schema_version': 1,
            'generation': int(next_generation),
            'map_id': candidate.map_id,
            'map_version': candidate.map_version,
            'package_path': str(candidate.package_path),
            'metadata_path': str(candidate.metadata_path),
            'navigation_map_yaml': candidate.asset_path('navigation_map'),
            'localization_map_pcd': candidate.asset_path('localization_map'),
            'relocalization_assets_path': candidate.asset_path('relocalization_assets'),
            'rtk_origin_yaml': candidate.asset_path('rtk_origin'),
        }
        backup = self._state_file.with_name(self._state_file.name + '.bak')
        try:
            if self._state_file.is_file():
                shutil.copy2(self._state_file, backup)
            self._atomic_write_yaml(self._state_file, state)
        except Exception:
            if backup.is_file():
                os.replace(backup, self._state_file)
            raise
        self._active = candidate
        self._generation = next_generation
        self._reason = reason
        self._registry.set_active(candidate.map_id, candidate.map_version)
        status = self._publish_status()
        self.get_logger().info(
            f'Active map selected: {candidate.map_id}/{candidate.map_version} '
            f'generation={next_generation}')
        return status

    def _fully_validate_exact(self, map_id: str, map_version: str) -> PackageInfo:
        self._refresh()
        candidate = self._find_exact(map_id, map_version)
        if candidate is None:
            raise ValueError('exact_map_id_and_version_not_uniquely_found')
        if not candidate.valid:
            raise ValueError(f'map_package_invalid:{candidate.reason}')
        verify = bool(self.get_parameter('verify_hashes_on_load').value)
        candidate = validate_package(candidate.metadata_path, verify_hashes=verify)
        if not candidate.valid:
            raise ValueError(f'map_package_failed_load_validation:{candidate.reason}')
        return candidate

    def _on_load(self, request, response):
        map_id = str(request.map_id).strip()
        map_version = str(request.map_version).strip()
        if not map_id or not map_version:
            response.success = False
            response.message = (
                'map_id and map_version are both required; '
                'implicit latest-version selection is forbidden')
            response.status = self._status_msg()
            return response
        try:
            candidate = self._fully_validate_exact(map_id, map_version)
            report = self._validate_info(candidate)
            if report['result'] != 'PASS':
                raise ValueError('map_validation_failed')
            status = self._activate_candidate(candidate, 'selected_and_validated')
        except (OSError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
            response.status = self._status_msg()
            return response

        response.success = True
        response.message = (
            'map package selected atomically; runtime consumers must still apply '
            'the same generation before navigation is considered switched')
        response.status = status
        return response

    def _on_start_edit(self, request, response):
        map_id = str(request.map_id).strip()
        map_version = str(request.map_version).strip()
        if not map_id or not map_version:
            response.success = False
            response.message = 'map_id and map_version are required for an edit session'
            return response
        try:
            candidate = self._fully_validate_exact(map_id, map_version)
            session = create_edit_session(candidate, self._edit_root)
        except (OSError, ValueError, RuntimeError) as exc:
            response.success = False
            response.message = str(exc)
            return response

        response.success = True
        response.message = (
            'edit session created from an immutable package; HMI may edit only '
            'the session navigation occupancy/topology assets')
        response.session = self._to_edit_session_msg(session)
        self.get_logger().info(
            f'Created map edit session {session.session_id} from '
            f'{session.base_map_id}/{session.base_map_version}')
        return response

    def _on_publish_edit(self, request, response):
        session_id = str(request.session_id).strip()
        target_map_id = str(request.target_map_id).strip()
        target_map_version = str(request.target_map_version).strip()
        if not session_id or not target_map_id or not target_map_version:
            response.success = False
            response.message = 'session_id, target_map_id and target_map_version are required'
            response.status = self._status_msg()
            return response

        try:
            session = validate_edit_session_for_publish(self._edit_root, session_id)
            base = validate_package(session.base_metadata_path, verify_hashes=True)
            if not base.valid:
                raise ValueError(f'base_map_package_changed_or_invalid:{base.reason}')
            if (base.map_id, base.map_version) != (
                    session.base_map_id, session.base_map_version):
                raise ValueError('edit_session_base_map_identity_mismatch')

            self._refresh()
            if self._find_exact(target_map_id, target_map_version) is not None:
                raise ValueError('target_map_id_and_version_already_exists')

            destination = promote(
                base_metadata=base.metadata_path,
                edited_map_yaml=session.navigation_map_yaml,
                map_root=self._root,
                map_id=target_map_id,
                map_version=target_map_version,
                activate=False,
                active_state_file=self._state_file,
            )
            published = validate_package(
                destination / 'metadata.yaml', verify_hashes=True)
            if not published.valid:
                raise RuntimeError(f'published_map_failed_validation:{published.reason}')

            # Once the immutable package exists the session is published even if
            # the optional activation step later fails.  This prevents retries
            # from attempting to overwrite/recreate an already published version.
            set_edit_session_state(
                self._edit_root,
                session_id,
                'published',
                f'published_as:{published.map_id}/{published.map_version}',
            )
            self._refresh()
            self._sync_registry()
        except (OSError, ValueError, RuntimeError) as exc:
            response.success = False
            response.message = str(exc)
            response.status = self._status_msg()
            return response

        response.package = self._to_package_msg(published)
        if bool(request.activate):
            try:
                report = self._validate_info(published)
                if report['result'] != 'PASS':
                    raise ValueError('map_validation_failed')
                status = self._activate_candidate(
                    published, 'published_edit_selected_and_validated')
            except (OSError, ValueError) as exc:
                response.success = False
                response.message = (
                    'map package was published successfully but activation failed; '
                    f'use /agt/map/load to retry activation: {exc}')
                response.status = self._status_msg()
                return response
        else:
            status = self._status_msg()

        response.success = True
        response.message = (
            'HMI edit published as a new immutable Map Package; '
            + ('new package selected as active map' if bool(request.activate)
               else 'active map left unchanged'))
        response.status = status
        self.get_logger().info(
            f'Published edit session {session_id} as '
            f'{published.map_id}/{published.map_version} activate={bool(request.activate)}')
        return response

    def _on_cancel_edit(self, request, response):
        session_id = str(request.session_id).strip()
        if not session_id:
            response.success = False
            response.message = 'session_id is required'
            return response
        try:
            current = load_edit_session(self._edit_root, session_id)
            if current.state != 'open':
                raise ValueError(f'edit_session_not_open:{current.state}')
            cancelled = set_edit_session_state(
                self._edit_root, session_id, 'cancelled', 'operator_cancelled')
        except (OSError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
            return response

        response.success = True
        response.message = 'edit session cancelled; staging files retained for audit'
        response.session = self._to_edit_session_msg(cancelled)
        return response

    @staticmethod
    def _optional_path(value: str) -> Optional[Path]:
        value = str(value).strip()
        return Path(value) if value else None

    def _on_generate(self, goal_handle):
        request = goal_handle.request
        result = GenerateMapPackage.Result()
        feedback = GenerateMapPackage.Feedback()
        feedback.phase = 'generate_navigation_assets'
        goal_handle.publish_feedback(feedback)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = 'map generation cancelled before publication'
            return result
        try:
            destination = generate_map_package(
                map_root=self._root,
                map_id=str(request.map_id),
                map_version=str(request.map_version),
                source_pcd=Path(str(request.source_pcd)),
                pipeline_config=Path(str(request.pipeline_config)),
                relocalization_assets_dir=self._optional_path(request.relocalization_assets_dir),
                trajectory_poses=self._optional_path(request.trajectory_poses),
                rtk_origin=self._optional_path(request.rtk_origin),
                preview=self._optional_path(request.preview),
            )
            feedback.phase = 'validate_published_package'
            goal_handle.publish_feedback(feedback)
            package = validate_package(destination / 'metadata.yaml', verify_hashes=True)
            if not package.valid:
                raise RuntimeError(f'published_map_failed_validation:{package.reason}')
        except (OSError, RuntimeError, ValueError) as exc:
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            return result

        goal_handle.succeed()
        result.success = True
        result.message = 'generated and published immutable Map Package; awaiting validation/activation'
        self._refresh()
        self._sync_registry()
        result.package = self._to_package_msg(package)
        self._publish_event(MAP_GENERATED, map_id=package.map_id,
                            map_version=package.map_version,
                            package_path=str(package.package_path))
        self.get_logger().info(
            f'Generated map package {package.map_id}/{package.map_version} via action')
        return result

    @staticmethod
    def _atomic_write_yaml(path: Path, data: Dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp_name = tempfile.mkstemp(
            prefix=f'.{path.name}.', suffix='.tmp', dir=str(path.parent))
        tmp_path = Path(tmp_name)
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                yaml.safe_dump(data, stream, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(tmp_path, path)
            try:
                directory_fd = os.open(
                    str(path.parent), os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
            except OSError:
                # Replacement is already atomic on the target filesystem;
                # directory fsync is best-effort for crash durability.
                pass
        finally:
            if tmp_path.exists():
                tmp_path.unlink()

    def _restore_active_state(self) -> None:
        if not self._state_file.is_file():
            return
        try:
            data = yaml.safe_load(self._state_file.read_text(encoding='utf-8')) or {}
            map_id = str(data.get('map_id', '')).strip()
            map_version = str(data.get('map_version', '')).strip()
            generation = int(data.get('generation', 0))
        except (OSError, UnicodeError, ValueError, TypeError, yaml.YAMLError) as exc:
            self._reason = f'active_state_invalid:{exc}'
            self.get_logger().error(self._reason)
            return

        candidate = self._find_exact(map_id, map_version)
        if candidate is None or not candidate.valid:
            self._reason = 'persisted_active_map_no_longer_valid_or_unique'
            self.get_logger().error(self._reason)
            return

        candidate = validate_package(
            candidate.metadata_path,
            verify_hashes=bool(self.get_parameter('verify_hashes_on_load').value),
        )
        if not candidate.valid:
            self._reason = f'persisted_active_map_failed_validation:{candidate.reason}'
            self.get_logger().error(self._reason)
            return

        self._active = candidate
        self._generation = max(generation, 1)
        self._reason = 'restored_and_revalidated'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
