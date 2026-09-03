from __future__ import annotations

import os
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from rclpy.durability import DurabilityPolicy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from agt_robot_interfaces.msg import MapPackage, MapStatus
from agt_robot_interfaces.srv import ListMapPackages, LoadMapPackage

from .map_package import PackageInfo, discover_packages, validate_package


class MapManager(Node):
    """Discover, validate and select one versioned Map Package.

    V1 deliberately stops at an atomic *selection* boundary. It does not pretend
    that writing active_map.yaml is the same thing as atomically reloading Nav2
    and the 3D localization backend. Runtime two-phase apply/rollback is the next
    layer and will consume the typed MapStatus produced here.
    """

    def __init__(self) -> None:
        super().__init__('agt_map_manager')
        self.declare_parameter('map_root', '~/.ros/agt_maps')
        self.declare_parameter(
            'active_state_file', '~/.ros/agt_navigation_v3/active_map.yaml')
        self.declare_parameter('verify_hashes_on_discovery', False)
        self.declare_parameter('verify_hashes_on_load', True)
        self.declare_parameter('status_topic', '/agt/map/status')
        self.declare_parameter('list_service', '/agt/map/list')
        self.declare_parameter('load_service', '/agt/map/load')

        self._root = Path(str(self.get_parameter('map_root').value)).expanduser().resolve()
        self._state_file = Path(
            str(self.get_parameter('active_state_file').value)).expanduser().resolve()
        self._root.mkdir(parents=True, exist_ok=True)
        self._state_file.parent.mkdir(parents=True, exist_ok=True)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(
            MapStatus, self.get_parameter('status_topic').value, qos)

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

        self._refresh()
        self._restore_active_state()
        self._publish_status()
        self.get_logger().info(
            f'Map Manager V1 root={self._root}; discovered {len(self._packages)} metadata files')

    def _refresh(self) -> None:
        verify = bool(self.get_parameter('verify_hashes_on_discovery').value)
        self._packages = list(discover_packages(self._root, verify_hashes=verify))

    @staticmethod
    def _to_package_msg(info: PackageInfo) -> MapPackage:
        out = MapPackage()
        out.map_id = info.map_id
        out.map_version = info.map_version
        out.package_path = str(info.package_path)
        out.valid = bool(info.valid)
        out.reason = info.reason
        out.navigation_map_yaml = info.asset_path('navigation_map')
        out.localization_map_pcd = info.asset_path('localization_map')
        out.rtk_origin_yaml = info.asset_path('rtk_origin')
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
            out.rtk_origin_yaml = self._active.asset_path('rtk_origin')
        return out

    def _publish_status(self) -> MapStatus:
        msg = self._status_msg()
        self._status_pub.publish(msg)
        return msg

    def _on_list(self, request, response):
        del request
        self._refresh()
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

    def _on_load(self, request, response):
        map_id = str(request.map_id).strip()
        map_version = str(request.map_version).strip()
        if not map_id or not map_version:
            response.success = False
            response.message = 'map_id and map_version are both required; implicit latest-version selection is forbidden'
            response.status = self._status_msg()
            return response

        self._refresh()
        candidate = self._find_exact(map_id, map_version)
        if candidate is None:
            response.success = False
            response.message = 'exact map_id/map_version not uniquely found'
            response.status = self._status_msg()
            return response
        if not candidate.valid:
            response.success = False
            response.message = f'map package is invalid: {candidate.reason}'
            response.status = self._status_msg()
            return response

        # Discovery may skip large-file hashing for responsiveness. Loading is
        # the integrity boundary and can revalidate all declared hashes.
        verify = bool(self.get_parameter('verify_hashes_on_load').value)
        candidate = validate_package(candidate.metadata_path, verify_hashes=verify)
        if not candidate.valid:
            response.success = False
            response.message = f'map package failed load-time validation: {candidate.reason}'
            response.status = self._status_msg()
            return response

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
            'rtk_origin_yaml': candidate.asset_path('rtk_origin'),
        }
        try:
            self._atomic_write_yaml(self._state_file, state)
        except OSError as exc:
            response.success = False
            response.message = f'failed to persist active map atomically: {exc}'
            response.status = self._status_msg()
            return response

        self._active = candidate
        self._generation = next_generation
        self._reason = 'selected_and_validated'
        status = self._publish_status()
        response.success = True
        response.message = (
            'map package selected atomically; runtime consumers must still apply '
            'the same generation before navigation is considered switched')
        response.status = status
        self.get_logger().info(
            f'Active map selected: {candidate.map_id}/{candidate.map_version} generation={next_generation}')
        return response

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
                directory_fd = os.open(str(path.parent), os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
            except OSError:
                # File replacement is already atomic on the target filesystem;
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

        # Restore is deliberately revalidated. A stale state file may not make a
        # modified/corrupt map active after reboot.
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
