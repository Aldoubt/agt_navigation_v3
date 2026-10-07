"""Profile-specific composition of existing V3 nodes; one map->odom owner."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from agt_field.bundle import validate_bundle
from agt_field.profile import load_profile, require_real
from agt_map_manager.runtime_binding import resolve_active_map
from agt_batch_lio_adapter.extrinsics import load_lio_body_to_lidar
from agt_navigation_runtime.lio_config import freeze_fastlio_config, require_same_calibration


def nodes(context):
    profile = load_profile(LaunchConfiguration("profile").perform(context))
    require_real(profile, motion=True)
    active = Path(LaunchConfiguration("active_state_file").perform(context))
    binding = resolve_active_map(active)
    package = binding.package
    validate_bundle(package.package_path)
    calibration = profile["root"] / profile["localization"]["fastlio_config"]
    mapping_calibration = package.package_path / "mapping/calibration.yaml"
    require_same_calibration(calibration, mapping_calibration)
    frozen = freeze_fastlio_config(
        calibration, profile["topics"]["lidar"], profile["topics"]["imu"]
    )
    # The global query must use mapping-era internal calibration, not V3's old Bunker defaults.
    t, q = load_lio_body_to_lidar(mapping_calibration)

    def share(name):
        return Path(get_package_share_directory(name))

    adapter_file = share("agt_fastlio_adapter") / "config/fastlio_navigation_adapter.yaml"
    global_file = share("agt_global_relocalization") / "config/global_relocalization.yaml"
    tracker_file = share("agt_map_tracker") / "config/map_tracker.yaml"
    manager_file = share("agt_localization_manager") / "config/localization_manager.yaml"
    base = profile["robot"]["base_frame"]
    lidar = profile["robot"]["lidar_frame"]
    return [
        Node(
            package="fastlio2",
            executable="lio_node",
            namespace="fastlio2",
            name="lio_node",
            parameters=[{"config_path": frozen, "use_sim_time": False}],
            output="screen",
        ),
        Node(
            package="agt_fastlio_adapter",
            executable="fastlio_adapter",
            name="agt_fastlio_adapter",
            parameters=[
                str(adapter_file),
                {
                    "body_to_base_calibration_file": frozen,
                    "mount_lidar_frame": lidar,
                    "mount_base_frame": base,
                    "tf_child_frame": profile["robot"]["rotation_frame"],
                },
            ],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share("agt_livox_tools") / "launch/livox_format_bridge.launch.py")
            )
        ),
        Node(
            package="agt_global_relocalization",
            executable="global_relocalization",
            name="agt_global_relocalization",
            parameters=[
                str(global_file),
                {
                    "global_map": package.asset_path("localization_map"),
                    "relocalization_assets": package.asset_path("relocalization_assets"),
                    "follow_map_manager": False,
                    "auto_request": True,
                    "body_to_base_calibration_file": frozen,
                    "mount_lidar_frame": lidar,
                    "mount_base_frame": base,
                    "mapping_body_livox_translation": list(t),
                    "mapping_body_livox_quaternion_xyzw": list(q),
                    "relocalization_query_frame_mode": "mapping_body",
                    "bbs_query_frame_mode": "mapping_body",
                },
            ],
            output="screen",
        ),
        Node(
            package="agt_localization_manager",
            executable="localization_manager",
            name="agt_localization_manager",
            parameters=[
                str(manager_file),
                {
                    "map_id": package.map_id,
                    "map_version": package.map_version,
                    "debug_identity_map_odom": False,
                },
            ],
            output="screen",
        ),
        Node(
            package="agt_map_tracker",
            executable="map_tracker",
            name="agt_map_tracker",
            parameters=[
                str(tracker_file),
                {
                    "global_map": package.asset_path("localization_map"),
                    "apply_correction": True,
                    "debug_directory": str(active.parent.parent / "logs/map_tracker"),
                },
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("profile"),
            DeclareLaunchArgument("active_state_file"),
            OpaqueFunction(function=nodes),
        ]
    )
