from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, name, arguments):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / name)),
        launch_arguments=arguments.items())


def generate_launch_description():
    map_pcd = LaunchConfiguration('global_map')
    bag_lidar = LaunchConfiguration('lidar_topic')
    batch_config = LaunchConfiguration('batch_config')
    description_share = Path(get_package_share_directory('tracked_chassis_description'))
    runtime_share = Path(get_package_share_directory('agt_navigation_runtime'))

    return LaunchDescription([
        DeclareLaunchArgument('global_map', description='Path to global_map.pcd'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('auto_relocalize', default_value='true'),
        DeclareLaunchArgument(
            'batch_config',
            default_value=str(runtime_share / 'config' / 'batch_lio_mid360.yaml'),
            description='Canonical Batch-LIO runtime config shared with adapter/relocalization.'),
        DeclareLaunchArgument(
            'robot_description_calibration_file',
            default_value=str(description_share / 'config' / 'field_acceptance.yaml'),
            description='Physical chassis/sensor calibration used by tracked_chassis_description.'),
        DeclareLaunchArgument(
            'relocalization_query_frame_mode', default_value='mapping_body',
            description='Formal PGO map mode mapping_body; base_link is deprecated compatibility.'),
        DeclareLaunchArgument(
            'bbs_query_frame_mode', default_value='mapping_body',
            description='Formal PGO candidate-BBS mode mapping_body; base_link is deprecated compatibility.'),
        DeclareLaunchArgument(
            'relocalization_executable', default_value='global_relocalization',
            description='global_relocalization or manual_seed_relocalization.'),
        DeclareLaunchArgument('relocalization_assets', default_value=''),
        DeclareLaunchArgument('lidar_topic', default_value='/agt/sensors/lidar/custom'),
        DeclareLaunchArgument('imu_topic', default_value='/agt/sensors/imu/data'),
        DeclareLaunchArgument('gicp_constraint_mode', default_value='full_se3'),
        DeclareLaunchArgument('max_roll_delta_deg', default_value='10.0'),
        DeclareLaunchArgument('max_pitch_delta_deg', default_value='10.0'),
        DeclareLaunchArgument('cloud_contract_capture_dir', default_value=''),

        # A single static map publisher. This node has no TF authority.
        Node(
            package='pcl_ros', executable='pcd_to_pointcloud', name='offline_global_map',
            output='screen', arguments=[],
            remappings=[('cloud_pcd', '/agt/map/points')],
            parameters=[{
                'file_name': map_pcd,
                'tf_frame': 'map',
                'publish_rate': 0.0,
                'use_sim_time': True,
            }],
        ),

        # Offline replay must use the same physical TF authority as the vehicle.
        # Do not duplicate base_link->lidar_link constants in this launch.
        include('tracked_chassis_description', 'display.launch.py', {
            'calibration_file': LaunchConfiguration('robot_description_calibration_file'),
            'start_rviz': 'false',
        }),

        include('agt_navigation_runtime', 'navigation_lio.launch.py', {
            'use_sim_time': 'true',
            'launch_batch_rviz': 'false',
            'batch_config': batch_config,
            'lidar_topic': bag_lidar,
            'imu_topic': LaunchConfiguration('imu_topic'),
        }),
        Node(
            package='agt_livox_tools',
            executable='livox_format_bridge',
            name='offline_query_builder',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'mode': 'custom_to_pointcloud2',
                'input_topic': bag_lidar,
                'output_topic': '/agt/relocalization/input_cloud',
            }],
        ),
        include('agt_global_relocalization', 'global_relocalization.launch.py', {
            'use_sim_time': 'true',
            'auto_request': LaunchConfiguration('auto_relocalize'),
            'relocalization_executable': LaunchConfiguration('relocalization_executable'),
            'global_map': map_pcd,
            'scan_topic': '/agt/relocalization/input_cloud',
            'follow_map_manager': 'false',
            'batch_lio_config_file': batch_config,
            'relocalization_query_frame_mode': LaunchConfiguration(
                'relocalization_query_frame_mode'),
            'bbs_query_frame_mode': LaunchConfiguration('bbs_query_frame_mode'),
            'relocalization_assets': LaunchConfiguration('relocalization_assets'),
            'gicp_constraint_mode': LaunchConfiguration('gicp_constraint_mode'),
            'max_roll_delta_deg': LaunchConfiguration('max_roll_delta_deg'),
            'max_pitch_delta_deg': LaunchConfiguration('max_pitch_delta_deg'),
            'cloud_contract_capture_dir': LaunchConfiguration('cloud_contract_capture_dir'),
        }),
        include('agt_localization_manager', 'localization_manager.launch.py', {
            'use_sim_time': 'true',
            'debug_identity_map_odom': 'false',
        }),
        Node(
            package='rviz2',
            executable='rviz2',
            name='offline_relocalization_rviz',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            arguments=['-d', str(
                Path(get_package_share_directory('agt_system_bringup'))
                / 'config' / 'offline_relocalization.rviz')],
            parameters=[{'use_sim_time': True}],
        ),
    ])
