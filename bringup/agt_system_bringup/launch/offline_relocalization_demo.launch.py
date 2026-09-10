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
    return LaunchDescription([
        DeclareLaunchArgument('global_map', description='Path to global_map.pcd'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('relocalization_assets', default_value=''),
        DeclareLaunchArgument('lidar_topic', default_value='/agt/sensors/lidar/custom'),
        DeclareLaunchArgument('imu_topic', default_value='/agt/sensors/imu/data'),
        # A single static map publisher. This node has no TF authority.
        Node(package='pcl_ros', executable='pcd_to_pointcloud', name='offline_global_map',
             output='screen', arguments=[],
             remappings=[('cloud_pcd', '/agt/map/points')],
             parameters=[{'file_name': map_pcd, 'tf_frame': 'map',
                          'publish_rate': 0.0, 'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='offline_base_to_lidar',
             arguments=['0.2615', '0', '0.407', '0.00313477', '0.200873', '-0.0006428', 'base_link', 'lidar_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='offline_lidar_to_livox',
             arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'livox_frame']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='offline_lidar_to_imu',
             arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'imu_link']),
        include('agt_mapping_bringup', 'navigation_lio.launch.py', {
            'use_sim_time': 'true', 'enable_pgo': 'false', 'launch_batch_rviz': 'false',
            'lidar_topic': bag_lidar, 'imu_topic': LaunchConfiguration('imu_topic')}),
        Node(package='agt_livox_tools', executable='livox_format_bridge', name='offline_query_builder',
             output='screen', parameters=[{
                 'use_sim_time': True, 'mode': 'custom_to_pointcloud2',
                 'input_topic': bag_lidar, 'output_topic': '/agt/relocalization/input_cloud'}]),
        include('agt_global_relocalization', 'global_relocalization.launch.py', {
            'use_sim_time': 'true', 'auto_request': 'true',
            'global_map': map_pcd, 'scan_topic': '/agt/relocalization/input_cloud',
            'follow_map_manager': 'false',
            'relocalization_assets': LaunchConfiguration('relocalization_assets')}),
        include('agt_localization_manager', 'localization_manager.launch.py',
                {'use_sim_time': 'true', 'debug_identity_map_odom': 'false'}),
        Node(package='rviz2', executable='rviz2', name='offline_relocalization_rviz',
             output='screen', condition=IfCondition(LaunchConfiguration('launch_rviz')), arguments=['-d', str(Path(get_package_share_directory(
                 'agt_system_bringup')) / 'config' / 'offline_relocalization.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
