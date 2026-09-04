import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, condition=None, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', launch_file)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def _validate_required_files(context):
    required = {
        'map': LaunchConfiguration('map').perform(context),
        'global_map': LaunchConfiguration('global_map').perform(context),
    }
    optional_assets = LaunchConfiguration('relocalization_assets').perform(context)
    for label, value in required.items():
        path = os.path.expanduser(value)
        if not value or not os.path.isfile(path):
            raise RuntimeError(f'agt rviz field demo: {label} file does not exist: {value!r}')
    if optional_assets:
        path = os.path.expanduser(optional_assets)
        if not os.path.isdir(path):
            raise RuntimeError(
                f'agt rviz field demo: relocalization_assets directory does not exist: {optional_assets!r}')
    return []


def generate_launch_description():
    map_file = LaunchConfiguration('map')
    global_map = LaunchConfiguration('global_map')
    relocalization_assets = LaunchConfiguration('relocalization_assets')
    map_id = LaunchConfiguration('map_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_rtk = LaunchConfiguration('enable_rtk')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    default_rviz = os.path.join(
        get_package_share_directory('agt_rviz_patrol'), 'config', 'agt_rviz_demo.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to converted Nav2 map.yaml'),
        DeclareLaunchArgument('global_map', description='Absolute path to the matching final global_map.pcd'),
        DeclareLaunchArgument(
            'relocalization_assets', default_value='',
            description='Optional output directory from build_relocalization_assets.'),
        DeclareLaunchArgument('map_id', default_value='rviz_field_demo'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_rtk', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        OpaqueFunction(function=_validate_required_files),

        # Hardware prerequisites intentionally stay outside this launch:
        #   livox_ros_driver2, Bunker CAN driver, robot_state_publisher/URDF,
        #   and Autolabor-C1 capability. This launch is the AGT software chain.

        # Navigation odometry: Batch-LIO + frame adapter.
        include('agt_mapping_bringup', 'navigation_lio.launch.py'),

        # Secondary PointCloud2 branch for local obstacles and global relocalization.
        # The timing-preserving CustomMsg stays directly connected to Batch-LIO.
        include('agt_livox_tools', 'livox_format_bridge.launch.py'),
        include('agt_pointcloud_preprocessor', 'obstacle_cloud.launch.py'),

        # Manual-trigger global localization. Explicit paths are used for this
        # acceptance stage so Map Manager/hot-switching is not a prerequisite.
        include('agt_global_relocalization', 'global_relocalization.launch.py', arguments={
            'global_map': global_map,
            'relocalization_assets': relocalization_assets,
            'follow_map_manager': 'false',
        }),
        include('agt_localization_manager', 'localization_manager.launch.py'),

        # RTK remains record/diagnostic only and may be disabled without changing localization.
        include('agt_rtk_manager', 'rtk_manager.launch.py', condition=IfCondition(enable_rtk)),

        # Nav2 + 50 Hz base guard + fixed inspection task runtime.
        include('agt_nav2_bringup', 'navigation.launch.py', arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
        }),
        include('agt_base_control', 'cmd_vel_guard.launch.py'),
        include('agt_navigation_runtime', 'runtime.launch.py'),
        include('agt_rviz_patrol', 'rviz_patrol.launch.py', arguments={
            'map_id': map_id,
        }),

        Node(
            package='rviz2',
            executable='rviz2',
            name='agt_field_demo_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(launch_rviz),
        ),
    ])
