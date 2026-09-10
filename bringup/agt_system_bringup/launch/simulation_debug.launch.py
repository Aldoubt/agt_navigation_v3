"""R1.1 Gazebo-only Nav2 runtime; it intentionally excludes all real localization."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / 'launch' / launch_file)),
        launch_arguments=(arguments or {}).items(),
    )


def _validate_mode(context):
    mode = LaunchConfiguration('localization_mode').perform(context)
    if mode not in ('identity', 'amcl'):
        raise RuntimeError("localization_mode must be 'identity' or 'amcl'")
    return []


def _mode_is(name):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration('localization_mode'), "' == '", name, "'",
    ]))


def generate_launch_description():
    share = Path(get_package_share_directory('agt_system_bringup'))
    demo_share = Path(get_package_share_directory('agt_demo_task'))
    default_map = (
        '/home/yangxuan/ros2_ws/agt_data/maps/'
        'bunker_mid360_mapping_20260901_205036/v003-indexed/navigation/map.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('navigation_map', default_value=default_map),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'localization_mode', default_value='identity',
            description='identity: controlled Gazebo control test; amcl: requires a /scan source.'),
        DeclareLaunchArgument('enable_camera', default_value='false'),
        DeclareLaunchArgument('enable_demo_task', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        OpaqueFunction(function=_validate_mode),

        _include('agt_gazebo_sim', 'sim_world.launch.py', {
            'gui': LaunchConfiguration('gui'),
        }),
        _include('agt_sim_odometry_adapter', 'sim_odometry_adapter.launch.py', {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }),

        # Identity is restricted to a controlled Gazebo world whose map and
        # world origins have been verified. It is not used by field launches.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='agt_sim_identity_map_to_odom', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            condition=_mode_is('identity'),
        ),
        # AMCL is an opt-in standard Nav2 mode. The existing Gazebo model does
        # not publish /scan, so this is intentionally not the R1.1 default.
        Node(
            package='nav2_amcl', executable='amcl', name='amcl', output='screen',
            parameters=[str(share / 'config' / 'simulation_amcl.yaml'), {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            condition=_mode_is('amcl'),
        ),
        _include('agt_nav2_bringup', 'navigation.launch.py', {
            'map': LaunchConfiguration('navigation_map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }),
        _include('agt_base_control', 'cmd_vel_guard.launch.py'),
        Node(
            package='rviz2', executable='rviz2', name='agt_sim_navigation_rviz',
            output='screen', arguments=['-d', str(demo_share / 'config' / 'navigation_debug.rviz')],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        Node(
            package='agt_capability_camera', executable='camera_capture_service',
            name='camera_capture_service', output='screen',
            condition=IfCondition(LaunchConfiguration('enable_camera')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        Node(
            package='agt_demo_task', executable='navigate_capture_task',
            name='navigate_capture_task', output='screen',
            condition=IfCondition(LaunchConfiguration('enable_demo_task')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
