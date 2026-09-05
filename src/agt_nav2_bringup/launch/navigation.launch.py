from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_files(context):
    checks = {
        'map': LaunchConfiguration('map').perform(context),
        'nav2_params_file': LaunchConfiguration('nav2_params_file').perform(context),
    }
    for label, value in checks.items():
        path = Path(value).expanduser()
        if not value or not path.is_file():
            raise RuntimeError(f'agt_nav2_bringup: {label} file does not exist: {value!r}')
    return []


def _include_navigation(context):
    """Resolve wrapper arguments before including upstream Nav2.

    Humble launch scoping can otherwise lose the wrapper's default params_file
    when this launch is itself included by a higher-level field launch.
    """
    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    launch_arguments = {
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
        'autostart': LaunchConfiguration('autostart').perform(context),
        'params_file': LaunchConfiguration('nav2_params_file').perform(context),
        'use_composition': 'False',
    }
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'navigation_launch.py')),
            launch_arguments=launch_arguments.items(),
        )
    ]


def generate_launch_description():
    agt_share = Path(get_package_share_directory('agt_nav2_bringup'))

    map_yaml = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to the derived Nav2 map YAML'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=str(agt_share / 'config' / 'nav2_params.yaml'),
            description='AGT Nav2 parameter file. Unique name avoids nested launch collisions.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=_validate_files),

        # Localization is external. AMCL/localization_launch.py is intentionally
        # not started; Localization Manager owns map->odom and Batch-LIO owns the
        # continuous local odometry input.
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params_file, {'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['map_server'],
            }],
        ),
        OpaqueFunction(function=_include_navigation),
    ])
