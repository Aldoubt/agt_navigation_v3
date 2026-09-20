from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _deep_merge(target, source):
    for key, value in source.items():
        if key in target and isinstance(target[key], dict) and isinstance(value, dict):
            _deep_merge(target[key], value)
        else:
            target[key] = value


def _params(tree, *node_path):
    node = tree
    for key in node_path:
        node = node[key]
    return node['ros__parameters']


def _canonical_params():
    share = Path(get_package_share_directory('agt_nav2_bringup'))
    merged = {}
    for name in (
        'robot.yaml', 'navigation.yaml', 'controller.yaml',
        'costmap.yaml', 'perception.yaml', 'safety.yaml',
    ):
        with (share / 'config' / name).open(encoding='utf-8') as stream:
            _deep_merge(merged, yaml.safe_load(stream) or {})

    robot = _params(merged, 'agt_robot_config')
    for costmap in ('local_costmap', 'global_costmap'):
        params = _params(merged, costmap, costmap)
        params['footprint'] = robot['footprint']
        params['footprint_padding'] = robot['footprint_padding']

    limits = _params(merged, 'agt_motion_limits')
    controller = _params(merged, 'controller_server')
    controller.update({
        'min_x_velocity_threshold': 0.01,
        'min_y_velocity_threshold': 0.0,
        'min_theta_velocity_threshold': 0.01,
    })
    controller['FollowPath'].update({
        'desired_linear_vel': limits['controller_cruise_mps'],
        'min_approach_linear_velocity': limits['controller_approach_mps'],
        'regulated_linear_scaling_min_speed': limits['controller_regulated_min_mps'],
        'rotate_to_heading_angular_vel': limits['rotate_to_heading_radps'],
        'max_angular_accel': limits['controller_bootstrap_angular_accel'],
    })
    _params(merged, 'behavior_server').update({
        'max_rotational_vel': limits['rotate_to_heading_radps'],
        'min_rotational_vel': limits['controller_regulated_min_mps'],
        'rotational_acc_lim': limits['angular_accel_radps2'],
    })
    _params(merged, 'velocity_smoother').update({
        'max_velocity': [limits['forward_mps'], 0.0, limits['angular_radps']],
        'min_velocity': [-limits['reverse_mps'], 0.0, -limits['angular_radps']],
        'max_accel': [limits['linear_accel_mps2'], 0.0, limits['angular_accel_radps2']],
        'max_decel': [-limits['linear_decel_mps2'], 0.0, -limits['angular_decel_radps2']],
    })
    output = Path(tempfile.mkdtemp(prefix='agt_nav2_params_')) / 'runtime.yaml'
    output.write_text(yaml.safe_dump(merged, sort_keys=False), encoding='utf-8')
    return str(output)


def _profile_params(context):
    explicit = LaunchConfiguration('nav2_params_file').perform(context)
    return explicit if explicit else _canonical_params()


def _validate_files(context):
    checks = {
        'map': LaunchConfiguration('map').perform(context),
        'nav2_params_file': _profile_params(context),
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
        'params_file': _profile_params(context),
        'use_composition': 'False',
    }
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'navigation_launch.py')),
            launch_arguments=launch_arguments.items(),
        )
    ]


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to the derived Nav2 map YAML'),
        DeclareLaunchArgument(
            'nav2_params_file', default_value='',
            description='Optional generated/experimental override; empty uses the six canonical files.',
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
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
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
