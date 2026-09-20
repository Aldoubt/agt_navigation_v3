"""Nav2, the sole obstacle-cloud node and the guarded command chain."""

from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


CONFIG_FILES = (
    'robot.yaml', 'navigation.yaml', 'controller.yaml',
    'costmap.yaml', 'perception.yaml', 'safety.yaml',
)


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


def _build_runtime_params(config_dir):
    merged = {}
    for name in CONFIG_FILES:
        with (config_dir / name).open(encoding='utf-8') as stream:
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
    follow = controller['FollowPath']
    follow.update({
        'desired_linear_vel': limits['controller_cruise_mps'],
        'min_approach_linear_velocity': limits['controller_approach_mps'],
        'regulated_linear_scaling_min_speed': limits['controller_regulated_min_mps'],
        'rotate_to_heading_angular_vel': limits['rotate_to_heading_radps'],
        'max_angular_accel': limits['controller_bootstrap_angular_accel'],
    })

    behavior = _params(merged, 'behavior_server')
    behavior.update({
        'max_rotational_vel': limits['rotate_to_heading_radps'],
        'min_rotational_vel': limits['controller_regulated_min_mps'],
        'rotational_acc_lim': limits['angular_accel_radps2'],
    })
    smoother = _params(merged, 'velocity_smoother')
    smoother.update({
        'max_velocity': [limits['forward_mps'], 0.0, limits['angular_radps']],
        'min_velocity': [-limits['reverse_mps'], 0.0, -limits['angular_radps']],
        'max_accel': [limits['linear_accel_mps2'], 0.0, limits['angular_accel_radps2']],
        'max_decel': [-limits['linear_decel_mps2'], 0.0, -limits['angular_decel_radps2']],
    })
    guard = _params(merged, 'agt_cmd_vel_guard')
    guard.update({
        'max_linear_x': limits['forward_mps'],
        'max_reverse_x': limits['reverse_mps'],
        'max_angular_z': limits['angular_radps'],
        'max_linear_accel': limits['linear_accel_mps2'],
        'max_linear_decel': limits['linear_decel_mps2'],
        'max_angular_accel': limits['angular_accel_radps2'],
    })

    output = Path(tempfile.mkdtemp(prefix='agt_navigation_params_')) / 'runtime.yaml'
    output.write_text(yaml.safe_dump(merged, sort_keys=False), encoding='utf-8')
    return str(output)


def _launch_runtime(context):
    map_path = Path(LaunchConfiguration('map').perform(context)).expanduser()
    if not map_path.is_file():
        raise RuntimeError(f'map must be an existing Nav2 map YAML: {str(map_path)!r}')

    share = Path(get_package_share_directory('agt_system_bringup'))
    nav2_share = Path(get_package_share_directory('agt_nav2_bringup'))
    runtime_share = Path(get_package_share_directory('agt_navigation_runtime'))
    patrol_share = Path(get_package_share_directory('agt_rviz_patrol'))
    params_file = _build_runtime_params(share / 'config')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    return [
        # Exactly one producer of /agt/navigation/points_obstacles.
        Node(
            package='agt_pointcloud_preprocessor',
            executable='agt_pointcloud_preprocessor',
            name='agt_pointcloud_preprocessor',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(nav2_share / 'launch' / 'navigation.launch.py')),
            launch_arguments={
                'map': str(map_path),
                'nav2_params_file': params_file,
                'use_sim_time': use_sim_time,
                'autostart': LaunchConfiguration('autostart').perform(context),
            }.items(),
        ),
        Node(
            package='agt_base_control', executable='cmd_vel_guard',
            name='agt_cmd_vel_guard', output='screen', parameters=[params_file]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(runtime_share / 'launch' / 'runtime.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(patrol_share / 'launch' / 'rviz_patrol.launch.py')),
            launch_arguments={
                'map_id': LaunchConfiguration('map_id').perform(context),
                'mission_dir': LaunchConfiguration('mission_dir').perform(context),
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute Nav2 map YAML path'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('map_id', default_value='field_navigation'),
        DeclareLaunchArgument('mission_dir', default_value='~/.ros/agt_rviz_patrol'),
        OpaqueFunction(function=_launch_runtime),
    ])
