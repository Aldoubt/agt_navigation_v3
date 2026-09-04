from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, launch_file, condition=None, arguments=None):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / launch_file)),
        condition=condition,
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    enable_pgo = LaunchConfiguration('enable_pgo')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('enable_pgo', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Mapping front-end. Keep the selected robotics-laboratory/fast-lio2
        # package unmodified; sensor topics/extrinsics remain in its own config.
        include('fastlio2', 'lio_launch.py', arguments={'use_sim_time': use_sim_time}),

        # Loop closure + GTSAM pose graph. Save the final map through
        # /pgo/save_maps with save_patches=true when HBA refinement is planned.
        include('pgo', 'pgo_launch.py', condition=IfCondition(enable_pgo),
                arguments={'use_sim_time': use_sim_time}),
    ])
