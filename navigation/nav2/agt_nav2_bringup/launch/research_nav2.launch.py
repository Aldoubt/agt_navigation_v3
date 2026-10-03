"""Research FollowPath stack with one epoch controller and tagged smoother."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _nodes(context):
    value=lambda name: LaunchConfiguration(name).perform(context)
    path=Path(value('params_file')).expanduser()
    if not path.is_file(): raise RuntimeError('explicit frozen research Nav2 params_file required')
    config=yaml.safe_load(path.read_text())
    controller=config['controller_server']['ros__parameters']
    if controller.get('controller_plugins')!=['ResearchFollowPath'] or controller['ResearchFollowPath'].get('plugin')!='agt_epoch_rpp_controller/EpochRppController':
        raise RuntimeError('research Nav2 requires the sole ResearchFollowPath epoch controller')
    local=config['local_costmap']['local_costmap']['ros__parameters']
    if local.get('global_frame')!='odom' or local.get('rolling_window') is not True or 'voxel_layer' not in local.get('plugins',[]):
        raise RuntimeError('research Nav2 requires rolling odom costmap and live obstacle observations')
    sim=value('use_sim_time').lower()=='true'
    nodes=[]
    names=['controller_server','planner_server','smoother_server','map_server']
    for package,name in [('nav2_controller','controller_server'),('nav2_planner','planner_server'),
                         ('nav2_smoother','smoother_server'),('nav2_map_server','map_server')]:
        overrides={'use_sim_time':sim}
        if name=='map_server': overrides['yaml_filename']=value('map')
        nodes.append(Node(package=package,executable=name,name=name,output='screen',
                          parameters=[str(path),overrides],
                          remappings=[('cmd_vel','/agt/research/unused_nav2_twist'),
                                      ('odom','/agt/odometry/local')]))
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
                      name='lifecycle_manager_research',output='screen',parameters=[{
                          'use_sim_time':sim,'autostart':True,'node_names':names}]))
    # No bt_navigator/behavior command producer is created in this profile.
    # The typed smoother is composed by greenhouse_research.launch.py.
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map'),DeclareLaunchArgument('params_file'),
                              DeclareLaunchArgument('use_sim_time',default_value='false'),
                              OpaqueFunction(function=_nodes)])
