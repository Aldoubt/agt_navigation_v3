"""Opt-in greenhouse observations or standalone frozen-map research control."""
from pathlib import Path
import math
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from agt_research_tools.manifest import verify, digest


def _share(package): return Path(get_package_share_directory(package))


def _params(path,name):
    tree=yaml.safe_load(Path(path).read_text())
    result=tree.get(name,{}).get('ros__parameters')
    if not isinstance(result,dict): raise RuntimeError(f'{path}: missing {name}.ros__parameters')
    return result


def _pinned(lock,path,asset):
    path=Path(path).expanduser().resolve(strict=True)
    record=lock['assets'][asset]
    root=Path(record['path'])
    if root.is_file():
        match=path==root
    else:
        relative=path.relative_to(root).as_posix() if path.is_relative_to(root) else None
        match=relative is not None and (relative=='.' or relative in {x['path'] for x in record['files']} or
              (path.is_dir() and any(x['path'].startswith(relative+'/') for x in record['files'])))
    if not match: raise RuntimeError(f'{path} must be included in frozen assets.{asset}')


def _include(package,name,args):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(_share(package)/'launch'/name)),
                                    launch_arguments=args.items())


def module_payload_interlock(robot_config_spec,robot):
    import sys
    sys.path.insert(0,str(_share('agt_robot_bringup')/'tools'))
    from robot_config import payload_interlock_required
    return payload_interlock_required(robot_config_spec,robot)


def _topics(config,expected):
    for key,topic in expected.items():
        if config.get(key,topic)!=topic:
            raise RuntimeError(f'research command ownership requires {key}={topic}')


def _launch(context):
    value=lambda name: LaunchConfiguration(name).perform(context)
    motion=value('enable_motion').lower()=='true'
    start_loc=value('start_localization').lower()=='true'
    sim=value('use_sim_time').lower()=='true'
    paths={name:value(name) for name in ('row_config','geometry_file','odom_quality_config','policy_config',
           'guard_config','smoother_config','capability_config','global_config','manager_config','tracker_config')}
    for name,path in paths.items():
        if not Path(path).expanduser().is_file(): raise RuntimeError(f'{name} file missing: {path}')
    robot=value('robot')
    geometry=yaml.safe_load(Path(paths['geometry_file']).read_text())['geometry']
    lock=verify(value('experiment_manifest')) if value('experiment_manifest') else None
    if motion:
        if lock is None: raise RuntimeError('motion requires a verified frozen experiment_manifest')
        if robot!=lock['robot_profile']: raise RuntimeError('robot differs from frozen experiment identity')
        if not start_loc: raise RuntimeError('research motion requires standalone research localization ownership')
        if not (Path(lock['assets']['relocalization_assets']['path'])/'polar_context.db').is_file():
            raise RuntimeError('research motion requires candidate assets with spatial ambiguity evidence (polar_context.db)')
        _pinned(lock,paths['geometry_file'],'sensor_geometry')
        for name,path in paths.items():
            if name!='geometry_file': _pinned(lock,path,'parameters')
        for name,flag in [('odom_quality_config','field_verified'),('policy_config','field_verified'),
                          ('guard_config','research_field_verified'),('smoother_config','field_verified'),
                          ('capability_config','research_field_verified'),('global_config','moving_query_calibrated'),
                          ('manager_config','research_recovery_calibrated'),('tracker_config','quality_calibrated')]:
            node={'odom_quality_config':'agt_odom_quality','policy_config':'agt_task_continuity',
                  'guard_config':'agt_cmd_vel_guard','smoother_config':'agt_epoch_velocity_smoother',
                  'capability_config':'agt_navigation_capability','global_config':'agt_global_relocalization',
                  'manager_config':'agt_localization_manager','tracker_config':'agt_map_tracker'}[name]
            if _params(paths[name],node).get(flag) is not True: raise RuntimeError(name+' requires measured '+flag)
        if geometry.get('field_verified') is not True: raise RuntimeError('verified footprint/self-geometry/coverage required')
        geometry_record=yaml.safe_load(Path(paths['geometry_file']).read_text())
        if geometry_record.get('robot_profile')!=robot:
            raise RuntimeError('verified sensor geometry must identify the selected robot_profile')
        for name in ('nav_config_dir','base_adapter_config','base_state_config'):
            if not value(name): raise RuntimeError(name+' is required for motion')
            _pinned(lock,value(name),'parameters')
        adapter=_params(value('base_adapter_config'),'agt_tracked_twist_adapter')
        state=yaml.safe_load(Path(value('base_state_config')).read_text()).get('base_state',{})
        if adapter.get('field_verified') is not True or adapter.get('robot_profile')!=robot:
            raise RuntimeError('verified profile-specific tracked driver boundary required')
        if state.get('field_verified') is not True or state.get('robot_profile')!=robot:
            raise RuntimeError('verified profile-specific actual base state mapping required')
        # Geometry, uncertainty bounds and stopping envelope use one calibrated
        # value at every stage. A disagreement is a launch error.
        policy=_params(paths['policy_config'],'agt_task_continuity')
        guard=_params(paths['guard_config'],'agt_cmd_vel_guard')
        pairs=[(geometry.get('braking_deceleration_mps2'),policy.get('braking_decel_mps2'),guard.get('research_braking_decel_mps2')),
               (geometry.get('command_latency_sec'),policy.get('stopping_latency_sec'),guard.get('research_stop_latency_sec')),
               (geometry.get('safety_margin_m'),policy.get('clearance_margin_m'),guard.get('research_clearance_margin_m'))]
        if any(not all(isinstance(v,(int,float)) and v>0 for v in group) or
               max(group)-min(group)>1e-9 for group in pairs):
            raise RuntimeError('row, mode and guard require identical positive calibrated braking/latency/margin values')
        owners=(policy.get('owner_id','agt_task_continuity'),guard.get('research_owner_id','agt_task_continuity'),
                _params(paths['smoother_config'],'agt_epoch_velocity_smoother').get('owner_id','agt_task_continuity'))
        if set(owners)!={'agt_task_continuity'}: raise RuntimeError('research command chain owner mismatch')
        tracker_params=_params(paths['tracker_config'],'agt_map_tracker')
        rate=tracker_params.get('tracking_rate_hz',0.5)
        result_age=tracker_params.get('max_result_age_sec',2.0)
        global_ttl=_params(paths['manager_config'],'agt_localization_manager').get('global_quality_timeout_sec',3.0)
        if (not all(isinstance(x,(int,float)) and math.isfinite(x) and x>0 for x in (rate,result_age,global_ttl)) or
                global_ttl<=1.0/rate+result_age):
            raise RuntimeError('global quality TTL must exceed tracker period plus its allowed result latency')
        capability_params=_params(paths['capability_config'],'agt_navigation_capability')
        if (policy.get('global_quality_timeout_sec',3.0)!=global_ttl or
                capability_params.get('research_global_quality_timeout_sec',3.0)!=global_ttl or
                capability_params.get('research_recovery_timeout_sec',3.0)<global_ttl):
            raise RuntimeError('manager, policy and capability require matching calibrated global quality windows')
        _topics(policy,{'task_topic':'/agt/research/task_context','mode_topic':'/agt/research/navigation_mode',
                        'global_quality_topic':'/agt/localization/quality','row_topic':'/agt/local_row/state',
                        'odom_quality_topic':'/agt/odometry/quality','odom_topic':'/agt/odometry/local',
                        'recovery_status_topic':'/agt/localization/recovery_status',
                        'base_state_topic':'/agt/base/state'})
        _topics(_params(paths['capability_config'],'agt_navigation_capability'),{
            'research_task_topic':'/agt/research/task_context','research_mode_topic':'/agt/research/navigation_mode',
            'research_row_path_topic':'/agt/local_row/path','research_global_quality_topic':'/agt/localization/quality',
            'research_odom_quality_topic':'/agt/odometry/quality','research_recovery_topic':'/agt/localization/recovery_status',
            'research_base_state_topic':'/agt/base/state','research_localization_topic':'/agt/localization/status'})
        _topics(_params(paths['smoother_config'],'agt_epoch_velocity_smoother'),{
            'input_topic':'/agt/research/cmd_vel_source','output_topic':'/agt/research/cmd_vel_smoothed',
            'mode_topic':'/agt/research/navigation_mode'})
        _topics(guard,{'output_topic':'/agt/base/cmd_vel','research_command_topic':'/agt/research/cmd_vel_smoothed',
                      'research_mode_topic':'/agt/research/navigation_mode','research_row_topic':'/agt/local_row/state',
                      'research_odom_topic':'/agt/odometry/quality','base_state_topic':'/agt/base/state'})
        _topics(adapter,{'input_topic':'/agt/base/cmd_vel','base_state_topic':'/agt/base/state'})
        _topics(state,{'output_topic':'/agt/base/state'})
        if adapter.get('driver_command_topic') in ('/agt/base/cmd_vel','/cmd_vel','/cmd_vel_nav',
                   '/agt/research/unused_nav2_twist','/agt/research/cmd_vel_source','/agt/research/cmd_vel_smoothed'):
            raise RuntimeError('adapter requires a dedicated downstream driver endpoint')
        require_payload=module_payload_interlock(value('robot_config'),robot)
        if require_payload and any(cfg.get('require_payload_drive_permission') is not True for cfg in
                    (policy,guard,_params(paths['capability_config'],'agt_navigation_capability'))):
            raise RuntimeError('installed payload requires its drive-permission interlock in every research stage')
    common={'use_sim_time':sim}
    nodes=[Node(package='agt_local_row_perception',executable='local_row_perception',name='agt_local_row_perception',
                output='screen',parameters=[common,{'config_file':paths['row_config'],
                    'geometry_file':paths['geometry_file'],'field_verified':motion}]),
           Node(package='agt_navigation_supervisor',executable='odom_quality',name='agt_odom_quality',
                output='screen',parameters=[paths['odom_quality_config'],common]),
           Node(package='agt_task_continuity',executable='task_continuity',name='agt_task_continuity',
                output='screen',parameters=[paths['policy_config'],common,{'research_enabled':motion}])]
    if start_loc and lock is None: raise RuntimeError('standalone research localization requires frozen experiment_manifest')
    if lock:
        identity={'map_id':lock['map_id'],'map_version':lock['map_version'],'map_hash':lock['map_hash']}
        pcd=lock['assets']['geometry_map']['path']
        tracker={**identity,**common,'global_map':pcd,'shadow_mode':True,'apply_correction':False,
                 'scan_topic':'/agt/livox/points_self_filtered','quality_topic':'/agt/localization/global_observation'}
        if not motion: tracker['quality_calibrated']=False
        nodes.append(Node(package='agt_map_tracker',executable='map_tracker',name='agt_map_tracker',
                          output='screen',parameters=[paths['tracker_config'],tracker]))
        if start_loc:
            calibration=value('lio_calibration_file')
            if not calibration: raise RuntimeError('lio_calibration_file required to keep body/base transform consistent')
            _pinned(lock,calibration,'calibration')
            if value('start_lio').lower()=='true':
                args={'lio_backend':value('lio_backend'),'body_to_base_calibration_file':calibration,
                      'fastlio_config':calibration,'batch_config':calibration,'use_sim_time':str(sim).lower()}
                nodes.append(_include('agt_navigation_runtime','lio.launch.py',args))
                nodes.append(_include('agt_livox_tools','livox_format_bridge.launch.py',{}))
            nodes.extend([
                Node(package='agt_global_relocalization',executable='global_relocalization',name='agt_global_relocalization',
                     output='screen',parameters=[paths['global_config'],common,identity,{
                         'global_map':pcd,'relocalization_assets':lock['assets']['relocalization_assets']['path'],
                         'scan_topic':'/agt/livox/points_self_filtered','follow_map_manager':False,
                         'body_to_base_calibration_file':calibration,'moving_query_enabled':motion,
                         'moving_query_calibrated':motion,'require_stationary':not motion,'publish_legacy_pose':False}]),
                Node(package='agt_localization_manager',executable='localization_manager',name='agt_localization_manager',
                     output='screen',parameters=[paths['manager_config'],common,identity,{
                         'research_recovery_enabled':True,'research_recovery_calibrated':motion}])])
    if motion:
        import importlib.util
        spec=importlib.util.spec_from_file_location('agt_research_nav_config',_share('agt_system_bringup')/'launch/navigation.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        if robot=='bunker_v1':
            directory=Path(value('nav_config_dir')).expanduser().resolve(strict=True)
            profile=module._robot_profile_path(robot)
            baseline=module._read_yaml(_share('agt_system_bringup')/'config/navigation_profiles/bunker_v1.yaml','Bunker baseline')
            module._validate_tracked_profile(directory,profile,baseline)
        else:
            directory=module.select_navigation_config(robot,value('nav_config_dir'),_share('agt_system_bringup')/'config')
            _pinned(lock,Path(directory)/'navigation_profile.yaml','parameters')
        # Verify every input YAML before composing one immutable runtime view.
        for filename in module.CONFIG_FILES: _pinned(lock,Path(directory)/filename,'parameters')
        generated=module._build_runtime_params(directory,local_obstacle_avoidance=True)
        tree=yaml.safe_load(Path(generated).read_text())
        robot_params=tree['agt_robot_config']['ros__parameters']
        if not module._same_numbers(geometry.get('footprint'),yaml.safe_load(robot_params['footprint'])):
            raise RuntimeError('perception footprint must equal the frozen Nav2 footprint')
        if geometry.get('frame_id')!='base_link' or tree['local_costmap']['local_costmap']['ros__parameters'].get('robot_base_frame')!='base_link':
            raise RuntimeError('research task and perception use the measured canonical base_link frame')
        controller=tree['controller_server']['ros__parameters']
        plugin=controller.pop('FollowPath')
        plugin.update(plugin='agt_epoch_rpp_controller/EpochRppController',research_enabled=True,
                      field_verified=True,owner_id='agt_task_continuity',use_rotate_to_heading=False,
                      allow_reversing=False)
        controller['controller_plugins']=['ResearchFollowPath'];controller['ResearchFollowPath']=plugin
        controller['general_goal_checker']['xy_goal_tolerance']=_params(paths['capability_config'],'agt_navigation_capability')['research_goal_tolerance_m']
        controller['general_goal_checker']['yaw_goal_tolerance']=_params(paths['capability_config'],'agt_navigation_capability')['research_goal_yaw_tolerance_rad']
        controller['general_goal_checker']['stateful']=False
        limits=tree['agt_motion_limits']['ros__parameters']
        if (adapter['max_forward_mps']>limits['forward_mps'] or adapter['max_angular_rps']>limits['angular_radps'] or
                guard['max_linear_x']>limits['forward_mps'] or guard['max_angular_z']>limits['angular_radps']):
            raise RuntimeError('research adapter/guard caps exceed the selected vehicle limits')
        tree['agt_pointcloud_preprocessor']['ros__parameters']['input_topic']='/agt/livox/points_self_filtered'
        tree['agt_pointcloud_preprocessor']['ros__parameters']['self_filter']['enabled']=False
        Path(generated).write_text(yaml.safe_dump(tree,sort_keys=False))
        map_yaml=value('navigation_map') or lock['navigation_map_yaml']
        if str(Path(map_yaml).expanduser().resolve())!=lock['navigation_map_yaml']:
            raise RuntimeError('research Nav2 map differs from frozen selected navigation_map_yaml')
        _pinned(lock,map_yaml,'navigation_map')
        nodes.extend([
            _include('agt_pointcloud_preprocessor','local_perception.launch.py',{'params_file':generated,'use_sim_time':str(sim).lower()}),
            _include('agt_nav2_bringup','research_nav2.launch.py',{'params_file':generated,'map':map_yaml,'use_sim_time':str(sim).lower()}),
            Node(package='agt_navigation_capability',executable='navigation_capability',name='agt_navigation_capability',
                 output='screen',parameters=[paths['capability_config'],common,{'robot_profile':robot,**identity,
                     'enable_research_task_continuity':True,'research_field_verified':True}]),
            Node(package='agt_base_runtime',executable='epoch_velocity_smoother',name='agt_epoch_velocity_smoother',
                 output='screen',parameters=[paths['smoother_config'],common,{'research_enabled':True,'field_verified':True}]),
            Node(package='agt_base_runtime',executable='motion_guard',name='agt_cmd_vel_guard',output='screen',
                 parameters=[paths['guard_config'],common,{'research_enabled':True,'expected_robot_profile':robot}]),
            Node(package='agt_base_runtime',executable='driver_state_normalizer.py',name='agt_driver_state_normalizer',
                 output='screen',parameters=[common,{'config_file':value('base_state_config')}]),
            Node(package='agt_base_runtime',executable='tracked_twist_adapter',name='agt_tracked_twist_adapter',
                 output='screen',parameters=[value('base_adapter_config'),common])])
    return nodes


def generate_launch_description():
    own=_share('agt_system_bringup')/'config/research'
    defaults={'enable_motion':'false','start_localization':'false','start_lio':'false','use_sim_time':'false',
              'robot':'bunker_v1','robot_config':'','experiment_manifest':'','lio_backend':'fastlio2','lio_calibration_file':'',
              'navigation_map':'','nav_config_dir':'','base_adapter_config':'','base_state_config':'',
              'row_config':str(_share('agt_local_row_perception')/'config/row_perception.yaml'),
              'geometry_file':str(_share('agt_local_row_perception')/'config/geometry_diagnostic.yaml'),
              'policy_config':str(_share('agt_task_continuity')/'config/task_continuity.yaml'),
              'global_config':str(_share('agt_global_relocalization')/'config/global_relocalization.yaml'),
              'manager_config':str(_share('agt_localization_manager')/'config/localization_manager.yaml'),
              'tracker_config':str(_share('agt_map_tracker')/'config/map_tracker.yaml')}
    for name in ('odom_quality','guard','smoother','capability'): defaults[name+'_config']=str(own/(name+'.yaml'))
    return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],
                              OpaqueFunction(function=_launch)])
