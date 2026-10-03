"""Record a new experiment session with immutable frozen-input provenance."""
import argparse
from datetime import datetime,timezone
from pathlib import Path
import subprocess
import signal
from .manifest import verify,atomic_yaml,digest

TOPICS=['/livox/lidar','/livox/imu','/agt/livox/points','/agt/odometry/local',
        '/agt/odometry/adapter_status','/agt/odometry/quality','/tf','/tf_static','/clock',
        '/agt/local_row/state','/agt/local_row/path','/agt/local_row/visibility',
        '/agt/localization/candidate','/agt/localization/global_observation',
        '/agt/localization/quality','/agt/localization/recovery_status','/agt/localization/status',
        '/agt/localization/metrics','/agt/global_relocalization/status',
        '/agt/research/task_context','/agt/research/navigation_mode','/plan','/local_plan',
        '/agt/research/cmd_vel_source','/agt/research/cmd_vel_smoothed','/agt/base/cmd_vel',
        '/agt/base/odom','/agt/base/state','/wheel/odom','/mux/cmd_vel','/bunker_status',
        '/agt/cmd_vel_guard/state','/agt/payload/drive_permission','/mission/state',
        '/ins/navsatfix','/ins/status','/diagnostics',
        '/follow_path/_action/status','/navigation/execute_task_segment/_action/status']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--experiment',required=True);parser.add_argument('--run-id',required=True)
    parser.add_argument('--dataset-role',required=True,choices=['D_R0','D_R1','D_R2','D_R3'])
    parser.add_argument('--route-id',required=True);parser.add_argument('--group',required=True)
    parser.add_argument('--reference-source',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--extra-topic',action='append',default=[])
    args=parser.parse_args()
    try:
        lock=verify(args.experiment)
        for name in ('run_id','route_id','group','reference_source'):
            if not getattr(args,name).strip(): raise ValueError(name+' required')
        output=Path(args.output).expanduser().resolve()
        if output.exists(): raise FileExistsError('new session output directory required')
        for repo in lock['repositories']:
            if output.is_relative_to(Path(repo['path'])):
                raise ValueError('recorded sessions must be outside pinned software repositories')
        for record in lock['assets'].values():
            root=Path(record['path'])
            if root.is_dir() and output.is_relative_to(root):
                raise ValueError('session output may not modify a frozen asset tree')
        topics=list(dict.fromkeys(TOPICS+args.extra_topic))
        if any(not x.startswith('/') for x in topics): raise ValueError('absolute topic names required')
        output.mkdir(parents=True,exist_ok=False)
        provenance={'schema_version':1,'started_utc':datetime.now(timezone.utc).isoformat(),
                    'experiment':digest(args.experiment),'experiment_bundle_hash':lock['bundle_hash'],
                    'map_id':lock['map_id'],'map_version':lock['map_version'],'map_hash':lock['map_hash'],
                    'robot_profile':lock['robot_profile'],'run_id':args.run_id,'dataset_role':args.dataset_role,
                    'route_id':args.route_id,'group':args.group,'reference_source':args.reference_source,
                    'topics':topics,'point_timing':'preserved raw Livox plus separate PointCloud2',
                    'outcome':'not_inferred; annotate every attempt including failures'}
        atomic_yaml(output/'session_manifest.yaml',provenance)
        child=subprocess.Popen(['ros2','bag','record','--output',str(output/'bag'),*topics])
        try: code=child.wait()
        except KeyboardInterrupt:
            child.send_signal(signal.SIGINT)
            try: code=child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                child.terminate();code=child.wait()
        atomic_yaml(output/'recording_result.yaml',{'finished_utc':datetime.now(timezone.utc).isoformat(),
                   'recorder_returncode':code,'bag_exists':(output/'bag'/'metadata.yaml').is_file(),
                   'mission_success':'not_inferred'})
    except (ValueError,OSError,KeyError,subprocess.SubprocessError) as exc: parser.exit(2,str(exc)+'\n')
