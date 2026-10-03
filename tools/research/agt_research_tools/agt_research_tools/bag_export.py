"""Export raw timed LiDAR and pose CSVs plus two-clock research records."""
from __future__ import annotations
import argparse
import bisect
import csv
import json
import math
from pathlib import Path

from .manifest import atomic_yaml, digest


def ns(stamp):
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def records(bag, storage, wanted):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag), storage_id=storage), ConverterOptions('cdr', 'cdr'))
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    classes = {}
    while reader.has_next():
        topic, raw, bag_time = reader.read_next()
        if topic not in wanted:
            continue
        if topic not in classes:
            classes[topic]=get_message(types[topic])
        yield topic, int(bag_time), deserialize_message(raw, classes[topic])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag'); parser.add_argument('--output', required=True)
    parser.add_argument('--storage', default='sqlite3')
    parser.add_argument('--cloud-topic', default='/agt/livox/points')
    parser.add_argument('--odom-topic', default='/agt/odometry/local')
    parser.add_argument('--epoch-topic', default='/agt/odometry/quality')
    parser.add_argument('--dataset-role', choices=['D_H', 'D_R0', 'D_R1', 'D_R2', 'D_R3'], required=True)
    parser.add_argument('--base-frame', required=True)
    parser.add_argument('--odom-frame', default='odom')
    args = parser.parse_args()
    from sensor_msgs_py import point_cloud2
    from rosidl_runtime_py.convert import message_to_ordereddict
    output = Path(args.output).resolve()
    if output.exists():
        parser.error('output must be a new directory; existing exports are immutable')
    output.mkdir(parents=True); (output/'clouds').mkdir()
    epochs, poses = [], []
    fallback_epoch, last_stamp = 1, 0
    for topic, bag_time, message in records(args.bag, args.storage, {args.odom_topic, args.epoch_topic}):
        if topic == args.epoch_topic:
            epochs.append((bag_time, int(message.odom_epoch)))
            continue
        stamp = ns(message.header.stamp)
        if (stamp <= 0 or message.header.frame_id != args.odom_frame or
                message.child_frame_id != args.base_frame):
            raise ValueError('pose source timestamp or frame contract invalid')
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):
            raise ValueError('nonfinite pose')
        if last_stamp and stamp < last_stamp:
            fallback_epoch += 1
        last_stamp = stamp
        poses.append((bag_time, stamp, fallback_epoch, (p.x,p.y,p.z,q.x,q.y,q.z,q.w)))
    if not poses:
        parser.error('no valid pose input; obtain a short-term trajectory or use geometry-only A0 input')
    epochs.sort()
    epoch_times = [x[0] for x in epochs]
    def epoch_at(bag_time, fallback):
        index = bisect.bisect_right(epoch_times, bag_time)-1
        return epochs[index][1] if index >= 0 else fallback
    pose_ownership = [(p[0], epoch_at(p[0],p[2])) for p in poses]
    pose_times = [p[0] for p in pose_ownership]
    def cloud_epoch(bag_time):
        index = bisect.bisect_right(pose_times, bag_time)-1
        return epoch_at(bag_time, pose_ownership[max(0,index)][1])
    with (output/'poses.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['stamp','x','y','z','qx','qy','qz','qw','odom_epoch','bag_time_ns','header_time_ns'])
        for bag_time, stamp, fallback, pose in sorted(poses,key=lambda x:(epoch_at(x[0],x[2]),x[1])):
            writer.writerow([f'{stamp/1e9:.9f}',*pose,epoch_at(bag_time,fallback),bag_time,stamp])
    diagnostics = {'/agt/local_row/state','/agt/local_row/visibility','/agt/research/navigation_mode',
                   '/agt/localization/quality','/agt/localization/candidate',args.epoch_topic,
                   '/wheel/odom','/agt/base/odom','/agt/research/cmd_vel_source',
                   '/agt/research/cmd_vel_smoothed','/agt/base/cmd_vel','/mux/cmd_vel',
                   '/agt/localization/status','/agt/task/state'}
    diagnostics.update({'/agt/localization/global_observation','/agt/localization/recovery_status',
                        '/agt/research/task_context','/agt/base/state','/mission/state',
                        '/agt/local_row/path','/plan','/local_plan','/agt/cmd_vel_guard/state'})
    count, missing_timing = 0, 0
    with (output/'scans.csv').open('w',newline='') as scans, (output/'records.jsonl').open('w') as log:
        sw=csv.writer(scans)
        sw.writerow(['stamp','cloud_path','sensor_frame','odom_epoch','bag_time_ns','header_time_ns','timebase_ns'])
        for topic,bag_time,message in records(args.bag,args.storage,diagnostics|{args.cloud_topic}):
            header=getattr(message,'header',None)
            header_stamp=ns(header.stamp) if header is not None else 0
            if topic != args.cloud_topic:
                log.write(json.dumps({'topic':topic,'bag_time_ns':bag_time,'header_time_ns':header_stamp,
                                      'message':message_to_ordereddict(message)},allow_nan=True)+'\n')
                continue
            base_stamp=int(message.timebase) if hasattr(message,'timebase') else header_stamp
            if base_stamp<=0: raise ValueError('positive cloud source time required')
            if base_stamp <= 0 or not message.header.frame_id:
                raise ValueError('cloud time/frame contract invalid')
            path=output/'clouds'/f'{count:08d}.csv'
            with path.open('w',newline='') as stream:
                cw=csv.writer(stream)
                cw.writerow(['x','y','z','timestamp','offset_time','intensity','tag','line'])
                if hasattr(message,'points'):
                    for p in message.points:
                        cw.writerow([p.x,p.y,p.z,f'{(base_stamp+int(p.offset_time))/1e9:.9f}',
                                     p.offset_time,p.reflectivity,p.tag,p.line])
                else:
                    fields={field.name for field in message.fields}
                    if not {'timestamp','offset_time'} & fields:
                        missing_timing += 1
                        raise ValueError('cloud lacks per-point timing; temporal export refused')
                    names=[name for name in ['x','y','z','timestamp','offset_time','intensity','tag','line'] if name in fields]
                    for point in point_cloud2.read_points(message,field_names=names,skip_nans=False):
                        p=dict(zip(names,point))
                        if not all(math.isfinite(float(p[k])) for k in ('x','y','z')):
                            continue
                        offset=int(p['offset_time']) if 'offset_time' in p else round((float(p['timestamp'])-base_stamp/1e9)*1e9)
                        if offset < 0:
                            raise ValueError('negative point offset')
                        absolute=float(p['timestamp']) if 'timestamp' in p else (base_stamp+offset)/1e9
                        if not math.isfinite(absolute) or absolute<=0 or abs(absolute-(base_stamp+offset)/1e9)>1e-6:
                            raise ValueError('inconsistent or invalid source point timing')
                        cw.writerow([p['x'],p['y'],p['z'],f'{absolute:.9f}',offset,
                                     p.get('intensity',0),p.get('tag',0),p.get('line',0)])
            sw.writerow([f'{base_stamp/1e9:.9f}',path.relative_to(output).as_posix(),
                         message.header.frame_id,cloud_epoch(bag_time),bag_time,header_stamp,base_stamp])
            count += 1
    if not count:
        raise ValueError('cloud topic absent or empty')
    atomic_yaml(output/'manifest.yaml', {'schema_version':1,'dataset_role':args.dataset_role,
                'bag':digest(args.bag),'cloud_topic':args.cloud_topic,'odom_topic':args.odom_topic,
                'base_frame':args.base_frame,'odom_frame':args.odom_frame,'scans':count,
                'timing_missing':missing_timing,'point_time_unit':'seconds absolute / offset nanoseconds',
                'epoch_source':args.epoch_topic if epochs else 'inferred_source_regression',
                'pose_order':'epoch then header stamp','clock_fields':['bag_time_ns','header_time_ns','timebase_ns'],
                'outputs':{name:digest(output/name) for name in ('scans.csv','poses.csv','records.jsonl','clouds')}})
    print(json.dumps({'output':str(output),'scans':count,'dataset_role':args.dataset_role}))
