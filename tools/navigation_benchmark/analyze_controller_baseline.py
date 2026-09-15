#!/usr/bin/env python3
"""Offline NAV-C1 bag analyzer. Missing signals remain explicit, never inferred."""
import argparse, csv, math
from pathlib import Path
import yaml
from controller_metrics import absolute_stats, delta_stats, nearest_path_error, nearest_values, previous_age, signal_stats, zero_crossings

TOPICS = {'/agt/odometry/local':'odom','/cmd_vel':'raw','/cmd_vel_smoothed':'smoothed','/mux/cmd_vel':'final','/plan':'plan','/agt/navigation/points_obstacles':'cloud','/local_costmap/costmap':'costmap'}

def stamp(msg, fallback):
    header = getattr(msg, 'header', None); value = getattr(header, 'stamp', None)
    return value.sec + value.nanosec/1e9 if value and (value.sec or value.nanosec) else fallback/1e9

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('bag', type=Path); parser.add_argument('--output-dir', type=Path, required=True); args=parser.parse_args(); args.output_dir.mkdir(parents=True, exist_ok=True)
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader=SequentialReader(); reader.open(StorageOptions(uri=str(args.bag), storage_id='sqlite3'), ConverterOptions('cdr','cdr'))
    types={x.name:x.type for x in reader.get_all_topics_and_types()}; data={key:[] for key in TOPICS.values()}; path=[]
    while reader.has_next():
        topic, raw, t=reader.read_next()
        if topic not in TOPICS: continue
        msg=deserialize_message(raw, get_message(types[topic])); key=TOPICS[topic]; ts=stamp(msg,t)
        if key in ('raw','smoothed','final'): data[key].append((ts,msg.linear.x,msg.angular.z))
        elif key=='odom':
            q=msg.pose.pose.orientation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)); data[key].append((ts,msg.pose.pose.position.x,msg.pose.pose.position.y,yaw,msg.twist.twist.linear.x,msg.twist.twist.angular.z))
        elif key=='plan': path=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
        elif key=='cloud': data[key].append((ts,msg.width*msg.height))
        else: data[key].append((ts,1))
    required=['odom','raw','smoothed','final']; missing=[x for x in required if not data[x]]
    report={'bag':{'path':str(args.bag),'data_contract':'DATA_CONTRACT_INCOMPLETE' if missing or not path else 'COMPLETE','missing_required':missing+(['plan'] if not path else [])},'deadband_wz_radps':.02,'frequencies':{},'commands':{},'command_modification':{},'tracking':{},'chassis':{'status':'CHASSIS_EXECUTION_NOT_OBSERVABLE'}}
    for key in ('odom','raw','smoothed','final','cloud','costmap'):
        samples=data[key]; report['frequencies'][key]=signal_stats([(x[0],0) for x in samples]) if samples else {'status':'NOT_AVAILABLE'}
    for key in ('raw','smoothed','final'):
        vx=[(t,x) for t,x,_ in data[key]]; wz=[(t,w) for t,_,w in data[key]]; report['commands'][key]={'vx':signal_stats(vx),'wz':signal_stats(wz),'wz_zero_crossings':zero_crossings(wz)} if vx else {'status':'NOT_AVAILABLE'}
    for label,a,b in [('raw_to_smoothed','raw','smoothed'),('smoothed_to_final','smoothed','final')]:
        report['command_modification'][label]={'vx':delta_stats([(t,x) for t,x,_ in data[a]],[(t,x) for t,x,_ in data[b]]),'wz':delta_stats([(t,w) for t,_,w in data[a]],[(t,w) for t,_,w in data[b]])} if data[a] and data[b] else {'status':'NOT_AVAILABLE'}
    if data['odom'] and path:
        errors=[nearest_path_error(x,y,yaw,path) for _,x,y,yaw,_,_ in data['odom']]; report['tracking']={'cross_track':absolute_stats(list(zip([x[0] for x in data['odom']],[e[0] for e in errors]))),'heading_error':absolute_stats(list(zip([x[0] for x in data['odom']],[e[1] for e in errors])))}
    raw_times=[x[0] for x in data['raw']]; odom_times=[x[0] for x in data['odom']]; ages=previous_age(raw_times,odom_times) if raw_times and odom_times else []
    report['state_command_coupling']={'odom_age_at_control':signal_stats(list(zip(raw_times,ages))) if ages else {'status':'NOT_AVAILABLE'},'commands_per_odom_sample':len(raw_times)/len(odom_times) if odom_times else float('nan')}
    report['oscillation']={'raw_wz_sign_flip_timestamps':report['commands'].get('raw',{}).get('wz_zero_crossings',{}).get('timestamps',[]),'status':'EVENTS_RECORDED; inspect against odom/path timeline'}
    with open(args.output_dir/'controller_baseline_report.yaml','w') as f: yaml.safe_dump(report,f,sort_keys=False)
    with open(args.output_dir/'controller_timeline.csv','w',newline='') as f:
        w=csv.writer(f); w.writerow(['t','odom_x','odom_y','odom_yaw','odom_vx','odom_wz','odom_age_s','cross_track_error','heading_error','raw_vx','raw_wz','smoothed_vx','smoothed_wz','final_vx','final_wz','obstacle_point_count'])
        for t,x,y,yaw,vx,wz in data['odom']:
            cross,head=nearest_path_error(x,y,yaw,path) if path else (float('nan'),float('nan')); vals=[]
            for k in ('raw','smoothed','final'):
                pair=nearest_values([t],[(s,(a,b)) for s,a,b in data[k]])[0] if data[k] else (float('nan'),float('nan')); vals.extend(pair)
            cloud=nearest_values([t],data['cloud'])[0] if data['cloud'] else float('nan'); age=previous_age([t],odom_times)[0]; w.writerow([t,x,y,yaw,vx,wz,age,cross,head,*vals,cloud])
if __name__=='__main__': main()
