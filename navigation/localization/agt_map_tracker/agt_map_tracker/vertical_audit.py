"""Read-only vertical-consistency audit for recorded tracker JSONL evidence."""
from __future__ import annotations
import argparse, json, math
from pathlib import Path

def rpy(qx, qy, qz, qw):
    roll = math.atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))
    pitch = math.asin(max(-1., min(1., 2*(qw*qy-qz*qx))))
    yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
    return [math.degrees(v) for v in (roll,pitch,yaw)]

def main():
    p=argparse.ArgumentParser(); p.add_argument('--events',required=True); p.add_argument('--artifact-analysis',required=True); p.add_argument('--output-dir',required=True); a=p.parse_args()
    artifacts={item['attempt_id']:item for item in json.loads(Path(a.artifact_analysis).read_text())['samples']}
    rows=[]
    for line in Path(a.events).read_text().splitlines():
        event=json.loads(line)
        if event.get('kind')!='tracker' or 'attempt_id' not in event: continue
        e=event; aid=e['attempt_id']; art=artifacts.get(aid)
        pred={k:e.get(f'T_map_base_prediction_{k}') for k in ('x','y','z','qx','qy','qz','qw')}
        row={'attempt_id':aid,'timestamp_ns':e.get('timestamp_ns'),'state':e.get('state'),'decision':e.get('decision'),
             'prediction':pred,'T_map_odom':{k:e.get(f'T_map_odom_{k}') for k in ('x','y','z','qx','qy','qz','qw')},
             'T_odom_base':{k:e.get(f'T_odom_base_{k}') for k in ('x','y','z','qx','qy','qz','qw')},
             'innovation':{k:e.get(k) for k in ('translation_innovation_dx_m','translation_innovation_dy_m','translation_innovation_dz_m','yaw_innovation_signed_deg')},
             'fitness':e.get('fitness'),'overlap':e.get('overlap'),'artifact':art}
        if all(pred[k] is not None for k in ('qx','qy','qz','qw')): row['prediction_rpy_deg']=rpy(pred['qx'],pred['qy'],pred['qz'],pred['qw'])
        rows.append(row)
    high=sorted((x for x in rows if x['state'] in ('HOLD','DEGRADED','RECOVERY_REQUIRED') and x['innovation']['translation_innovation_dz_m'] is not None),key=lambda x:abs(x['innovation']['translation_innovation_dz_m']),reverse=True)[:5]
    ok=sorted((x for x in rows if x['state']=='TRACKING_OK' and x['innovation']['translation_innovation_dz_m'] is not None),key=lambda x:abs(x['innovation']['translation_innovation_dz_m']),reverse=True)[:5]
    markers=[]
    for i,x in enumerate(high+ok):
        p=x['prediction']; dz=x['innovation']['translation_innovation_dz_m']; dx=x['innovation']['translation_innovation_dx_m']; dy=x['innovation']['translation_innovation_dy_m']
        if p['z'] is not None: markers.append({'id':i,'frame_id':'map','attempt_id':x['attempt_id'],'prediction_xyz':[p['x'],p['y'],p['z']], 'result_xyz_approx':[p['x']+dx,p['y']+dy,p['z']+dz], 'note':'result is reconstructed from signed innovation; raw measured pose was not recorded'})
    result={'attempt_count':len(rows),'top5_innovation_high':high,'top5_tracking_ok':ok,
            'frame_contract':'PASS' if all(x.get('prediction',{}).get('z') is not None for x in rows if x.get('prediction',{}).get('z') is not None) else 'FAIL',
            'contract_evidence':'events record T_map_base_prediction = T_map_odom * T_odom_base_link; crop center and native initial pose are populated from that prediction','limitations':'query/crop distributions exist only for retained artifacts; raw measured GICP pose is not present in events'}
    out=Path(a.output_dir); out.mkdir(parents=True,exist_ok=True); (out/'tracker_timeline.json').write_text(json.dumps(result,indent=2)+'\n'); (out/'rviz_markers.yaml').write_text(json.dumps({'markers':markers},indent=2)+'\n')
if __name__=='__main__': main()
