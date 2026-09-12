"""Read-only T_map_odom * T_odom_base continuity audit."""
from __future__ import annotations
import argparse,json,math
from pathlib import Path
def rpy(q):
 x,y,z,w=q; return [math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))),math.degrees(math.asin(max(-1,min(1,2*(w*y-z*x))))),math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))]
def main():
 p=argparse.ArgumentParser();p.add_argument('--events',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args(); rows=[]
 for l in Path(a.events).read_text().splitlines():
  e=json.loads(l)
  if e.get('kind')!='tracker' or e.get('T_map_base_prediction_z') is None:continue
  def pose(prefix):
   q=[e[f'{prefix}_{k}'] for k in ('qx','qy','qz','qw')];return {'xyz':[e[f'{prefix}_{k}'] for k in ('x','y','z')],'rpy_deg':rpy(q)}
  rows.append({'attempt_id':e['attempt_id'],'timestamp_ns':e.get('timestamp_ns'),'T_map_odom':pose('T_map_odom'),'T_odom_base':pose('T_odom_base'),'T_map_base_computed':pose('T_map_base_prediction')})
 def deltas(key):
  out=[]
  for x,y in zip(rows,rows[1:]):
   t=math.dist(x[key]['xyz'],y[key]['xyz']); angles=[abs(a-b) for a,b in zip(x[key]['rpy_deg'],y[key]['rpy_deg'])];out.append({'attempt_id':y['attempt_id'],'translation_m':t,'roll_deg':angles[0],'pitch_deg':angles[1],'yaw_deg':angles[2]})
  return out
 ds={k:deltas(k) for k in ('T_map_odom','T_odom_base','T_map_base_computed')}
 def stat(values,k):
  v=sorted(x[k] for x in values);return {'p95':v[round(.95*(len(v)-1))],'max':v[-1],'max_attempt':max(values,key=lambda x:x[k])['attempt_id']}
 summary={k:{m:stat(v,m) for m in ('translation_m','roll_deg','pitch_deg','yaw_deg')} for k,v in ds.items()}
 source='MAP_ODOM_ANOMALY' if summary['T_map_odom']['translation_m']['max']>summary['T_odom_base']['translation_m']['max'] else 'ODOM_CHAIN_ANOMALY'
 result={'sample_count':len(rows),'timeline':rows,'continuity':summary,'responsibility':source,'markers':[{'id':i,'frame_id':'map','xyz':x['T_map_base_computed']['xyz'],'anomaly':x['T_map_base_computed']['xyz'][2]>3} for i,x in enumerate(rows)]}
 o=Path(a.output_dir);o.mkdir(parents=True,exist_ok=True);(o/'pose_chain_timeline.json').write_text(json.dumps(result,indent=2)+'\n');(o/'rviz_markers.yaml').write_text(json.dumps({'markers':result['markers']},indent=2)+'\n')
if __name__=='__main__':main()
