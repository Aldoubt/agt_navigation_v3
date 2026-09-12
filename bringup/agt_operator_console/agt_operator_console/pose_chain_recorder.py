"""Passive native/adapter odometry recorder for offline replay."""
from __future__ import annotations
import json, math
from collections import deque
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

def stamp(value): return value.sec + value.nanosec / 1e9
def pose(msg):
 p,q=msg.pose.pose.position,msg.pose.pose.orientation
 return {'timestamp':stamp(msg.header.stamp),'frame_id':msg.header.frame_id,'child_frame_id':msg.child_frame_id,'position':[p.x,p.y,p.z],'quaternion_xyzw':[q.x,q.y,q.z,q.w]}
def rotation_delta(a,b):
 dot=abs(sum(x*y for x,y in zip(a,b))); return math.degrees(2*math.acos(min(1,max(-1,dot))))
class Recorder(Node):
 def __init__(self):
  super().__init__('pose_chain_recorder'); self.declare_parameter('report_dir',''); root=Path(self.get_parameter('report_dir').value).expanduser(); root.mkdir(parents=True,exist_ok=True); self.out=root/'pose_chain_runtime.jsonl';self.f=self.out.open('w');self.native=deque(maxlen=1000);self.rows=[];self.clock=None
  self.create_subscription(Clock,'/clock',lambda m:setattr(self,'clock',stamp(m.clock)),10);self.create_subscription(Odometry,'/aft_mapped_to_init',self.up,100);self.create_subscription(Odometry,'/agt/odometry/local',self.adapter,100)
 def up(self,m): self.native.append(pose(m))
 def adapter(self,m):
  a=pose(m)
  if not self.native:return
  n=min(self.native,key=lambda x:abs(x['timestamp']-a['timestamp'])); d=[a['position'][i]-n['position'][i] for i in range(3)];row={'clock':self.clock,'native':n,'adapter':a,'timestamp_delta_sec':a['timestamp']-n['timestamp'],'translation_delta_m':math.sqrt(sum(x*x for x in d)),'translation_delta_xyz_m':d,'rotation_delta_deg':rotation_delta(n['quaternion_xyzw'],a['quaternion_xyzw'])}; self.f.write(json.dumps(row)+'\n');self.f.flush();self.rows.append(row)
 def destroy_node(self):
  if self.rows:
   vals=lambda k:sorted(r[k] for r in self.rows); summary={'count':len(self.rows),'translation_delta_m':stats(vals('translation_delta_m')),'rotation_delta_deg':stats(vals('rotation_delta_deg')),'timestamp_delta_sec':stats([abs(r['timestamp_delta_sec']) for r in self.rows]),'native_frame_contract':frames(self.rows,'native'),'adapter_frame_contract':frames(self.rows,'adapter')};(self.out.parent/'pose_chain_summary.yaml').write_text(json.dumps(summary,indent=2)+'\n')
  self.f.close();super().destroy_node()
def stats(v): return {'mean':sum(v)/len(v),'p95':v[round(.95*(len(v)-1))],'max':v[-1]}
def frames(rows,key): return sorted({(r[key]['frame_id'],r[key]['child_frame_id']) for r in rows})
def main():
 rclpy.init();n=Recorder()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.shutdown()
