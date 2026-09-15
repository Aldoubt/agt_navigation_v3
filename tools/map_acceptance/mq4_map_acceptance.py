#!/usr/bin/env python3
"""MQ4 deterministic map-level preflight; never mutates maps or Nav2 params."""
import argparse, math
from pathlib import Path

def poses(path):
 out=[]
 for i,line in enumerate(Path(path).read_text().splitlines()):
  q=line.split()
  if len(q)>=8: out.append((i,float(q[1]),float(q[2])))
 return out
def pgm(path):
 f=open(path,'rb'); assert f.readline().strip()==b'P5'; line=f.readline()
 while line.startswith(b'#'):line=f.readline()
 w,h=map(int,line.split());f.readline();return w,h,list(f.read(w*h))
def mapload(root):
 text=(Path(root)/'map.yaml').read_text().splitlines(); d={};i=0
 while i<len(text):
  if ':' in text[i] and not text[i].startswith(' '):
   k,v=text[i].split(':',1);d[k]=v.strip()
   if k=='origin':d[k]=[float(text[i+j].split('-',1)[1]) for j in range(1,4)]
  i+=1
 w,h,p=pgm(Path(root)/d.get('image','map.pgm'));return d,w,h,p
def state(m,x,y):
 d,w,h,p=m;ix=math.floor((x-d['origin'][0])/float(d['resolution']));iy=math.floor((y-d['origin'][1])/float(d['resolution']))
 if not(0<=ix<w and 0<=iy<h):return 'outside'
 return {0:'occupied',254:'free',205:'unknown'}.get(p[(h-1-iy)*w+ix],'unknown')
def pairs(ps):
 result=[]
 for lo,hi,label in [(5,10,'short'),(15,30,'medium'),(30,1e9,'long')]:
  picked=0
  for direction in (1,-1):
   for i,a in enumerate(ps):
    for b in ps[i+1:]:
     d=math.hypot(a[1]-b[1],a[2]-b[2])
     if lo<=d<hi:
      s,g=(a,b) if direction==1 else (b,a);result.append((f'{label}_{"forward" if direction==1 else "reverse"}_{picked}',s,g,d));picked+=1;break
    if picked>=3:break
  # deterministic cap: 18 total, three direction-pairs per band
 return result[:18]
def dump(path, obj):
 def rec(f,k,v,n=0):
  pad=' '*n
  if isinstance(v,dict):f.write(f'{pad}{k}:\n');[rec(f,a,b,n+2) for a,b in v.items()]
  elif isinstance(v,list):f.write(f'{pad}{k}: {v}\n')
  else:f.write(f'{pad}{k}: {v}\n')
 with open(path,'w') as f:
  for k,v in obj.items():rec(f,k,v)
def main():
 p=argparse.ArgumentParser();p.add_argument('--poses',required=True);p.add_argument('--mq0',required=True);p.add_argument('--mq2',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args();out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True); ps=poses(a.poses);pp=pairs(ps)
 pair_doc={'pairs':[{'pair_id':n,'start':{'x':s[1],'y':s[2]},'goal':{'x':g[1],'y':g[2]},'straight_line_distance_m':round(d,4),'trajectory_index_start':s[0],'trajectory_index_goal':g[0]} for n,s,g,d in pp]};dump(out/'mq4_test_pairs.yaml',pair_doc)
 report={}
 for name,root in [('MQ0',a.mq0),('MQ2_A1B',a.mq2)]:
  m=mapload(root); rows=[]
  for n,s,g,d in pp:
   ss,gs=state(m,s[1],s[2]),state(m,g[1],g[2]);status='VALID' if ss=='free' and gs=='free' else 'START_BLOCKED' if ss!='free' else 'GOAL_BLOCKED'
   rows.append({'pair_id':n,'start_state':ss,'goal_state':gs,'preflight':status,'straight_line_distance_m':round(d,4)})
  report[name]={'map':{'resolution':m[0]['resolution'],'origin':m[0]['origin'],'width':m[1],'height':m[2]},'pairs':rows,'valid_start_goal':sum(x['preflight']=='VALID' for x in rows)}
 dump(out/'mq4_map_connectivity_report.yaml',report)
 dump(out/'mq4_nav2_ab_report.yaml',{'planner_config':'nav2_params.yaml: SmacPlanner2D, allow_unknown=true, inflation_radius=0.55, footprint=0.52x0.40','candidates':{k:{'preflight_valid_pairs':v['valid_start_goal'],'planner_status':'NOT_RUN_REQUIRES_LIFECYCLE_FIXTURE'} for k,v in report.items()},'verdict':'BOTH_REVIEW'})
if __name__=='__main__':main()
