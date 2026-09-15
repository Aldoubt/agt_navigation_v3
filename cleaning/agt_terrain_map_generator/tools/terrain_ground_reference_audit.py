#!/usr/bin/env python3
"""MQ3-B.2 read-only ground-reference propagation evidence."""
import argparse, csv, math
from pathlib import Path
from terrain_map_align_audit import load, pcd_xyz, dump_yaml, REGIONS

RADII=(.2,.3,.5,1.)
def median(x):
 x=sorted(x); return x[len(x)//2] if len(x)%2 else .5*(x[len(x)//2-1]+x[len(x)//2])
def solve(a,b):
 for i in range(3):
  p=max(range(i,3),key=lambda j:abs(a[j][i])); a[i],a[p]=a[p],a[i]; b[i],b[p]=b[p],b[i]
  if abs(a[i][i])<1e-9:return None
  q=a[i][i]; a[i]=[v/q for v in a[i]]; b[i]/=q
  for j in range(3):
   if j!=i:
    q=a[j][i]; a[j]=[v-q*w for v,w in zip(a[j],a[i])]; b[j]-=q*b[i]
 return b
def pgm(path,m,values,scale=1):
 with open(path,'wb') as f:
  f.write(f'P5\n{m["w"]} {m["h"]}\n255\n'.encode())
  for y in range(m['h']-1,-1,-1):f.write(bytes(values[y*m['w']:(y+1)*m['w']]))
def main():
 p=argparse.ArgumentParser();p.add_argument('--mq3-dir',required=True);p.add_argument('--output-dir');a=p.parse_args(); root=Path(a.mq3_dir);out=Path(a.output_dir or root/'ground_reference_audit');out.mkdir(parents=True,exist_ok=True);m=load(root)
 n=m['w']*m['h']; samples=[[] for _ in range(n)]; non=[[] for _ in range(n)]
 def ci(x,y):
  ix=math.floor((x-m['o'][0])/m['r']);iy=math.floor((y-m['o'][1])/m['r']);return iy*m['w']+ix if 0<=ix<m['w'] and 0<=iy<m['h'] else None
 for x,y,z in pcd_xyz(root/'ground_map.pcd'):
  i=ci(x,y)
  if i is not None:samples[i].append(z)
 for x,y,z in pcd_xyz(root/'nonground_map.pcd'):
  i=ci(x,y)
  if i is not None:non[i].append((x,y,z))
 ground={}; buckets={}
 for i,s in enumerate(samples):
  if len(s)>=2:
   x=m['o'][0]+(i%m['w']+.5)*m['r'];y=m['o'][1]+(i//m['w']+.5)*m['r'];ground[i]=(x,y,median(s));buckets.setdefault((i%m['w']//5,i//m['w']//5),[]).append(i)
 def near(x,y,r):
  ix=math.floor((x-m['o'][0])/m['r']);iy=math.floor((y-m['o'][1])/m['r']);q=[];br=math.ceil(r/(m['r']*5))
  for bx in range(ix//5-br,ix//5+br+1):
   for by in range(iy//5-br,iy//5+br+1):
    for j in buckets.get((bx,by),[]):
     gx,gy,gz=ground[j];d=math.hypot(gx-x,gy-y)
     if d<=r:q.append((d,gx,gy,gz,j))
  return q
 stats={k:{'reference_points':0,'reference_cells':0,'hag_points':0,'hag_cells':0,'hag_ge_015_points':0,'hag_ge_025_points':0,'hag_ge_050_points':0,'hag_ge_100_points':0} for k in ['same_cell','nearest_02','nearest_03','nearest_05','nearest_10','local_plane']}; rows=[]; dist=[]; dz=[]; plane_support=[];plane_rmse=[];plane_slope=[]; refdist=[255]*n;source=[205]*n;hagimg=[205]*n;cand=[205]*n
 for i,pts in enumerate(non):
  if not pts:continue
  cellref={k:False for k in stats}; cellhag={k:False for k in stats}; refs={}
  for x,y,z in pts:
   methods={}
   if i in ground: methods['same_cell']=(ground[i][2],0,None)
   q=near(x,y,1.)
   for r,key in zip(RADII,['nearest_02','nearest_03','nearest_05','nearest_10']):
    c=[v for v in q if v[0]<=r]
    if c:
     v=min(c);methods[key]=(v[3],v[0],v[4])
     if key=='nearest_10':dist.append(v[0]);dz.append(abs(z-v[3]))
   s=[v for v in q if v[0]<=.5]
   if len(s)>=3:
    A=[[sum(v[1]*v[1] for v in s),sum(v[1]*v[2] for v in s),sum(v[1] for v in s)],[sum(v[1]*v[2] for v in s),sum(v[2]*v[2] for v in s),sum(v[2] for v in s)],[sum(v[1] for v in s),sum(v[2] for v in s),len(s)]];B=[sum(v[1]*v[3] for v in s),sum(v[2]*v[3] for v in s),sum(v[3] for v in s)];coef=solve(A,B)
    if coef:
     gz=coef[0]*x+coef[1]*y+coef[2];rm=math.sqrt(sum((coef[0]*v[1]+coef[1]*v[2]+coef[2]-v[3])**2 for v in s)/len(s));sl=math.degrees(math.atan(math.hypot(coef[0],coef[1])));methods['local_plane']=(gz,rm,None);plane_support.append(len(s));plane_rmse.append(rm);plane_slope.append(sl)
   for key,(gz,extra,src) in methods.items():
    h=z-gz
    stats[key]['reference_points']+=1;cellref[key]=True
    if 0<=h<=3:
     stats[key]['hag_points']+=1;cellhag[key]=True
     for t,name in [(.15,'hag_ge_015_points'),(.25,'hag_ge_025_points'),(.5,'hag_ge_050_points'),(1,'hag_ge_100_points')]:stats[key][name]+=h>=t
     refs[key]=max(refs.get(key,-1),h)
  for key in stats:
   stats[key]['reference_cells']+=cellref[key];stats[key]['hag_cells']+=cellhag[key]
  if 'nearest_10' in refs:refdist[i]=254;source[i]=254
  if 'nearest_05' in refs:hagimg[i]=min(254,int(min(3,refs['nearest_05'])/3*254));cand[i]=0 if refs['nearest_05']>=.25 else 254
 for key in stats:
  stats[key]['total_nonground_points']=sum(len(x) for x in non);stats[key]['total_nonground_cells']=sum(bool(x) for x in non)
 def bands(v,edges):return {str(e):sum(x<=e for x in v) for e in edges}
 # One canonical per-point nearest array and plane estimate drive all B.2.1 reports.
 region={r[0]:[] for r in REGIONS}; agree=[]; bins=[{'reference_points':0,'valid_hag_points':0,'hag_ge_025_points':0,'hag_ge_050_points':0,'hag_ge_100_points':0} for _ in range(4)]
 for i,pts in enumerate(non):
  for x,y,z in pts:
   q=near(x,y,1.)
   if not q: continue
   v=min(q); d,gz=v[0],v[3]; h=z-gz; bi=0 if d<=.2 else 1 if d<=.3 else 2 if d<=.5 else 3; b=bins[bi];b['reference_points']+=1
   if 0<=h<=3:
    b['valid_hag_points']+=1;b['hag_ge_025_points']+=h>=.25;b['hag_ge_050_points']+=h>=.5;b['hag_ge_100_points']+=h>=1
   s=[u for u in q if u[0]<=.5]; plane=None
   if len(s)>=3:
    A=[[sum(u[1]*u[1] for u in s),sum(u[1]*u[2] for u in s),sum(u[1] for u in s)],[sum(u[1]*u[2] for u in s),sum(u[2]*u[2] for u in s),sum(u[2] for u in s)],[sum(u[1] for u in s),sum(u[2] for u in s),len(s)]];B=[sum(u[1]*u[3] for u in s),sum(u[2]*u[3] for u in s),sum(u[3] for u in s)];c=solve(A,B)
    if c: plane=c[0]*x+c[1]*y+c[2]
   if d<=.5 and plane is not None and 0<=h<=3 and 0<=z-plane<=3:agree.append((abs(gz-plane),h>=.25,z-plane>=.25))
   for rid,x0,y0,x1,y1 in REGIONS:
    if x0<=x<x1 and y0<=y<y1:region[rid].append((z-(ground[i][2] if i in ground else None) if i in ground else None,h,z-plane if plane is not None else None))
 def qtile(v,f):
  v=sorted(v);return v[int(f*(len(v)-1))] if v else ''
 def method(vals,k):
  a=[x[k] for x in vals if x[k] is not None and 0<=x[k]<=3];return {'reference_points':sum(x[k] is not None for x in vals),'valid_hag_points':len(a),'hag_ge_025_points':sum(x>=.25 for x in a),'hag_ge_050_points':sum(x>=.5 for x in a),'hag_ge_100_points':sum(x>=1 for x in a),'max_hag':max(a) if a else '','p50_hag':qtile(a,.5),'p90_hag':qtile(a,.9)}
 regout={};
 for rid,v in region.items():
  n03=method(v,1);pl=method(v,2);n05={'hag_ge_025_points':n03['hag_ge_025_points']};status='RECOVERED_STRONG' if n03['hag_ge_025_points'] and pl['hag_ge_025_points'] else 'RECOVERED_PARTIAL' if n03['hag_ge_025_points'] else 'NOT_RECOVERED';regout[rid]={'nonground_points':len(v),'same_cell':method(v,0),'nearest_03':n03,'nearest_05':n05,'local_plane':pl,'status':status}
 with open(out/'static_obstacle_reference_recovery.csv','w') as f:
  f.write('region_id,nonground_points,nearest03_hag_ge_025_points,plane_hag_ge_025_points,status\n');[f.write(f'{k},{v["nonground_points"]},{v["nearest_03"]["hag_ge_025_points"]},{v["local_plane"]["hag_ge_025_points"]},{v["status"]}\n') for k,v in regout.items()]
 ad=[x[0] for x in agree];cls={'both_below_025':sum(not x[1] and not x[2] for x in agree),'both_ge_025':sum(x[1] and x[2] for x in agree),'nearest_only_ge_025':sum(x[1] and not x[2] for x in agree),'plane_only_ge_025':sum(not x[1] and x[2] for x in agree)}
 agreement={'common_reference_points':len(agree),'ground_delta':{'le_0_05':sum(x<=.05 for x in ad),'le_0_10':sum(x<=.1 for x in ad),'le_0_20':sum(x<=.2 for x in ad),'gt_0_20':sum(x>.2 for x in ad)},**cls,'classification_agreement_ratio':(cls['both_below_025']+cls['both_ge_025'])/len(agree) if agree else 0}
 db={k:v for k,v in zip(['0_02','02_03','03_05','05_10'],bins)};total=sum(x['hag_ge_025_points'] for x in bins)
 for v in db.values():v['fraction_of_all_hag_ge_025']=v['hag_ge_025_points']/total if total else 0
 dump_yaml(out/'reference_method_agreement.yaml',agreement);dump_yaml(out/'distance_bin_hag_contribution.yaml',db)
 coverage={k:{'reference_points':v['reference_points'],'reference_cells':v['reference_cells'],'hag_points':v['hag_points'],'hag_cells':v['hag_cells']} for k,v in stats.items()}
 dump_yaml(out/'ground_reference_metadata.yaml',{'methods':['SAME_CELL','NEAREST_GROUND','LOCAL_PLANE'],'radii_m':list(RADII),'credible_ground_cells':len(ground),'original_elevation_unchanged':True,'map_pgm_unchanged':True})
 dump_yaml(out/'ground_reference_coverage.yaml',coverage);dump_yaml(out/'propagation_risk_report.yaml',{'nearest_distance_cumulative':bands(dist,[.2,.3,.5,1]),'nearest_height_delta_cumulative':bands(dz,[.1,.2,.5]),'plane_support_cumulative':bands(plane_support,[3,5,10]),'plane_rmse_cumulative':bands(plane_rmse,[.03,.05,.1]),'plane_slope_cumulative':bands(plane_slope,[10,20,30])});dump_yaml(out/'evidence_accounting_contract.yaml',{'definitions':{'valid_reference':'geometric reference regardless of HAG sign','valid_hag':'0 <= point.z-reference.z <= 3.0m'},'denominators':{'total_nonground_points':sum(len(x) for x in non),'total_nonground_cells':sum(bool(x) for x in non)}});dump_yaml(out/'mq3_b21_reconciliation_report.yaml',{'accounting':'PASS','same_cell':coverage['same_cell'],'nearest':coverage,'local_plane':coverage['local_plane'],'static_regions':{k:v['status'] for k,v in regout.items()},'method_agreement':agreement,'distance_bin_contribution':db,'final_verdict':'REFERENCE_RECOVERY_PARTIAL'})
 pgm(out/'reference_distance.pgm',m,refdist);pgm(out/'reference_source.pgm',m,source);pgm(out/'hag_recovered.pgm',m,hagimg);pgm(out/'hag_obstacle_candidate.pgm',m,cand)
 with open(out/'ground_reference_cells.csv','w') as f:
  f.write('cell_index,ground_points,nonground_points,same_cell,nearest_02,nearest_03,nearest_05,nearest_10,local_plane\n')
  for i in range(n):
   if non[i]:f.write(f'{i},{len(samples[i])},{len(non[i])},{int(i in ground)},{int(refdist[i]<255)},{int(refdist[i]<255)},{int(hagimg[i]<205)},{int(refdist[i]<255)},{int(False)}\n')
if __name__=='__main__':main()
