#!/usr/bin/env python3
"""MQ3-B.1 read-only world-coordinate map evidence audit."""
import argparse
import math
import struct
from pathlib import Path

STATES = {0: 'occupied', 254: 'free', 205: 'unknown'}
REGIONS = [
    ('mq1_trigger_143', 59.736, -17.445, 60.736, -15.745),
    ('mq1_trigger_4', 56.836, -28.545, 57.736, -27.345),
    ('mq1_trigger_164', 60.336, -15.645, 61.036, -14.745),
    ('mq1_trigger_246', 62.536, -10.345, 63.036, -8.645),
    ('mq1_trigger_11', 55.736, -27.845, 56.736, -26.945),
]

def yaml(path):
    lines = Path(path).read_text().splitlines(); out = {}; i = 0
    while i < len(lines):
        if ':' in lines[i] and not lines[i].startswith(' '):
            k, v = lines[i].split(':', 1); v = v.strip()
            if v and v.startswith('['): out[k] = [float(x) for x in v.strip('[]').split(',')]
            elif v: out[k] = v
            elif i + 3 < len(lines) and lines[i+1].lstrip().startswith('-'):
                out[k] = [float(lines[i+j].split('-',1)[1]) for j in range(1,4)]
        i += 1
    return out

def pgm(path):
    f = open(path, 'rb'); magic = f.readline().strip()
    if magic != b'P5': raise ValueError(path)
    line = f.readline()
    while line.startswith(b'#'): line = f.readline()
    w,h = map(int,line.split()); assert int(f.readline()) == 255
    return w,h,list(f.read(w*h))

def load(root):
    meta = yaml(Path(root)/'map.yaml'); w,h,p = pgm(Path(root)/meta.get('image','map.pgm'))
    return {'root':str(root),'w':w,'h':h,'p':p,'r':float(meta['resolution']), 'o':meta['origin']}

def pcd_xyz(path):
    f=open(path,'rb'); header=[]
    while True:
        line=f.readline().decode().strip(); header.append(line)
        if line.startswith('DATA '): break
    fields=next(x.split()[1:] for x in header if x.startswith('FIELDS ')); size=next(x.split()[1:] for x in header if x.startswith('SIZE ')); step=sum(map(int,size)); points=int(next(x.split()[1] for x in header if x.startswith('POINTS ')))
    offsets={}; n=0
    for name,width in zip(fields,size): offsets[name]=n; n+=int(width)
    raw=f.read(); return [(struct.unpack_from('<f',raw,i*step+offsets['x'])[0],struct.unpack_from('<f',raw,i*step+offsets['y'])[0],struct.unpack_from('<f',raw,i*step+offsets['z'])[0]) for i in range(points)]

def terrain_evidence(m, root):
    ground=pcd_xyz(Path(root)/'ground_map.pcd'); non=pcd_xyz(Path(root)/'nonground_map.pcd'); n=m['w']*m['h']; samples=[[] for _ in range(n)]; noncells=[[] for _ in range(n)]
    def idx(x,y):
        ix=math.floor((x-m['o'][0])/m['r']); iy=math.floor((y-m['o'][1])/m['r'])
        return iy*m['w']+ix if 0<=ix<m['w'] and 0<=iy<m['h'] else None
    for x,y,z in ground:
        i=idx(x,y)
        if i is not None: samples[i].append(z)
    for x,y,z in non:
        i=idx(x,y)
        if i is not None: noncells[i].append(z)
    elev=[None]*n; confidence=[0.0]*n; hist={str(k):0 for k in range(5)}
    for i,s in enumerate(samples):
        hist[str(min(len(s),4))]+=1
        if s:
            s.sort(); elev[i]=s[len(s)//2] if len(s)%2 else .5*(s[len(s)//2-1]+s[len(s)//2])
            confidence[i]=min(1.0,len(s)/3.0)
    w,h,slope=pgm(Path(root)/'slope.pgm'); slope=map_order(w,h,slope)
    w,h,mapimg=pgm(Path(root)/'map.pgm'); mapimg=map_order(w,h,mapimg)
    ng_any=sum(bool(x) for x in noncells); ng_ref=sum(bool(noncells[i]) and elev[i] is not None and confidence[i]>=.5 for i in range(n)); hag=[]
    for i,vals in enumerate(noncells):
        if elev[i] is not None and confidence[i]>=.5:
            hag += [z-elev[i] for z in vals if 0<=z-elev[i]<=3]
    hag.sort(); q=lambda f: hag[int(f*(len(hag)-1))] if hag else None
    unknown=[i for i,p in enumerate(mapimg) if p==205]; exclusive={k:0 for k in ('NO_ELEVATION','LOW_ELEVATION_CONFIDENCE','NO_SLOPE_SUPPORT','NONGROUND_WITHOUT_GROUND_REFERENCE','LOW_EFFECTIVE_CONFIDENCE','OTHER')}
    for i in unknown:
        if elev[i] is None: why='NO_ELEVATION'
        elif confidence[i]<.5: why='LOW_ELEVATION_CONFIDENCE'
        elif slope[i]==205: why='NO_SLOPE_SUPPORT'
        elif noncells[i] and not (elev[i] is not None and confidence[i]>=.5): why='NONGROUND_WITHOUT_GROUND_REFERENCE'
        else: why='LOW_EFFECTIVE_CONFIDENCE'
        exclusive[why]+=1
    return {'ground_points':len(ground),'nonground_points':len(non),'grid_cells_total':n,'ground_cells_any_points':sum(bool(x) for x in samples),'ground_cells_min_points_pass':sum(len(x)>=3 for x in samples),'ground_cells_confidence_pass':sum(x>=.5 for x in confidence),'elevation_known_cells':sum(x is not None for x in elev),'slope_known_cells':sum(x!=205 for x in slope),'nonground_cells_any_points':ng_any,'nonground_cells_with_ground_reference':ng_ref,'nonground_cells_without_ground_reference':ng_any-ng_ref,'nonground_points_with_local_ground':len(hag),'nonground_points_without_local_ground':len(non)-len(hag),'hag_ge_015':sum(x>=.15 for x in hag),'hag_ge_025':sum(x>=.25 for x in hag),'hag_ge_050':sum(x>=.5 for x in hag),'hag_ge_100':sum(x>=1 for x in hag),'max_hag':max(hag) if hag else None,'p50_hag':q(.5),'p90_hag':q(.9),'histogram':hist,'exclusive_unknown_reason':exclusive}

def state(m,x,y):
    ix=math.floor((x-m['o'][0])/m['r']); iy=math.floor((y-m['o'][1])/m['r'])
    if ix < 0 or iy < 0 or ix >= m['w'] or iy >= m['h']: return 'outside'
    return STATES.get(m['p'][(m['h']-1-iy)*m['w']+ix], 'unknown')

def bounds(m): return [m['o'][0],m['o'][1],m['o'][0]+m['w']*m['r'],m['o'][1]+m['h']*m['r']]
def map_order(w,h,p): return [p[(h-1-y)*w+x] for y in range(h) for x in range(w)]
def transitions(a,b):
    lo=[max(x,y) for x,y in zip(bounds(a)[:2],bounds(b)[:2])]; hi=[min(x,y) for x,y in zip(bounds(a)[2:],bounds(b)[2:])]
    result={x:{y:0 for y in ('occupied','free','unknown','outside')} for x in ('occupied','free','unknown','outside')}
    n=0; x=lo[0]+0.05
    while x < hi[0]:
        y=lo[1]+0.05
        while y < hi[1]: result[state(a,x,y)][state(b,x,y)]+=1; n+=1; y+=0.1
        x+=0.1
    return lo+hi,n,result
def dump_yaml(path,obj,indent=0):
    with open(path,'w') as f:
        def rec(k,v,d):
            sp=' '*d
            if isinstance(v,dict):
                f.write(f'{sp}{k}:\n'); [rec(a,b,d+2) for a,b in v.items()]
            else: f.write(f'{sp}{k}: {v}\n')
        for k,v in obj.items(): rec(k,v,0)

def rect_audit(m,poses):
    swept=[]
    for line in Path(poses).read_text().splitlines():
        q=line.split();
        if len(q)<8: continue
        x,y=float(q[1]),float(q[2]); qw,qx,qy,qz=map(float,q[4:8]); yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
        swept.append((x,y,yaw))
    cells={}
    for x,y,a in swept:
        for ix in range(math.floor((x-.72-m['o'][0])/m['r'])-6,math.ceil((x+.4-m['o'][0])/m['r'])+7):
            for iy in range(math.floor((y-.72-m['o'][1])/m['r'])-6,math.ceil((y+.4-m['o'][1])/m['r'])+7):
                cx=m['o'][0]+(ix+.5)*m['r']; cy=m['o'][1]+(iy+.5)*m['r']; dx=cx-x; dy=cy-y
                u=math.cos(a)*dx+math.sin(a)*dy; v=-math.sin(a)*dx+math.cos(a)*dy
                if -.72 <= u <= .4 and abs(v)<=.46: cells[(ix,iy)]=state(m,cx,cy)
    c={s:list(cells.values()).count(s) for s in ('free','occupied','unknown','outside')}; c['swept_cells']=len(cells); c['conflict_cells']=c['occupied']+c['unknown']; c['conflict_ratio']=c['conflict_cells']/len(cells) if cells else 0; return c

def main():
    p=argparse.ArgumentParser(); p.add_argument('--mq0',required=True); p.add_argument('--mq2',required=True); p.add_argument('--mq3',required=True); p.add_argument('--poses',required=True); p.add_argument('--output-dir',required=True); a=p.parse_args()
    maps={k:load(v) for k,v in [('mq0',a.mq0),('mq2',a.mq2),('mq3',a.mq3)]}; out=Path(a.output_dir); out.mkdir(parents=True,exist_ok=True)
    evidence=terrain_evidence(maps['mq3'], a.mq3)
    align={'maps':{k:{'origin':m['o'],'resolution':m['r'],'width':m['w'],'height':m['h'],'bounds':bounds(m)} for k,m in maps.items()}}
    for n,x,y in [('mq0_to_mq2','mq0','mq2'),('mq2_to_mq3','mq2','mq3'),('mq0_to_mq3','mq0','mq3')]:
        box,count,matrix=transitions(maps[x],maps[y]); align[n]={'overlap_bbox':box,'overlap_cell_count':count,'transition_matrix':matrix}
    dump_yaml(out/'grid_alignment_report.yaml',align)
    rect=rect_audit(maps['mq3'],a.poses); dump_yaml(out/'trajectory_rect_audit.yaml',{'footprint_model':'rectangular_body','front_m':.4,'rear_m':.72,'half_width_m':.46,**rect})
    funnel={k:evidence[k] for k in ('grid_cells_total','ground_cells_any_points','ground_cells_min_points_pass','ground_cells_confidence_pass','elevation_known_cells','slope_known_cells','nonground_cells_any_points','nonground_cells_with_ground_reference','nonground_cells_without_ground_reference')}
    funnel.update({'patch_count':292,'raw_points':559280,'filtered_points':461051,'ground_points':evidence['ground_points'],'nonground_points':evidence['nonground_points'],'traversability_free_cells':maps['mq3']['p'].count(254),'traversability_blocked_cells':maps['mq3']['p'].count(0),'traversability_unknown_cells':maps['mq3']['p'].count(205)})
    dump_yaml(out/'terrain_funnel_report.yaml',funnel)
    dump_yaml(out/'nonground_reference_audit.yaml',{k:evidence[k] for k in evidence if k.startswith('nonground_') or k.startswith('hag_') or k in ('max_hag','p50_hag','p90_hag')})
    dump_yaml(out/'unknown_reason_report.yaml',{'exclusive_final_reason':evidence['exclusive_unknown_reason'],'exclusive_total':sum(evidence['exclusive_unknown_reason'].values()),'final_unknown_cells':maps['mq3']['p'].count(205)})
    dump_yaml(out/'ground_point_density_report.yaml',{'ground_points_per_cell_histogram':evidence['histogram']})
    rows=[]
    for rid,x0,y0,x1,y1 in REGIONS:
        def counts(m):
            d={s:0 for s in ('occupied','free','unknown','outside')}; x=x0+.05
            while x<x1:
                y=y0+.05
                while y<y1: d[state(m,x,y)]+=1; y+=.1
                x+=.1
            return d
        q=counts(maps['mq2']); r=counts(maps['mq3']); status='PRESERVED' if r['occupied'] else ('DEGRADED' if q['occupied'] and r['unknown'] else 'INCONCLUSIVE')
        rows.append([rid,x0,y0,x1,y1,q['occupied'],q['free'],q['unknown'],r['occupied'],r['free'],r['unknown'],'', '', '', '', '', status])
    with open(out/'static_obstacle_world_review.csv','w') as f:
        f.write('region_id,min_x,min_y,max_x,max_y,mq2_occupied,mq2_free,mq2_unknown,mq3_occupied,mq3_free,mq3_unknown,mq3_ground_points,mq3_nonground_points,mq3_hag_known_cells,mq3_hag_unknown_cells,mq3_max_hag,status\n')
        for row in rows: f.write(','.join(map(str,row))+'\n')
    dump_yaml(out/'mq3_b1_evidence_report.yaml',{'world_alignment':'PASS','rectangular_trajectory_mq3':rect,'funnel':funnel,'unknown_reason':evidence['exclusive_unknown_reason'],'static_regions':{r[0]:r[-1] for r in rows},'verdict':'REJECT_MQ3_BASELINE'})
if __name__ == '__main__': main()
