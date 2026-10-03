"""Derive perception self shapes from the same expanded URDF used for TF."""
import argparse
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from .manifest import atomic_yaml


def vector(text, default):
    values=[float(x) for x in text.split()] if text else list(default)
    if len(values)!=3 or not all(math.isfinite(v) for v in values):
        raise ValueError('finite xyz/rpy vector required')
    return values


def transform(origin):
    xyz=vector(None if origin is None else origin.get('xyz'),[0,0,0])
    r,p,y=vector(None if origin is None else origin.get('rpy'),[0,0,0])
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    out=np.eye(4)
    out[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    out[:3,3]=xyz
    return out


def rpy(rot):
    pitch=math.atan2(-rot[2,0],math.hypot(rot[0,0],rot[1,0]))
    if abs(math.cos(pitch))<1e-9:
        raise ValueError('singular collision orientation requires explicit geometry review')
    return [math.atan2(rot[2,1],rot[2,2]),pitch,math.atan2(rot[1,0],rot[0,0])]


def derive(urdf, profile, base='base_link', padding=0.0):
    data=Path(urdf).read_bytes()
    model=ET.fromstring(data)
    if model.tag!='robot' or any('xacro' in elem.tag for elem in model.iter()):
        raise ValueError('expanded URDF required; expand the selected measured xacro first')
    joints={j.find('child').get('link'):j for j in model.findall('joint')}
    def at_base(link, seen=()):
        if link==base: return np.eye(4)
        if link in seen or link not in joints: raise ValueError('collision frame cannot reach base: '+link)
        joint=joints[link]
        if joint.get('type')!='fixed':
            raise ValueError('moving payload needs a measured runtime shape; cannot freeze '+link)
        return at_base(joint.find('parent').get('link'),seen+(link,))@transform(joint.find('origin'))
    shapes=[]
    for link in model.findall('link'):
        for index,collision in enumerate(link.findall('collision')):
            pose=at_base(link.get('name'))@transform(collision.find('origin'))
            shape={'id':link.get('name')+'_'+str(index),'center_xyz':pose[:3,3].tolist(),
                   'rpy':rpy(pose[:3,:3]),'padding_m':padding}
            geom=collision.find('geometry')
            if geom is None: raise ValueError('collision geometry missing')
            box,cylinder=geom.find('box'),geom.find('cylinder')
            if box is not None:
                size=vector(box.get('size'),[])
                if min(size)<=0: raise ValueError('positive box dimensions required')
                shape.update(type='box',size_xyz=size)
            elif cylinder is not None:
                radius,height=float(cylinder.get('radius')),float(cylinder.get('length'))
                if not all(math.isfinite(v) and v>0 for v in (radius,height)):
                    raise ValueError('positive cylinder dimensions required')
                shape.update(type='cylinder',radius_m=radius,height_m=height)
            else:
                raise ValueError('mesh/sphere collision requires reviewed box/cylinder proxy: '+link.get('name'))
            shapes.append(shape)
    if not shapes: raise ValueError('no collision shapes found')
    cfg=yaml.safe_load(Path(profile).read_text())
    if cfg['frames']['base']!=base: raise ValueError('Robot Profile base frame mismatch')
    return {'schema_version':1,'robot_profile':cfg['robot_id'],
            'model_sha256':hashlib.sha256(data).hexdigest(),
            'geometry':{'frame_id':base,'field_verified':False,
                        'footprint':cfg['geometry']['footprint'],'vehicle_height_m':0.0,
                        'braking_deceleration_mps2':0.0,'command_latency_sec':0.0,
                        'safety_margin_m':cfg['geometry']['safety_margin'],'self_shapes':shapes}}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',required=True);parser.add_argument('--profile',required=True)
    parser.add_argument('--base-frame',default='base_link');parser.add_argument('--padding-m',type=float,default=0.0)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    if not math.isfinite(args.padding_m) or args.padding_m<0: parser.error('nonnegative padding required')
    try: atomic_yaml(args.output,derive(args.urdf,args.profile,args.base_frame,args.padding_m))
    except (ValueError,OSError,KeyError,ET.ParseError) as exc: parser.exit(2,str(exc)+'\n')
