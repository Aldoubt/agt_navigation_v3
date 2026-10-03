"""Compile manually reviewed stable-structure regions into an immutable PCD."""
import argparse
import hashlib
import math
from pathlib import Path
import numpy as np
import yaml
from agt_map_converter.pcd_to_nav_map import load_xyz
from .manifest import atomic_yaml,digest
from .geometry import transform


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',required=True);parser.add_argument('--regions',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    try:
        spec=yaml.safe_load(Path(args.regions).read_text())
        if (spec.get('schema_version')!=1 or spec.get('frame_id')!='map' or
                spec.get('reviewed') is not True or not str(spec.get('reviewed_by','')).strip()):
            raise ValueError('reviewed map-frame regions and provenance required')
        source=digest(args.map)
        if spec.get('source_map_sha256')!=source['files'][0]['sha256']:
            raise ValueError('stable regions must refer to this exact source PCD hash')
        points=load_xyz(Path(args.map))
        retained=np.zeros(len(points),dtype=bool)
        regions=spec.get('regions')
        if not isinstance(regions,list) or not regions: raise ValueError('reviewed stable box regions required')
        for region in regions:
            if not str(region.get('id','')).strip() or not str(region.get('evidence','')).strip():
                raise ValueError('each stable structure requires id and evidence')
            center=np.asarray(region['center_xyz'],dtype=float)
            size=np.asarray(region['size_xyz'],dtype=float)
            angles=np.asarray(region['rpy'],dtype=float)
            if any(v.shape!=(3,) or not np.isfinite(v).all() for v in (center,size,angles)) or (size<=0).any():
                raise ValueError('finite region transform and positive size required')
            import xml.etree.ElementTree as ET
            origin=ET.Element('origin',xyz=' '.join(map(str,center)),rpy=' '.join(map(str,angles)))
            pose=transform(origin)
            local=(points-center)@pose[:3,:3]
            retained|=(np.abs(local)<=size/2).all(axis=1)
        stable=points[retained]
        if not len(stable): raise ValueError('reviewed regions selected no finite points')
        output=Path(args.output).resolve()
        output.mkdir(parents=True,exist_ok=False)
        cloud=output/'stable.pcd'
        with cloud.open('x') as stream:
            stream.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 8 8 8\nTYPE F F F\nCOUNT 1 1 1\n')
            stream.write(f'WIDTH {len(stable)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(stable)}\nDATA ascii\n')
            np.savetxt(stream,stable,fmt='%.17g')
        np.save(output/'finite_xyz_indices.npy',np.flatnonzero(retained),allow_pickle=False)
        if digest(args.map)['sha256']!=source['sha256']:
            raise ValueError('source PCD changed during stable selection')
        atomic_yaml(output/'stable_mask.yaml',{'schema_version':1,'frame_id':'map',
                    'method':'manual_reviewed_oriented_boxes','reviewed_by':spec['reviewed_by'],
                    'source_map':source,'regions':digest(args.regions),'finite_input_points':len(points),
                    'stable_points':len(stable),'stable_map':digest(cloud),
                    'indices':digest(output/'finite_xyz_indices.npy'),
                    'index_domain':'finite XYZ sequence returned by agt_map_converter.load_xyz',
                    'native_assets_must_be_rebuilt_for':str(cloud)})
        print(str(cloud))
    except (OSError,ValueError,KeyError,TypeError) as exc: parser.exit(2,str(exc)+'\n')
