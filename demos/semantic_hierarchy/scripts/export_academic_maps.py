#!/usr/bin/env python3
"""Export display colors on unchanged measured coordinates and validate mapping."""
import hashlib
import json
from pathlib import Path
import numpy as np
from academic_palette import SEMANTIC, height_colors
from scene_profile import DATA_ROOT

ROOT=Path(__file__).resolve().parents[1]
OUT=DATA_ROOT/'outputs/academic_upgrade'

def export(name,xyz,rgb,labels=None):
    dtype=[(a,'<f4') for a in 'xyz']+[('rgb','<u4')]
    if labels is not None:dtype.append(('label','u1'))
    a=np.empty(len(xyz),dtype=dtype)
    for i,k in enumerate('xyz'):a[k]=xyz[:,i]
    c=rgb.astype(np.uint32);a['rgb']=(c[:,0]<<16)|(c[:,1]<<8)|c[:,2]
    if labels is not None:a['label']=labels
    suffix=' label' if labels is not None else ''
    header=(f'VERSION .7\nFIELDS x y z rgb{suffix}\nSIZE 4 4 4 4'+(' 1' if suffix else '')+
            '\nTYPE F F F U'+(' U' if suffix else '')+'\nCOUNT 1 1 1 1'+(' 1' if suffix else '')+
            f'\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n')
    path=OUT/(name+'.pcd');path.write_bytes(header.encode()+a.tobytes())
    check=np.frombuffer(path.read_bytes()[len(header):],dtype=dtype)
    assert np.array_equal(np.column_stack([check[k] for k in 'xyz']),xyz)
    assert np.array_equal(check['rgb'],a['rgb'])
    if labels is not None:assert np.array_equal(check['label'],labels)
    return dict(path=str(path),points=len(xyz),bytes=path.stat().st_size,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest(),xyz_exactly_preserved=True)

def main():
    xyz=np.load(DATA_ROOT/'outputs/reference_demo/mapping_replay.npz')['points']
    native=np.load(DATA_ROOT/'outputs/reference_demo/model_semantics.npz')
    source=np.load(DATA_ROOT/'data/semantic_map.npz')
    assigned=native['source_index']>=0
    assert np.array_equal(native['labels'][assigned],source['labels'][native['source_index'][assigned]])
    features=np.load(OUT/'utonia_features.npz')
    idx,inv=features['encoder_point_indices'],features['input_voxel_inverse']
    voxel=json.loads((OUT/'utonia_report.json').read_text())['input_voxel_m']
    assert np.array_equal(features['colors'],features['encoder_rgb'][inv])
    keys=np.floor(xyz/voxel).astype(np.int32)
    assert np.array_equal(keys,keys[idx][inv])
    assert len(np.unique(idx))==len(idx)
    assert np.isfinite(xyz).all()
    files=[export('academic_height',xyz,height_colors(xyz[:,2])),
           export('academic_semantic',xyz,SEMANTIC[native['labels']],native['labels']),
           export('utonia_features',xyz,features['colors'])]
    result=dict(status='passed',measured_points=len(xyz),original_class_ids_preserved=True,
                feature_colors_match_voxel_membership=True,encoder_input_points=len(idx),files=files,
                note='Integrity checks only; not accuracy evaluation. Utonia colors do not encode class labels.')
    (OUT/'map_integrity.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
