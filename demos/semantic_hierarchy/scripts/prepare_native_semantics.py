#!/usr/bin/env python3
"""Associate the pre-existing model-derived semantic map without geometry relabeling."""
import hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/reference_demo'
raw_path=ROOT/'data/semantic_map.npz'
raw=np.load(raw_path)
dense=np.load(OUT/'mapping_replay.npz')
p=dense['points']
distance,nearest=cKDTree(raw['points']).query(p,workers=4)
match=distance<=.12
labels=np.where(match,raw['labels'][nearest],0).astype(np.uint8)
# Preserve the existing go2w_anything_models.py vocabulary and application palette.
colors=np.array([[85,100,115],[45,170,255],[180,105,250],[255,165,35],
                 [250,65,135],[155,175,195],[65,195,135],[235,205,65]],np.uint8)
assert labels.max()<len(colors)
np.savez_compressed(OUT/'model_semantics.npz',labels=labels,colors=colors[labels],
                    source_index=np.where(match,nearest,-1).astype(np.int32))
packed=np.empty(len(p),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4'),('label','u1')])
for i,key in enumerate('xyz'):packed[key]=p[:,i]
rgb=colors[labels].astype(np.uint32)
packed['rgb']=(rgb[:,0]<<16)|(rgb[:,1]<<8)|rgb[:,2]
packed['label']=labels
header=('VERSION .7\nFIELDS x y z rgb label\nSIZE 4 4 4 4 1\nTYPE F F F U U\n'
        f'COUNT 1 1 1 1 1\nWIDTH {len(p)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(p)}\nDATA binary\n')
(OUT/'dense_model_semantic.pcd').write_bytes(header.encode()+packed.tobytes())
report=dict(source=str(raw_path.resolve()),sha256=hashlib.sha256(raw_path.read_bytes()).hexdigest(),
            source_pipeline='Grounding DINO + SAM2 masks / synchronized metric UE depth / LIO pose / temporal semantic voxel fusion',
            association_radius_m=.12,geometry_relabeling=False,
            class_ids=['unassigned','stairs','ramp','cylinder','person','wall','floor','stairs/ramp ambiguous'],
            palette=colors.tolist(),counts={str(k):int((labels==k).sum()) for k in range(8)},
            note='Preserves existing model-derived class IDs, not a full Hydra/S-Graphs inference result')
(OUT/'model_semantics_provenance.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
