#!/usr/bin/env python3
"""Validate dense replay provenance against sampled original scan messages."""
import json
from pathlib import Path
import numpy as np
import rosbag

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/reference_demo'
data=np.load(OUT/'mapping_replay.npz')
p,seen=data['points'],data['first_seen']
report=json.loads((OUT/'replay_report.json').read_text())
assert np.isfinite(p).all() and np.all(np.diff(seen)>=0)
assert len(p)==report['unique_measured_points']
q=np.floor(p/report['voxel_m']).astype(np.int64)+(1<<20)
keys=(q[:,0]<<42)|(q[:,1]<<21)|q[:,2]
assert len(np.unique(keys))==len(p), 'Duplicate occupied voxels'
assert np.all((p>[-3,-8,-1.5])&(p<[21,13,8.2]))
sample_ids=np.linspace(0,len(p)-1,100,dtype=int)
expected={}
for i in sample_ids:expected.setdefault(float(seen[i]),[]).append(p[i].tobytes())
verified=0
with rosbag.Bag(report['source_bag']) as bag:
    for _,msg,t in bag.read_messages(topics=[report['topic']]):
        timestamp=t.to_sec()
        if timestamp not in expected:continue
        offsets={f.name:f.offset for f in msg.fields}
        dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[offsets[k] for k in 'xyz'],'itemsize':msg.point_step})
        a=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step)).reshape(-1)
        xyz=np.column_stack([a[k] for k in 'xyz'])
        original={point.tobytes() for point in xyz}
        for point in expected[timestamp]:
            assert point in original,'Output XYZ is not an exact original return at its timestamp'
            verified+=1
assert verified==len(sample_ids)
assert set(np.unique(data['labels'])).issubset(range(7))
assert np.all(data['labels'][data['source_index']<0]==0)
annotations=json.loads((OUT/'semantic_annotations.json').read_text())
assert len(annotations)==27 and len({n['id'] for n in annotations})==27
assert all(n['associated_dense_points']>=15 for n in annotations)
assert not any(n['kind'] in ['person','elevator'] for n in annotations)
payload=(OUT/'dense_semantic_map.pcd').read_bytes().split(b'DATA binary\n',1)[1]
assert len(payload)==18*len(p)
native=np.load(OUT/'model_semantics.npz')
raw=np.load(ROOT/'data/semantic_map.npz')
ix=native['source_index'];good=ix>=0
assert np.array_equal(native['labels'][good],raw['labels'][ix[good]])
assert np.all(native['labels'][~good]==0)
assert np.max(np.linalg.norm(p[good]-raw['points'][ix[good]],axis=1))<=.120001
model_provenance=json.loads((OUT/'model_semantics_provenance.json').read_text())
palette=np.array(model_provenance['palette'],np.uint8)
assert np.array_equal(native['colors'],palette[native['labels']])
native_pcd=(OUT/'dense_model_semantic.pcd').read_bytes().split(b'DATA binary\n',1)[1]
assert len(native_pcd)==17*len(p)
result=dict(passed=True,unique_voxels=len(p),sampled_original_returns_verified=verified,
            annotation_instances=len(annotations),
            checks=['finite XYZ and unchanged sampled original returns',
                    'source scan timestamp equality','unique 4 cm voxels',
                    'ROI bounds','known label vocabulary','unmatched points stay unknown',
                    'all displayed instances have observed point support','no unsupported person/elevator',
                    'dense semantic PCD layout','existing model class IDs preserved',
                    'association distance <= 0.12 m','identical class-to-color mapping'])
(OUT/'validation.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
