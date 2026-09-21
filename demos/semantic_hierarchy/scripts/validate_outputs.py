#!/usr/bin/env python3
"""Check data preservation, graph references, and conservative map export."""
import json
from pathlib import Path
import numpy as np
from PIL import Image
import spark_dsg
from build_scene import pcd_xyz

ROOT=Path(__file__).resolve().parents[1]
out=ROOT/'outputs'
report=json.loads((out/'report.json').read_text())
scene=json.loads((out/'scene_graph.json').read_text())
cloud=np.load(out/'semantic_cloud.npz')
source=pcd_xyz(Path(report['source_geometry']))
assert np.array_equal(cloud['points'],source), 'Measured XYZ must remain unchanged'
assert all(len(cloud[k])==len(source) for k in ['labels','original_labels','colors','evidence','roi'])
assert set(np.unique(cloud['labels'])).issubset(range(7))
assert set(np.unique(cloud['evidence'])).issubset(range(4))
nodes={n['id']:n for n in scene['nodes']}
assert len(nodes)==len(scene['nodes'])
assert all(e['source'] in nodes and e['target'] in nodes for e in scene['edges'])
assert not any(n['kind'] in ['person','elevator'] for n in nodes.values())
floors=sorted(n['z'] for n in nodes.values() if n['kind']=='floor')
assert len(floors)==2 and -.8<floors[0]<-.2 and 4.1<floors[1]<5.1
assert not any(z>6 for z in floors), 'Roof must not become a floor'
assert any(e['relation']=='stairs_candidate' for e in scene['edges'])
sg=spark_dsg.DynamicSceneGraph.load(out/'spark_dsg.json')
assert sg.num_nodes()==len(nodes) and sg.num_edges()==len(scene['edges'])
for p in (out/'floor_maps').glob('*.pgm'):
    grid=np.array(Image.open(p))
    assert set(np.unique(grid))=={0,205,254}, 'Observed/unknown distinction lost'
    assert (grid==205).mean()>.2, 'Unobserved space must not be fabricated as free'
pcd=(out/'semantic_map.pcd').read_bytes().split(b'DATA binary\n',1)
assert len(pcd[1])==18*len(source)
result={'passed':True,'checks':['measured XYZ unchanged','label/evidence arrays aligned',
                              'valid graph node/edge references','no unsupported person/elevator',
                              'two supported floor candidates, no roof','stairs candidate edge',
                              'Spark-DSG round trip','Nav2 unknown cells retained','PCD binary layout'],
        'nodes':len(nodes),'edges':len(scene['edges']),'points':len(source)}
(out/'validation.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
