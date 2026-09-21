#!/usr/bin/env python3
"""Check the actual latched RViz input messages against source coordinates/colors."""
import json
from pathlib import Path
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray,Marker
from std_msgs.msg import String
from academic_palette import height_colors,paper_height_colors,SEMANTIC

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/academic_upgrade'
rospy.init_node('audit_academic_ros',anonymous=True)
xyz=np.load(ROOT/'outputs/reference_demo/mapping_replay.npz')['points']
labels=np.load(ROOT/'outputs/reference_demo/model_semantics.npz')['labels']
features=np.load(OUT/'utonia_features.npz')['colors']
status=json.loads(rospy.wait_for_message('/reference_map/status',String,timeout=20).data)
assert status['points']==len(xyz), 'Audit requires final map: pause replay and seek to its endpoint first'
checks={}
for name,points,colors in [('height_points',xyz,height_colors(xyz[:,2])),
                          ('paper_height_points',xyz,paper_height_colors(xyz[:,2])),
                          ('model_semantic_points',xyz[labels>0],SEMANTIC[labels[labels>0]]),
                          ('model_context',xyz[labels==0],SEMANTIC[labels[labels==0]]),
                          ('utonia_features',xyz,features)]:
    msg=rospy.wait_for_message('/reference_map/'+name,PointCloud2,timeout=20)
    assert msg.header.frame_id=='lio_map' and not msg.is_bigendian
    offsets={f.name:f.offset for f in msg.fields}
    dtype=np.dtype(dict(names=['x','y','z','rgb'],formats=['<f4','<f4','<f4','<u4'],
                        offsets=[offsets[k] for k in ['x','y','z','rgb']],itemsize=msg.point_step))
    a=np.frombuffer(msg.data,dtype=dtype)
    actual_xyz=np.column_stack([a[k] for k in 'xyz'])
    rgb=colors.astype(np.uint32);expected=(rgb[:,0]<<16)|(rgb[:,1]<<8)|rgb[:,2]
    assert np.array_equal(actual_xyz,points), name+' XYZ mismatch'
    assert np.array_equal(a['rgb'],expected), name+' RGB mismatch'
    checks[name]=dict(points=len(a),frame=msg.header.frame_id,xyz_exact=True,rgb_exact=True)
scene=json.loads((ROOT/'outputs/scene_graph.json').read_text())
graph=rospy.wait_for_message('/reference_map/metric_graph',MarkerArray,timeout=20)
node_markers=[m for m in graph.markers if m.ns=='metric_graph_nodes' and m.action!=Marker.DELETEALL]
edge_markers=[m for m in graph.markers if m.ns=='metric_graph_edges' and m.action!=Marker.DELETEALL]
assert len(node_markers)==len(scene['nodes'])==28
assert len(edge_markers)==len(scene['edges'])==28
checks['graph']=dict(nodes=len(node_markers),edges=len(edge_markers))
(OUT/'live_ros_audit.json').write_text(json.dumps(dict(status='passed',checks=checks),indent=2))
(ROOT/'outputs/fastlio_comparison/live_ros_audit.json').write_text(json.dumps(dict(status='passed',checks=checks),indent=2))
print(json.dumps(checks,indent=2))
