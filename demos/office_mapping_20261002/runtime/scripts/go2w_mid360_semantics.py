"""Attach learned labels to measured MID360 points without changing geometry."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

def sensor_world(xyz,sensor,pose):
    r=sensor['rotation'];p=sensor['position']
    xyz=Rotation.from_euler('xyz',[r['roll'],-r['pitch'],-r['yaw']],degrees=True).apply(xyz)
    xyz+=np.array([p['x'],-p['y'],p['z']])*.01
    q=pose.orientation;t=pose.position
    return (Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(xyz)+[t.x,t.y,t.z]).astype(np.float32)

def associate_labels(measured_xyz,semantic_xyz,semantic_labels,radius=.12):
    labels=np.zeros(len(measured_xyz),np.uint8)
    if len(semantic_xyz):
        distance,index=cKDTree(semantic_xyz).query(measured_xyz,distance_upper_bound=radius)
        matched=np.isfinite(distance)
        labels[matched]=semantic_labels[index[matched]]
    return labels
