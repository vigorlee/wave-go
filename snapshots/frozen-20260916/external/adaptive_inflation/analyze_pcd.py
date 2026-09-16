"""Batch entry point for applying adaptive clearance to a PCD without ROS."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from adaptive_core import read_pcd,voxel_centroids,build_grid,analyze
ap=argparse.ArgumentParser();ap.add_argument('pcd',type=Path);ap.add_argument('--floor',type=float,required=True);ap.add_argument('--position',type=float,nargs=2,required=True);ap.add_argument('--yaw',type=float,default=0,help='radians');ap.add_argument('--speed',type=float,default=.2);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
p,lab,info=read_pcd(args.pcd)
if not np.any(lab):p=voxel_centroids(p,.05);lab=np.zeros(len(p),np.uint8)
g=build_grid(p,lab,args.floor);metrics,derived=analyze(g,args.position,args.yaw,args.speed)
args.output.mkdir(parents=True,exist_ok=True)
report=dict(source=str(args.pcd.resolve()),sha256=hashlib.sha256(args.pcd.read_bytes()).hexdigest(),**info,**metrics)
(args.output/'analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
np.savez_compressed(args.output/'costmap.npz',**derived,occupancy=g['occupancy'],observed=g['observed'],distance=g['distance'],origin=g['origin'],resolution=g['res'])
# Export only the derived inflation projection in a separate, explicitly named PCD.
y,x=np.nonzero(derived['inflated']);a=np.zeros(len(x),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('cost','<u4')]);a['x']=g['origin'][0]+(x+.5)*g['res'];a['y']=g['origin'][1]+(y+.5)*g['res'];a['z']=args.floor+.04;a['cost']=100
header=f'VERSION .7\nFIELDS x y z cost\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH {len(a)}\nHEIGHT 1\nPOINTS {len(a)}\nDATA binary\n'
out=args.output/'derived_inflation_projection.pcd'
if out.resolve()==args.pcd.resolve():raise ValueError('Output would overwrite source')
with out.open('wb') as f:f.write(header.encode());f.write(a.tobytes())
print(json.dumps({'radius':metrics['radius'],'width':metrics['corridor_width'],'output':str(args.output.resolve())}))
