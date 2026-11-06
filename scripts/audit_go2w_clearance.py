#!/usr/bin/python3
"""Report measured route spacing, explicitly separating proxies from people."""
import argparse,csv,json,math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
ap=argparse.ArgumentParser();ap.add_argument('run',type=Path);a=ap.parse_args()
rows=list(csv.DictReader((a.run/'odom_physics.csv').open()));xy=np.array([[float(r['x_m']),float(r['y_m'])] for r in rows]);yaw=np.array([float(r['yaw_rad']) for r in rows]);z=np.array([float(r['z_m']) for r in rows])
root=ET.parse(a.run/'physics_scene.xml').getroot();out={}
for name in ['Cylinder37','Cylinder38','Cylinder39','Cylinder41']:
 g=root.find(f".//worldbody/geom[@name='{name}']");p=np.array(list(map(float,g.get('pos').split())));size=list(map(float,g.get('size').split()));delta=p[:2]-xy
 local=np.column_stack([np.cos(yaw)*delta[:,0]+np.sin(yaw)*delta[:,1],-np.sin(yaw)*delta[:,0]+np.cos(yaw)*delta[:,1]])
 gap=np.linalg.norm(np.maximum(np.abs(local)-[.30,.15],0),axis=1)-size[0]
 valid=(z+.20>p[2]-size[1])&(z-.30<p[2]+size[1])
 out[name]=dict(minimum_configured_footprint_gap_m=float(gap[valid].min()) if valid.any() else None)
min_dynamic=math.inf;source=set()
events=a.run/'events.jsonl'
for line in (events.read_text().splitlines() if events.exists() else []):
 try:r=json.loads(line)
 except ValueError:continue
 if r.get('topic')!='world_model_status':continue
 p=r['payload'];source.add(p.get('dynamic_position_source','unknown'))
 for actor in p.get('dynamic_actors',[]):
  d=actor.get('robot_center_distance_m')
  if isinstance(d,(int,float)):min_dynamic=min(min_dynamic,d)
result=dict(static_terrain_obstacles=out,configured_footprint_m=[.60,.30],dynamic_position_sources=sorted(source),minimum_dynamic_proxy_center_distance_m=min_dynamic if math.isfinite(min_dynamic) else None,native_human_clearance_verified=False,notes='Configured 2D footprint spacing with vertical overlap gate. Dynamic values describe MuJoCo proxies, not native UE people. Not a full-body collision guarantee.')
(a.run/'CLEARANCE_QA.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
