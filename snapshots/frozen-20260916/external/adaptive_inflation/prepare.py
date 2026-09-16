from pathlib import Path
import json,hashlib
import numpy as np
from scipy.spatial import cKDTree
from adaptive_core import read_pcd,voxel_centroids,build_grid,corridor_width
R=Path(__file__).resolve().parent
SOURCES={
 'flycar':('/home/unitree/flycar/src/localization/fast_lio/PCD/ch31-2.pcd',-.15,'Flycar 实测 PCD'),
 'go2w':('/home/unitree/Go2w+cosmos3Nav/wave-go-Wavegov1/demos/Wavegov1/evidence/semantic_v1/semantic_map.pcd',-.27,'Go2-W 语义 PCD')}
meta={}
for key,(path,floor,title) in SOURCES.items():
 print('Reading',key,flush=True);p,l,info=read_pcd(path)
 if key=='flycar':
  display=voxel_centroids(p,.025);labels=np.zeros(len(display),np.uint8)
  # Sparse isolated returns are excluded from the analysis only; visible cloud retained.
  analysis=voxel_centroids(p,.05);dist,_=cKDTree(analysis).query(analysis,k=5);good=dist[:,-1]<.25
  analysis=analysis[good];labs=np.zeros(len(analysis),np.uint8);removed=int((~good).sum())
 else:display=p;labels=l;analysis=p;labs=l;removed=0
 g=build_grid(analysis,labs,floor)
 np.savez_compressed(R/f'{key}_prepared.npz',points=display,labels=labels,analysis_points=analysis,analysis_labels=labs)
 # Select inspection positions with measured ground support. These are not trajectories.
 ground=analysis[np.abs(analysis[:,2]-floor)<.15,:2];ground=ground[::max(1,len(ground)//90)]
 choices=[]
 for pos in ground:
  iyx=np.floor((pos-g['origin'])/.05).astype(int)
  clearance=g['distance'][iyx[1],iyx[0]]
  if not .48<clearance<2.:continue
  for yaw in [0,np.pi/2,np.pi,-np.pi/2]:
   width,rays=corridor_width(g,pos,yaw)
   if width is not None:choices.append(dict(position=pos.tolist(),yaw=float(yaw),width=width))
 choices.sort(key=lambda x:x['width'])
 if not choices:raise ValueError('No supported inspection point')
 narrow=min(choices,key=lambda x:abs(x['width']-1.3));wide=max(choices,key=lambda x:x['width'])
 meta[key]=dict(title=title,source=path,sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),**info,display_points=len(display),analysis_points=len(analysis),isolated_analysis_voxels_removed=removed,floor=floor,height_band=[floor+.18,floor+1.65],display_voxel_m=.025 if key=='flycar' else None,analysis_voxel_m=.05,grid_resolution_m=.05,occupancy_components=g['component_count'],examples={'narrow':narrow,'wide':wide},source_frame='PCD source coordinates; no cross-dataset registration',labels_present=bool(np.any(labels)))
 print(key,meta[key]['display_points'],meta[key]['analysis_points'],meta[key]['examples'],flush=True)
(R/'datasets.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n')
