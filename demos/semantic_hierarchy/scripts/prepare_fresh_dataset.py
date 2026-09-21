#!/usr/bin/env python3
"""Make a replay profile from the accepted new map without changing its geometry."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,default=ROOT/'outputs/level_fix/fresh_20260919_02');p.add_argument('--output',type=Path,default=ROOT/'outputs/fresh_semantic_20260919');a=p.parse_args();run=a.run;out=a.output
ref=out/'outputs/reference_demo';ref.mkdir(parents=True,exist_ok=True);(out/'data').mkdir(exist_ok=True)
assert not (ref/'mapping_replay.npz').exists(),'Refuse to overwrite prepared geometry'
original=np.load(run/'structural_calibrated/map_arrays.npz');x=original['xyzi'];roi=((x[:,:3]>[-3,-8,-1.5])&(x[:,:3]<[21,13,9.2])).all(1);source_index=np.flatnonzero(roi).astype(np.int32);x=x[roi];points=x[:,:3].copy()
def keys(p,size=.04):
 q=np.floor(p/size).astype(np.int64)+(1<<20);assert q.min()>=0 and q.max()<(1<<21)
 return (q[:,0]<<42)|(q[:,1]<<21)|q[:,2]
k=keys(points);order=np.argsort(k);sortedkeys=k[order];seen=np.full(len(points),np.inf);raw=np.memmap(run/'structural_calibrated/calibrated_xyzi.float32',mode='r',dtype='<f4').reshape(-1,4);frames=[json.loads(l) for l in (run/'capture/frames.jsonl').read_text().splitlines()]
for f in frames:
 v=raw[f['offset']:f['offset']+f['points'],:3];kk=keys(v);j=np.searchsorted(sortedkeys,kk);ok=j<len(sortedkeys);ok[ok]&=sortedkeys[j[ok]]==kk[ok];ids=order[j[ok]];np.minimum.at(seen,ids,f['stamp'])
assert np.isfinite(seen).all();sort=np.argsort(seen,kind='stable');points=points[sort];seen=seen[sort];source_index=source_index[sort]
np.savez_compressed(ref/'mapping_replay.npz',points=points,first_seen=seen,labels=np.zeros(len(points),np.uint8),evidence=np.zeros(len(points),np.uint8),source_index=source_index,voxel=np.array(.04),scan_stats=np.array([[f['stamp'],int((seen==f['stamp']).sum()),int((seen<=f['stamp']).sum())] for f in frames]))
def pcd(path,q):
 h=f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(q)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(q)}\nDATA binary\n';path.write_bytes(h.encode()+q.astype('<f4').tobytes())
pcd(ref/'dense_measured_map.pcd',points)
_,ids=np.unique(np.floor(points/.10).astype(np.int32),axis=0,return_index=True);pcd(out/'data/graph_geometry.pcd',points[np.sort(ids)])
trajectory=original['poses'][:,:4];np.save(out/'data/lio_trajectory.npy',trajectory)
report=dict(source_run=str(run.resolve()),source_geometry=str((run/'structural_calibrated/office_two_floors_structural_4cm.pcd').resolve()),source_sha256=hashlib.sha256((run/'structural_calibrated/office_two_floors_structural_4cm.pcd').read_bytes()).hexdigest(),unique_measured_points=len(points),full_map_points=len(original['xyzi']),input_scans=len(frames),input_points=len(raw),voxel_m=.04,first_stamp=float(frames[0]['stamp']),last_stamp=float(frames[-1]['stamp']),duration_seconds=float(frames[-1]['stamp']-frames[0]['stamp']),roi=[[-3,-8,-1.5],[21,13,9.2]],graph_input_points=len(ids),coordinate_changes='None: ROI selects points from the accepted structural-calibrated map; its known floor priors remain applicable.',semantic_source='pending new synchronized RGB-D model inference',limitations=['Known floor heights used by input map calibration; not unassisted SLAM.','Visibility replay; final offline semantic overlays are not online inference.'])
(ref/'replay_report.json').write_text(json.dumps(report,indent=2));(out/'profile.json').write_text(json.dumps(dict(title='新采集 · 校正地图',source_recording=run.name,structural_prior=True,floor_heights=[0.,5.51],note='新 RGB-D 模型标注 · 同一校正坐标 · 已知层高先验'),ensure_ascii=False,indent=2));print(json.dumps(report,indent=2))
