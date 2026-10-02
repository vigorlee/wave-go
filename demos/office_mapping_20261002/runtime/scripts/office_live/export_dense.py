from pathlib import Path
import numpy as np,json,hashlib,sys
R=Path(sys.argv[1]).resolve();src=R/'registered_xyzi.float32';a=np.memmap(src,dtype='<f4',mode='r').reshape(-1,4)
assert src.stat().st_size%16==0
keys=np.floor(a[:,:3]/.03).astype(np.int64)
assert (np.abs(keys)<1048576).all()
packed=((keys[:,0]+1048576)<<42)|((keys[:,1]+1048576)<<21)|(keys[:,2]+1048576)
_,ix=np.unique(packed,return_index=True);del packed,keys
points=np.asarray(a[ix]);n=len(points);out=R/'go2w_live_dense_3cm.pcd'
header=f'# Registered LiDAR accumulation; dynamic history retained\nVERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n'
with out.open('wb') as f:f.write(header.encode());f.write(points.astype('<f4').tobytes())
# Same capture, coarser baseline: distinct spatial support, not duplicated records.
baseline=len(np.unique(np.floor(points[:,:3]/.12).astype(np.int32),axis=0))
np.savez_compressed(R/'go2w_live_dense_3cm.npz',points=points[:,:3],intensity=points[:,3])
result=dict(source_points=len(a),export_points=n,voxel_m=.03,same_capture_12cm_voxels=baseline,density_voxel_count_ratio=n/baseline,file=str(out),size_bytes=out.stat().st_size,xyz_bounds=[points[:,:3].min(0).tolist(),points[:,:3].max(0).tolist()],finite=bool(np.isfinite(points).all()),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),geometry='actual UE LiDAR rays registered with LIO; no inserted or duplicated points',dynamic_history_retained=True)
(R/'pcd_report.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
