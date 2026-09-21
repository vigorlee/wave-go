#!/usr/bin/env python3
"""Export recorded returns, retain first measured return in each 4 cm voxel."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from sklearn.linear_model import RANSACRegressor,LinearRegression

def planes(xyz,bands):
    fits={}
    for name,(lo,hi) in bands.items():
        a=xyz[(xyz[:,2]>lo)&(xyz[:,2]<hi)&(xyz[:,0]>-3)&(xyz[:,0]<20)&(xyz[:,1]>-8)&(xyz[:,1]<13)]
        a=a[::max(1,len(a)//80000)]
        if len(a)<500:continue
        model=RANSACRegressor(LinearRegression(),min_samples=3,residual_threshold=.075,max_trials=300,random_state=42).fit(a[:,:2],a[:,2])
        slope=model.estimator_.coef_;norm=np.r_[-slope,1.];norm/=np.linalg.norm(norm)
        inliers=a[model.inlier_mask_]
        fits[name]=dict(slope=slope.tolist(),intercept=float(model.estimator_.intercept_),normal=norm.tolist(),tilt_deg=float(np.degrees(np.arccos(norm[2]))),inliers=len(inliers),tested=len(a),rmse_m=float(np.sqrt(np.mean((model.predict(inliers[:,:2])-inliers[:,2])**2))),height_band=[lo,hi])
    if len(fits)==2:fits['normal_difference_deg']=float(np.degrees(np.arccos(np.clip(np.dot(fits['F0']['normal'],fits['F1']['normal']),-1,1))))
    return fits

def pcd(path,a):
    with path.open('wb') as f:
        f.write(('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n'%(len(a),len(a))).encode());f.write(a.astype('<f4').tobytes())

def main():
    p=argparse.ArgumentParser();p.add_argument('capture',type=Path);p.add_argument('--voxel',type=float,default=.04);a=p.parse_args()
    out=a.capture.parent/'export';out.mkdir(exist_ok=True)
    target=out/'office_two_floors_4cm.pcd'
    if target.exists():raise RuntimeError('Refuse to overwrite an existing exported map')
    raw=np.memmap(a.capture/'registered_xyzi.float32',dtype='<f4',mode='r').reshape(-1,4)
    # Streaming uniqueness in bounded chunks; preserve raw observed geometry.
    selected=np.empty((0,4),dtype='<f4')
    for start in range(0,len(raw),1000000):
        block=np.concatenate((selected,np.asarray(raw[start:start+1000000])))
        keys=np.floor(block[:,:3]/a.voxel).astype('<i4');view=np.ascontiguousarray(keys).view('V12').ravel()
        _,idx=np.unique(view,return_index=True);selected=block[np.sort(idx)]
    pcd(target,selected)
    poses=np.array([json.loads(l) for l in (a.capture/'estimated_poses.jsonl').read_text().splitlines()])
    np.savez_compressed(out/'map_arrays.npz',xyzi=selected,poses=poses)
    report=dict(source=str(a.capture.resolve()),raw_returns=len(raw),pcd_points=len(selected),voxel_m=a.voxel,operation='first measured return per voxel; no plane flattening, no invented points',frame='map',duration_s=float(poses[-1,0]-poses[0,0]),floor_fit=planes(selected[:,:3],{'F0':(-1.4,.5),'F1':(4.3,5.8)}),pcd_sha256=hashlib.sha256(target.read_bytes()).hexdigest())
    (out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()
