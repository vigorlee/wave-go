#!/usr/bin/env python3
"""Plane estimates from initial and final observed floor patches, not map-wide bands."""
import argparse,json
from pathlib import Path
import numpy as np
from sklearn.linear_model import RANSACRegressor,LinearRegression

def fit_floor(raw,frames,poses,window):
    selected=[]
    for frame in frames:
        t=frame['stamp']
        if not window[0]<=t<=window[1]:continue
        pose=np.array([np.interp(t,poses[:,0],poses[:,i]) for i in range(1,4)])
        pts=raw[frame['offset']:frame['offset']+frame['points'],:3]
        d=pts-pose
        keep=(np.linalg.norm(d[:,:2],axis=1)<3)&(d[:,2]<-.15)&(d[:,2]>-1.)
        selected.append(pts[keep])
    x=np.concatenate(selected);x=x[::max(1,len(x)//100000)]
    def valid(model,xx,yy):return np.linalg.norm(model.coef_)<np.tan(np.deg2rad(12))
    m=RANSACRegressor(LinearRegression(),min_samples=3,residual_threshold=.04,max_trials=500,random_state=42,is_model_valid=valid).fit(x[:,:2],x[:,2]);n=np.r_[-m.estimator_.coef_.astype(float),1];n/=np.linalg.norm(n);q=x[m.inlier_mask_]
    return dict(normal=n.tolist(),tilt_deg=float(np.degrees(np.arccos(n[2]))),intercept=float(m.estimator_.intercept_),slope=m.estimator_.coef_.tolist(),inliers=len(q),sampled=len(x),rmse_m=float(np.sqrt(np.mean((m.predict(q[:,:2])-q[:,2])**2))),centroid=np.mean(q,axis=0).tolist(),stamp_window=window)

def main():
 p=argparse.ArgumentParser();p.add_argument('capture',type=Path);a=p.parse_args();raw=np.memmap(a.capture/'registered_xyzi.float32',dtype='<f4',mode='r').reshape(-1,4);frames=[json.loads(l) for l in (a.capture/'frames.jsonl').read_text().splitlines()];poses=np.array([json.loads(l) for l in (a.capture/'estimated_poses.jsonl').read_text().splitlines()]);start=frames[0]['stamp'];end=frames[-1]['stamp'];f0=fit_floor(raw,frames,poses,[start,start+3]);f1=fit_floor(raw,frames,poses,[end-3,end]);r=dict(first_floor=f0,last_floor=f1,normal_difference_deg=float(np.degrees(np.arccos(np.clip(np.dot(f0['normal'],f1['normal']),-1,1)))),estimated_robot_height_gain_m=float(poses[-1,3]-poses[0,3]),note='Local floor patches within 3 m of sensor; no geometric corrections applied. Last patch is only second floor if route completed.')
 (a.capture.parent/'floor_patch_audit.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
if __name__=='__main__':main()
