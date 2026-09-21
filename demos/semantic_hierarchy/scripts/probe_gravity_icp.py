#!/usr/bin/env python3
"""Sensor-only diagnostic: gyro orientation plus robust 4-DoF scan-to-map ICP."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R,Slerp
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'outputs/level_fix/icp_probe';out.mkdir(exist_ok=True)
s=np.load(ROOT/'outputs/level_fix/raw_scan_probe.npz');imu=np.load(ROOT/'outputs/level_fix/old_imu.npy');t=imu[:,0];initial=imu[t<t[0]+2];acc=initial[:,1:4].mean(0);bias=initial[:,4:7].mean(0)
r=R.align_vectors(np.array([[0,0,1.]]),np.array([acc/np.linalg.norm(acc)]))[0];qs=[r.as_quat()]
for i in range(1,len(imu)):
 r=r*R.from_rotvec((.5*(imu[i,4:7]+imu[i-1,4:7])-bias)*(t[i]-t[i-1]));qs.append(r.as_quat())
interp=Slerp(t,R.from_quat(qs));cloud=np.empty((0,3));pos=np.zeros(3);prev=np.zeros(3);yaw=0;poses=[];scanposes=[]
def voxel(x,size):
 keys=np.floor(x/size).astype('<i4');_,idx=np.unique(np.ascontiguousarray(keys).view('V12'),return_index=True);return x[np.sort(idx)]
for i,stamp in enumerate(s['stamps']):
 raw=s[f'cloud{i}'];rr=np.linalg.norm(raw,axis=1);raw=raw[(rr>.55)&(rr<30)&np.isfinite(raw).all(1)];raw=voxel(raw,.12);base=interp(np.clip(stamp,t[0],t[-1])).apply(raw+np.array([.15568,-.02329,.13366]));previous=pos.copy()
 if len(cloud):
  tree=cKDTree(cloud);pos=pos+np.clip(pos-prev,-.7,.7)
  for iteration in range(12):
   rot=R.from_euler('z',yaw);q=rot.apply(base);x=q+pos;d,idx=tree.query(x,k=5,workers=4);ne=cloud[idx];center=ne.mean(1);nn=ne-center[:,None];cov=np.einsum('nki,nkj->nij',nn,nn);evals,evec=np.linalg.eigh(cov);normal=evec[:,:,0];res=np.einsum('ni,ni->n',normal,x-center)
   ok=(d[:,-1]<.8)&(evals[:,0]<.001)&(evals[:,1]>.00003)&(np.abs(res)<.25)
   if ok.sum()<50:raise RuntimeError(f'Too few constraints at {i}: {ok.sum()}')
   n=normal[ok];r0=res[ok];tangent=np.column_stack((-q[:,1],q[:,0],np.zeros(len(q))));j=np.column_stack((n,np.einsum('ni,ni->n',normal,tangent)[ok]));weight=np.minimum(1,.04/np.maximum(abs(r0),1e-6));h=j.T@(weight[:,None]*j);b=-j.T@(weight*r0);delta=np.linalg.solve(h+np.eye(4)*1e-5,b);pos+=delta[:3];yaw+=delta[3]
   if np.linalg.norm(delta)<.0002:break
 prev=previous;world=R.from_euler('z',yaw).apply(base)+pos;cloud=voxel(np.concatenate((cloud,world)),.10);poses.append([stamp,*pos,yaw]);scanposes.append([stamp,*pos,*(R.from_euler('z',yaw)*interp(np.clip(stamp,t[0],t[-1]))).as_quat()])
 if i%10==0:print(i,len(cloud),np.round(pos,3),flush=True)
np.savez_compressed(out/'probe.npz',points=cloud,poses=np.array(scanposes));print('final',pos,flush=True)
