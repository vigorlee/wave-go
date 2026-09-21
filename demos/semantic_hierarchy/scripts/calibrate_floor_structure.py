#!/usr/bin/env python3
"""Rebuild measured scans with gyro leveling and explicit scene floor-height priors.
This is structural calibration, not an independent SLAM accuracy result.
No simulator pose, orientation or trajectory is read. Every scan is transformed
rigidly; no point is projected onto a plane and no synthetic point is created.
"""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R,Slerp
from scipy.ndimage import median_filter,gaussian_filter1d
from export_level_map import pcd,planes
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();run=a.run
assert json.loads((run/'course/result.json').read_text())['passed'],'Require a completed route'
out=run/'structural_calibrated';out.mkdir(exist_ok=True);target=out/'office_two_floors_structural_4cm.pcd'
assert not target.exists(),'Never overwrite a calibrated result'
prior=json.loads((ROOT/'config/office_floor_priors.json').read_text());(out/'floor_priors.json').write_text(json.dumps(prior,indent=2))
imu=np.load(run/'imu_signals.npy');t=imu[:,0];initial=imu[t<t[0]+2];acc=initial[:,1:4].mean(0);bias=initial[:,4:7].mean(0)
r=R.align_vectors(np.array([[0,0,1.]]),np.array([acc/np.linalg.norm(acc)]))[0];qs=[r.as_quat()]
for i in range(1,len(imu)):
 r=r*R.from_rotvec((.5*(imu[i,4:7]+imu[i-1,4:7])-bias)*(t[i]-t[i-1]));qs.append(r.as_quat())
gi=Slerp(t,R.from_quat(qs));poses=np.array([json.loads(l) for l in (run/'capture/estimated_poses.jsonl').read_text().splitlines()]);li=Slerp(poses[:,0],R.from_quat(poses[:,4:]));frames=[json.loads(l) for l in (run/'capture/frames.jsonl').read_text().splitlines()];st=np.array([f['stamp'] for f in frames]);assert st[0]>=t[0] and st[-1]<=t[-1]
lrot=li(st);grot=gi(st);yawdiff=lrot.as_euler('xyz')[:,2]-grot.as_euler('xyz')[:,2];nrot=R.from_rotvec(np.column_stack((np.zeros(len(st)),np.zeros(len(st)),yawdiff)))*grot;delta=nrot*lrot.inv();lp=np.column_stack([np.interp(st,poses[:,0],poses[:,i]) for i in range(1,4)])
raw=np.memmap(run/'capture/registered_xyzi.float32',dtype='<f4',mode='r').reshape(-1,4)
events=[json.loads(l) for l in (run/'course/events.jsonl').read_text().splitlines()];events=[e for e in events if e['state']=='stage_starting'];et=np.array([e['time'] for e in events]);obs=[];evidence=[]
for i,f in enumerate(frames):
 j=np.searchsorted(et,st[i],side='right')-1;stage=events[j]['stage_id'] if j>=0 else 'stair_alignment'
 if stage not in prior['calibration_stages']:continue
 pts=np.asarray(raw[f['offset']:f['offset']+f['points'],:3]);rel=delta[i].apply(pts-lp[i]);z=rel[:,2];hist,edges=np.histogram(z,bins=np.arange(-.72,-.30,.008));k=hist.argmax();center=.5*(edges[k]+edges[k+1]);mask=abs(z-center)<.025
 if mask.sum()<100:continue
 xy=rel[mask,:2]
 if np.linalg.eigvalsh(np.cov(xy.T))[0]<.10:continue
 floor_rel=float(np.median(z[mask]));target_z=prior['calibration_stages'][stage];dz=target_z-floor_rel-lp[i,2];obs.append([st[i],dz]);evidence.append(dict(frame=i,stamp=st[i],stage=stage,target_floor_m=target_z,relative_plane_z=floor_rel,points=int(mask.sum()),vertical_correction_m=dz))
obs=np.array(obs);assert len(obs)>100 and obs[-1,0]>st[-1]-2,'Insufficient floor calibration observations'
# Smooth each contiguous floor segment, then interpolate correction through the
# stair/ramp transitions; preserve measured within-scan geometry everywhere.
values=obs[:,1].copy();splits=np.r_[0,np.where(np.diff(obs[:,0])>1.)[0]+1,len(obs)]
for start,end in zip(splits[:-1],splits[1:]):values[start:end]=gaussian_filter1d(median_filter(values[start:end],size=min(9,2*((end-start)//2)+1),mode='nearest'),sigma=2,mode='nearest')
zfix=np.interp(st,obs[:,0],values);newpos=lp.copy();newpos[:,2]+=zfix
np.savez_compressed(out/'calibration_trajectory.npz',stamp=st,original_position=lp,corrected_position=newpos,original_quaternion=lrot.as_quat(),corrected_quaternion=nrot.as_quat(),vertical_correction=zfix,observations=obs)
(out/'plane_observations.json').write_text(json.dumps(evidence,indent=2))
# Preserve all calibrated returns as a separate source, plus a 4 cm PCD export.
selected=np.empty((0,4),dtype='<f4');pending=[];count=0
with (out/'calibrated_xyzi.float32').open('wb') as sink:
 for i,f in enumerate(frames):
  pts=np.array(raw[f['offset']:f['offset']+f['points']],copy=True);pts[:,:3]=delta[i].apply(pts[:,:3]-lp[i])+newpos[i];sink.write(pts.astype('<f4').tobytes());pending.append(pts);count+=len(pts)
  if count>900000 or i==len(frames)-1:
   block=np.concatenate([selected,*pending]);keys=np.floor(block[:,:3]/.04).astype('<i4');_,idx=np.unique(np.ascontiguousarray(keys).view('V12'),return_index=True);selected=block[np.sort(idx)];pending=[];count=0
pcd(target,selected);np.savez_compressed(out/'map_arrays.npz',xyzi=selected,poses=np.column_stack((st,newpos,nrot.as_quat())))
r=dict(source_run=str(run.resolve()),method='Gyro-leveled measured scans with observed-floor vertical constraints and known scene height priors',not_unassisted_slam=True,simulated_pose_used=False,ground_floor_prior_m=prior['floor0_m'],landing_prior_m=prior['landing_m'],upper_floor_prior_m=prior['floor1_m'],raw_returns=len(raw),pcd_points=len(selected),voxel_m=.04,calibration_observations=len(obs),correction_range_m=[float(zfix.min()),float(zfix.max())],floor_fit=planes(selected[:,:3],{'F0':(-.18,.18),'F1':(5.3,5.72)}),sha256=hashlib.sha256(target.read_bytes()).hexdigest(),geometry='Each scan transformed rigidly using a corrected sensor pose. No points projected onto planes; stair/ramp surfaces retained. LIO and original maps remain separate.')
(out/'report.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
