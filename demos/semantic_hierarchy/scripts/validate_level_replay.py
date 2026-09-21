#!/usr/bin/env python3
"""Validate LIO against held-out simulator poses; reference is never estimator input."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R,Slerp
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();root=Path(__file__).resolve().parents[1]
g=np.load(root/'outputs/level_fix/old_trajectory_comparison.npz')['sim_reference'];x=np.array([json.loads(l) for l in (a.run/'capture/estimated_poses.jsonl').read_text().splitlines()]);x=x[(x[:,0]>=g[0,0])&(x[:,0]<=g[-1,0])];s=Slerp(g[:,0],R.from_quat(g[:,4:]));offset=R.from_quat(x[0,4:])*s(x[0,0]).inv();e=(R.from_quat(x[:,4:]).inv()*offset*s(x[:,0])).as_euler('xyz',degrees=True);ref_gain=np.interp(x[-1,0],g[:,0],g[:,3])-np.interp(x[0,0],g[:,0],g[:,3]);gain=x[-1,3]-x[0,3]
r=dict(duration_s=float(x[-1,0]-x[0,0]),estimated_height_gain_m=float(gain),reference_height_gain_m=float(ref_gain),height_gain_error_m=float(gain-ref_gain),final_attitude_error_deg=e[-1].tolist(),reference_usage='evaluation only; no simulated pose/orientation enters LIO or its independent gyro reference')
(a.run/'trajectory_validation.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
