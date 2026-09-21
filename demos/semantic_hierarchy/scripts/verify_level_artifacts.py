#!/usr/bin/env python3
import json,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];run=ROOT/'outputs/level_fix/fresh_20260919_02';backup=ROOT/'outputs/backups/20260919_195139_before_level_fix'
entries=json.loads((backup/'manifest.json').read_text());checks=[]
for e in entries:
 for name in ['source','backup']:
  p=Path(e[name]);checks.append(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),matches=hashlib.sha256(p.read_bytes()).hexdigest()==e['sha256']))
assert all(x['matches'] for x in checks)
original=np.memmap(run/'capture/registered_xyzi.float32',dtype='<f4',mode='r').reshape(-1,4);corrected=np.memmap(run/'structural_calibrated/calibrated_xyzi.float32',dtype='<f4',mode='r').reshape(-1,4);assert original.shape==corrected.shape
frames=[json.loads(l) for l in (run/'capture/frames.jsonl').read_text().splitlines()];maxerr=0
for f in frames[::113]:
 idx=np.arange(f['offset'],f['offset']+min(f['points'],500),5);a=original[idx,:3];b=corrected[idx,:3];err=np.max(abs(np.linalg.norm(a[1:]-a[:-1],axis=1)-np.linalg.norm(b[1:]-b[:-1],axis=1)));maxerr=max(maxerr,float(err));assert np.array_equal(original[idx,3],corrected[idx,3])
assert maxerr<2e-5
pcds=[]
for p in [run/'export/office_two_floors_4cm.pcd',run/'structural_calibrated/office_two_floors_structural_4cm.pcd']:
 with p.open('rb') as f:
  header=[]
  while True:
   line=f.readline();header.append(line.decode().strip())
   if line.startswith(b'DATA '):break
  offset=f.tell();n=int(next(x for x in header if x.startswith('POINTS ')).split()[1]);assert p.stat().st_size-offset==n*16
  points=np.memmap(p,offset=offset,dtype='<f4',mode='r').reshape(n,4);assert np.isfinite(points).all()
 pcds.append(dict(path=str(p),points=n,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
r=dict(passed=True,source_and_backup_checks=checks,source_files_preserved=len(entries),new_route=json.loads((run/'course/result.json').read_text()),raw_returns=len(original),rigid_scan_distance_max_error_m=maxerr,intensity_preserved=True,pcds=pcds)
(run/'artifact_validation.json').write_text(json.dumps(r,indent=2));print('PASS:',len(entries),'original files unchanged; rigid scan max error',maxerr,'m; route',r['new_route']['completed_stages'])
