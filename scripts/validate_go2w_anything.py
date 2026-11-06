#!/usr/bin/env python3
import json
from pathlib import Path
import numpy as np

def validate(out):
 out=Path(out);rows=[]
 for line in (out/'anything/frames.jsonl').read_text().splitlines():
  try:rows.append(json.loads(line))
  except ValueError:continue
 periods=np.diff([r['finished_at'] for r in rows]);regions={};errors=[]
 for f in (out/'anything').glob('*/predictions.npz'):
  a=np.load(f);r=f.parent.name.rsplit('_',1)[0];entry=regions.setdefault(r,{'snapshots':0,'metric_semantic_clouds':0});entry['snapshots']+=1
  if a['relative_depth'].shape!=a['semantic_labels'].shape or not np.isfinite(a['relative_depth']).all():errors.append(str(f))
  pc=f.parent/'semantic_cloud.npz'
  if pc.exists():
   cloud=np.load(pc);entry['metric_semantic_clouds']+=1
   if len(cloud['points'])!=len(cloud['labels']) or not np.isfinite(cloud['points']).all():errors.append(str(pc))
 for r in ('stairs','ramp','dynamic'):
  if r not in regions or not regions[r]['metric_semantic_clouds']:errors.append('missing '+r)
 result=json.loads((out/'result.json').read_text()) if (out/'result.json').exists() else {}
 q={'model_manifest':json.loads((Path(__file__).resolve().parents[1]/'models/anything/manifest.json').read_text()),'model_frames':len(rows),'median_output_hz':float(1/np.median(periods)) if len(periods) else 0,'regions':regions,'errors':errors,'navigation_acceptance_passed':result.get('acceptance_passed',False),'navigation_stages':result.get('stage_count'),'semantic_accuracy_measured':False,'uses_scene_prior':False,'depth_prediction':'relative_inverse_depth','metric_cloud_range_source':'UE_sensor_depth','metric_cloud_label_source':'GroundingDINO_SAM2'}
 for k in ('depth_ms','dino_ms','sam_ms','total_ms','input_age_seconds','metric_cloud_points','rgb_depth_delta_seconds','mid360_points','mid360_labeled_points','mid360_rgb_delta_seconds'):
  values=[r[k] for r in rows if k in r]
  if values:q[k]={'median':float(np.median(values)),'p90':float(np.percentile(values,90))}
 q['passed']=bool(rows) and not errors and q['navigation_acceptance_passed'];(out/'ANYTHING_QA.json').write_text(json.dumps(q,ensure_ascii=False,indent=2));print(json.dumps(q,ensure_ascii=False,indent=2));return q
if __name__=='__main__':
 import sys
 validate(sys.argv[1])
