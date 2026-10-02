#!/usr/bin/python3
"""Validate saved sensor evidence separately from navigation acceptance."""
import json,argparse
from pathlib import Path
import numpy as np

def main():
 ap=argparse.ArgumentParser();ap.add_argument('output',type=Path);args=ap.parse_args();out=args.output
 if (out/'anything/frames.jsonl').exists():
  from validate_go2w_anything import validate
  validate(out);return
 p=out/'pointcloud';report={'source':'UE_depth_backprojection_no_interpolation','semantic_source':'geometric_candidates_and_temporal_tracking','uses_scene_prior':False}
 for name in ('dense_status','perception'):
  rows=[]
  for s in (p/(name+'.jsonl')).read_text().splitlines():
   try:rows.append(json.loads(s)['payload'])
   except ValueError:continue
  stats={'samples':len(rows)}
  for k in ('observed_hz','points','display_points','processing_ms','odom_delta_seconds'):
   a=[r[k] for r in rows if k in r and r[k]>0]
   if a:stats[k]={'p10':float(np.percentile(a,10)),'median':float(np.median(a)),'p90':float(np.percentile(a,90))}
  if name=='perception':stats['moving_track_samples']=sum(any(o['label']=='moving cluster' for o in r['objects']) for r in rows)
  report[name]=stats
 regions={};errors=[]
 for f in p.glob('*/metadata.json'):
  m=json.loads(f.read_text());region=m['region'];entry=regions.setdefault(region,{'snapshots':0,'dense_snapshots':0,'semantic_snapshots':0});entry['snapshots']+=1
  for name,key in [('dense_depth','dense_snapshots'),('algorithm','semantic_snapshots')]:
   q=f.parent/(name+'.npz')
   if q.exists():
    a=np.load(q);pts=a['points'];entry[key]+=1
    if not len(pts) or not np.isfinite(pts).all():errors.append(str(q))
    if name=='algorithm' and len(a['labels'])!=len(pts):errors.append(str(q)+': label mismatch')
 for r in ['stairs_up','stairs_down','flat_slalom','ramp_up','ramp_down','dynamic_crossing']:
  if r not in regions or not regions[r]['dense_snapshots'] or not regions[r]['semantic_snapshots']:errors.append('missing '+r)
 report['regions']=regions;report['errors']=errors;report['sensor_evidence_passed']=not errors
 result=json.loads((out/'result.json').read_text()) if (out/'result.json').exists() else {}
 report['navigation']={k:result.get(k) for k in ['result','reason','acceptance_passed','stage_count','duration_sec','recorded_travel_distance_m']}
 report['overall_passed']=report['sensor_evidence_passed'] and result.get('acceptance_passed',False)
 (out/'POINTCLOUD_QA.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
