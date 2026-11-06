"""Audit recorded progress and motion; report measurements, not assumed success."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import numpy as np

def audit(root):
    result = json.loads((root/'course/result.json').read_text())
    events = [json.loads(s) for s in (root/'course/events.jsonl').read_text().splitlines()]
    tracking = json.loads((root/'course/path_tracking.json').read_text())
    rows = list(csv.DictReader((root/'course/odom.csv').open()))
    t = np.array([float(r['time']) for r in rows])
    xyz = np.array([[float(r[k]) for k in ('x','y','z')] for r in rows])
    completed = [e['stage_index'] for e in events if e['state']=='stage_completed']
    yields = [e for e in events if e['state']=='pedestrian_yield']
    cleared = [e for e in events if e['state']=='pedestrian_clear']
    waits = []
    for e in yields:
        end = next((c['time'] for c in cleared if c['time']>e['time']), t[-1])
        mask = (t>e['time']+1.0)&(t<end)
        pts = xyz[mask]
        waits.append(dict(start=e['time'], end=end, duration_s=end-e['time'], settled_xy_span_m=float(np.linalg.norm(np.ptp(pts[:,:2],axis=0))) if len(pts)>1 else None))
    video = root/'go2w_four_screen_demo.mp4'
    metadata = json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration:stream=codec_name,width,height,r_frame_rate','-of','json',str(video)],text=True))
    decode = subprocess.run(['ffmpeg','-v','error','-i',str(video),'-f','null','-'],capture_output=True,text=True)
    report = dict(planner=result['planner'], navigation_passed=result['passed'], completed_stages=completed,
                  all_12_in_order=completed==list(range(12)),
                  duration_s=events[-1]['time']-events[0]['time'], final_pose=rows[-1],
                  measured_travel_m=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()),
                  maximum_adjacent_odom_jump_m=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).max()),
                  yield_count=len(yields), clear_count=len(cleared), waiting_intervals=waits,
                  cross_track_median_m=float(np.median([r['cross_track_m'] for r in tracking])),
                  cross_track_p95_m=float(np.quantile([r['cross_track_m'] for r in tracking],.95)),
                  minimum_measured_candidate_center_distance_m=result.get('minimum_candidate_center_distance_m'),
                  video_metadata=metadata, video_full_decode_ok=decode.returncode==0 and not decode.stderr.strip(),
                  video_decode_messages=decode.stderr,
                  limitation='Obstacle candidates are geometric LiDAR clusters. Center distance is not a calibrated mesh-to-mesh clearance.')
    alignment=root/'frame_alignment.json'
    trajectory=root/'capture/lio_trajectory.jsonl'
    if alignment.exists() and trajectory.exists():
        lio=np.array([[r[k] for k in ('stamp','x','y','z')] for r in (json.loads(s) for s in trajectory.read_text().splitlines())])
        lio=lio[(lio[:,0]>=t[0])&(lio[:,0]<=t[-1])]
        if len(lio):
            tr=np.linalg.inv(np.asarray(json.loads(alignment.read_text())['matrix']))
            estimated=lio[:,1:]@tr[:3,:3].T+tr[:3,3]
            reference=np.column_stack([np.interp(lio[:,0],t,xyz[:,i]) for i in range(3)])
            error=np.linalg.norm(estimated-reference,axis=1)
            report['lio_position_check']=dict(samples=len(lio),median_m=float(np.median(error)),p95_m=float(np.quantile(error,.95)),final_m=float(error[-1]),method='Initial rigid alignment only; interpolate simulator odometry at LIO timestamps. Includes timestamp/latency effects; not a surveyed map accuracy evaluation.')
    (root/'verification.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('run',type=Path);audit(a.parse_args().run)
