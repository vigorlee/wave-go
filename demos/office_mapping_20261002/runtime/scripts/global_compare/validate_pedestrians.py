"""Validate actual spawned meshes and movement from UE's own object properties."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import time

R = Path(__file__).resolve().parents[2]
LOG = R / 'matrix/src/UeSim/Linux/zsibot_mujoco_ue/Saved/Logs/zsibot_mujoco_ue.log'

def query(command):
    offset = LOG.stat().st_size
    subprocess.run(['/usr/bin/python3', str(R / 'scripts/global_compare/ue_console.py'), command], check=True)
    with LOG.open('r') as f:
        f.seek(offset)
        return f.read()

def locations():
    s = query('getall CapsuleComponent RelativeLocation')
    matches = re.findall(r'(BP_Crown_Traj_1_C_\d+)\.CollisionCylinder.RelativeLocation = \(X=([-\d.]+),Y=([-\d.]+),Z=([-\d.]+)\)', s)
    return {name: list(map(float, xyz)) for name, *xyz in matches}, s

def progress():
    s = query('getall BP_Human_C Traj_Index')
    return {name:int(index) for name,index in re.findall(r'(BP_Crown_Traj_1_C_\d+)\.Traj_Index = (\d+)', s)}, s

if __name__ == '__main__':
    a = argparse.ArgumentParser();a.add_argument('--output', type=Path, required=True);args = a.parse_args()
    meshes = query('getall SkeletalMeshComponent SkeletalMesh')
    assert meshes.count('SK_Survival_Character.SK_Survival_Character') == 2, meshes
    start_index, start_log = progress()
    samples, logs = [], []
    for delay in (0, .7, 1.2):
        time.sleep(delay)
        positions, raw = locations()
        samples.append(positions);logs.append(raw)
    before, after = samples[0], samples[-1]
    assert all(len(s) == 2 and s.keys() == before.keys() for s in samples), samples
    moved = {k: max(sum((a-b)**2 for a,b in zip(s[k], t[k]))**.5/100 for s in samples for t in samples) for k in before}
    assert all(x > .05 for x in moved.values()), moved
    end_index, end_log = progress()
    assert len(start_index) == 2 and all(end_index.get(k, -1) > v for k,v in start_index.items()), (start_index, end_index)
    report = dict(verified_count=2, actual_mesh='SK_Survival_Character', actual_class='BP_Crown_Traj_1_C', before_cm=before, after_cm=after, samples_cm=samples, observed_displacement_m=moved, start_waypoint_indices=start_index, end_waypoint_indices=end_index, raw_logs=meshes+start_log+''.join(logs)+end_log)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps({k:v for k,v in report.items() if k != 'raw_logs'}))
