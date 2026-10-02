#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
TAKE_DIR="$(realpath "$1")"
[[ ! -e "$TAKE_DIR/go2w_four_screen_demo.mp4" ]] || { echo "Recording already exists; choose a fresh run." >&2; exit 1; }
/usr/bin/python3 - "$TAKE_DIR" <<'PY'
from pathlib import Path
import hashlib,json,shutil,sys
out=Path(sys.argv[1])/'run_configuration';out.mkdir(exist_ok=True)
files=['scripts/global_compare/follow_route.py','scripts/global_compare/dense_capture.py','scripts/global_compare/validate_pedestrians.py','scripts/start_office_global_demo.sh','scripts/start_go2w_global_compare_world.sh','config/global_compare/pedestrians.json','config/global_compare/office_demo.rviz','matrix/scene/scene_go2w_global_compare.json','config/office_lio_dense.yaml','matrix/config/config.json','artifacts/global_compare_20261001/plans/routes.json']
hashes={}
for name in files:
 p=Path(name);dest=out/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest);hashes[name]=hashlib.sha256(p.read_bytes()).hexdigest()
hashes['install/bin/go2w_rl_bridge']=hashlib.sha256(Path('install/bin/go2w_rl_bridge').read_bytes()).hexdigest()
(out/'sha256.json').write_text(json.dumps(hashes,indent=2))
PY
systemctl --user stop go2w-manual.service go2w-office-physics-view.service go2w-office-record.service >/dev/null 2>&1 || true
systemd-run --user --quiet --collect --unit=go2w-office-physics-view --setenv=DISPLAY=:0 --setenv=XAUTHORITY="$XAUTHORITY" --property="WorkingDirectory=$PWD" "$PWD/.venv-anything/bin/python" scripts/office_inline_physics_view.py
/usr/bin/python3 - <<'PY'
import subprocess,time
for _ in range(50):
 listing=subprocess.run(['wmctrl','-l'],capture_output=True,text=True)
 if listing.returncode:time.sleep(.2);continue
 ws=[x.split()[0] for x in listing.stdout.splitlines() if 'MuJoCo : ' in x]
 if len(ws)>1:
  subprocess.run(['wmctrl','-ir',ws[-1],'-T','MuJoCo Physics | OfficeWorld live state']);break
 time.sleep(.2)
else:raise RuntimeError('Read-only physics view missing')
PY
systemd-run --user --quiet --collect --unit=go2w-office-record --setenv=DISPLAY=:0 --setenv=XAUTHORITY="$XAUTHORITY" --property="WorkingDirectory=$PWD" /usr/bin/python3 scripts/global_compare/record_four_screen.py "$TAKE_DIR" "${2:-NavFnDijkstra}"
for i in {1..30}; do [[ -s "$TAKE_DIR/ffmpeg.pid" ]] && break; sleep 1; done
[[ -s "$TAKE_DIR/ffmpeg.pid" ]]
# Explicit automatic launch transfers control after the manual process has stopped.
rm -f .run/go2w/manual_control
systemd-run --user --quiet --collect --unit=go2w-course --property=KillSignal=SIGINT --property="WorkingDirectory=$PWD" --property="StandardOutput=append:$TAKE_DIR/course.log" --property="StandardError=append:$TAKE_DIR/course.log" --setenv="OFFICE_TAKE_DIR=$TAKE_DIR" --setenv="OFFICE_PLANNER=${2:-NavFnDijkstra}" /bin/bash -c 'source /opt/ros/humble/setup.bash; source genisom_roamerx_open/install/setup.bash; export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp; exec /usr/bin/python3 scripts/global_compare/follow_route.py --output "$OFFICE_TAKE_DIR/course" --routes artifacts/global_compare_20261001/plans/routes.json --planner "$OFFICE_PLANNER"'
/usr/bin/python3 - "$TAKE_DIR" <<'PY'
import json,time,sys,os,signal,subprocess
from pathlib import Path
p=Path(sys.argv[1]);end=time.monotonic()+630
while time.monotonic()<end:
 try:
  r=json.loads((p/'course/result.json').read_text());print(r['state'],r['completed_stages'],r.get('last_event',{}).get('reason'),flush=True)
  if r['state'] in ('succeeded','failed','canceled','blocked'):break
 except (FileNotFoundError,json.JSONDecodeError):pass
 time.sleep(10)
time.sleep(4)
os.kill(int((p/'ffmpeg.pid').read_text()),signal.SIGINT)
subprocess.run(['systemctl','--user','stop','go2w-live-dense-capture.service'])
PY
