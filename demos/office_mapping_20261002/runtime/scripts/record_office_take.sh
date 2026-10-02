#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
TAKE_DIR="$(realpath "$1")"
[[ ! -e "$TAKE_DIR/go2w_four_screen_demo.mp4" ]] || { echo "Recording already exists; choose a fresh run." >&2; exit 1; }
systemctl --user stop go2w-office-physics-view.service go2w-office-record.service >/dev/null 2>&1 || true
systemd-run --user --quiet --collect --unit=go2w-office-physics-view --setenv=DISPLAY=:0 --setenv=XAUTHORITY="$XAUTHORITY" --property="WorkingDirectory=$PWD" "$PWD/.venv-anything/bin/python" scripts/office_physics_view.py
/usr/bin/python3 - <<'PY'
import subprocess,time
for _ in range(50):
 ws=[x.split()[0] for x in subprocess.check_output(['wmctrl','-l'],text=True).splitlines() if 'MuJoCo : ' in x]
 if len(ws)>1:
  subprocess.run(['wmctrl','-ir',ws[-1],'-T','MuJoCo Physics | OfficeWorld live state']);break
 time.sleep(.2)
else:raise RuntimeError('Read-only physics view missing')
PY
systemd-run --user --quiet --collect --unit=go2w-office-record --setenv=DISPLAY=:0 --setenv=XAUTHORITY="$XAUTHORITY" --property="WorkingDirectory=$PWD" /usr/bin/python3 scripts/record_office_four_screen.py "$TAKE_DIR"
for i in {1..30}; do [[ -s "$TAKE_DIR/ffmpeg.pid" ]] && break; sleep 1; done
[[ -s "$TAKE_DIR/ffmpeg.pid" ]]
systemd-run --user --quiet --collect --unit=go2w-course --property=KillSignal=SIGINT --property="WorkingDirectory=$PWD" --property="StandardOutput=append:$TAKE_DIR/course.log" --property="StandardError=append:$TAKE_DIR/course.log" --setenv="OFFICE_TAKE_DIR=$TAKE_DIR" /bin/bash -c 'source /opt/ros/humble/setup.bash; source genisom_roamerx_open/install/setup.bash; export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp; exec /usr/bin/python3 scripts/go2w_office_continuous_course.py --output "$OFFICE_TAKE_DIR/course"'
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
