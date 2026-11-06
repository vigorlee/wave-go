#!/usr/bin/env bash
# Open four live experiment windows and start the complete automatic course.
set -euo pipefail
cd "$(dirname "$0")/.."
source scripts/office_ros_environment.sh
PLANNER="${1:-SmacAStar}"
START_STAGE="${OFFICE_START_STAGE:-0}"
[[ "$START_STAGE" =~ ^([0-9]|1[0-4])$ ]] || { echo 'OFFICE_START_STAGE must be 0 through 14.' >&2; exit 2; }
case "$PLANNER" in
  NavFnDijkstra|SmacAStar) ;;
  *) echo 'Planner must be NavFnDijkstra or SmacAStar' >&2; exit 2 ;;
esac
RUN_NAME="office_auto_$(date +%Y%m%d_%H%M%S)"
TAKE_DIR="$PWD/artifacts/$RUN_NAME"
systemctl --user stop go2w-router-watchdog.service go2w-manual.service go2w-recovery.service go2w-lio-rviz-20261002.service go2w-lio-review-20261002.service >/dev/null 2>&1 || true
bash scripts/start_office_global_demo.sh "$RUN_NAME"
if [[ -n "${OFFICE_PREPARE_GATE:-}" ]]; then
  printf 'Waiting for read-only scene inspection and route validation: %s\n' "$TAKE_DIR"
  for ((i=0;i<120;i++)); do
    [[ "$(cat "$OFFICE_PREPARE_GATE" 2>/dev/null || true)" == "$TAKE_DIR" ]] && break
    sleep 1
  done
  [[ "$(cat "$OFFICE_PREPARE_GATE" 2>/dev/null || true)" == "$TAKE_DIR" ]] || { echo 'Scene preparation timed out; robot remains stopped.' >&2; exit 1; }
fi
mkdir -p "$TAKE_DIR/screenshots"
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
systemd-run --user --quiet --collect --unit=go2w-office-physics-view \
  --setenv=DISPLAY=:0 --setenv=XAUTHORITY="$XAUTHORITY" \
  --property="WorkingDirectory=$PWD" \
  "$PWD/.venv-anything/bin/python" scripts/office_inline_physics_view.py
/usr/bin/python3 - <<'PY'
import ctypes, subprocess, time
for _ in range(50):
    rows = subprocess.check_output(['wmctrl', '-l'], text=True).splitlines()
    windows = [row.split()[0] for row in rows if 'MuJoCo : ' in row]
    if len(windows) > 1:
        subprocess.run(['wmctrl', '-ir', windows[-1], '-T', 'MuJoCo Physics | OfficeWorld live state'], check=True)
        subprocess.run(['wmctrl', '-ir', windows[0], '-b', 'remove,above'], check=True)
        # Keep the actual simulator alive behind the read-only experiment view.
        # Closing this duplicate window would terminate physics and UE together.
        subprocess.run(['wmctrl', '-ir', windows[0], '-T', 'MuJoCo simulator backend (keep running)'], check=True)
        xlib = ctypes.CDLL('libX11.so.6')
        xlib.XOpenDisplay.argtypes=[ctypes.c_char_p];xlib.XOpenDisplay.restype=ctypes.c_void_p
        xlib.XIconifyWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int]
        xlib.XFlush.argtypes=[ctypes.c_void_p];xlib.XCloseDisplay.argtypes=[ctypes.c_void_p]
        display=xlib.XOpenDisplay(b':0')
        if not display:raise SystemExit('X11 display unavailable')
        xlib.XIconifyWindow(display,int(windows[0],16),0)
        xlib.XFlush(display);xlib.XCloseDisplay(display)
        break
    time.sleep(.2)
else:
    raise SystemExit('Live physics view did not open')
PY
ROAMERX_RVIZ_CONFIG="$PWD/config/global_compare/office_demo.rviz" bash scripts/show_go2w_four_windows.sh

# Wait until the newly initialized LiDAR map is actually publishing.
/usr/bin/python3 - "$TAKE_DIR" <<'PY'
import json, sys, time
from pathlib import Path
p = Path(sys.argv[1])
for _ in range(60):
    try:
        status = json.loads((p / 'capture/live_status.json').read_text())
        if status['scans'] >= 5 and status['display_points'] > 0 and (p / 'frame_alignment.json').exists():
            break
    except (FileNotFoundError, json.JSONDecodeError):
        pass
    time.sleep(.5)
else:
    raise SystemExit('Live point cloud did not become ready; automatic navigation was not started')
PY

# The stopped manual panel must relinquish its persistent control latch.
rm -f .run/go2w/manual_control
if [[ "${OFFICE_RECORD:-0}" == 1 ]]; then
  systemd-run --user --quiet --collect --unit=go2w-office-record \
    --property=KillMode=mixed --property=KillSignal=SIGINT --property=TimeoutStopSec=20 \
    --property="WorkingDirectory=$PWD" --setenv=DISPLAY=:0 --setenv="XAUTHORITY=$XAUTHORITY" \
    /usr/bin/python3 scripts/global_compare/record_window_demo.py --output-dir "$TAKE_DIR/recordings" --duration 600
  sleep 2
fi
systemd-run --user --quiet --collect --unit=go2w-course \
  --property=KillMode=control-group --property=KillSignal=SIGINT \
  --property="WorkingDirectory=$PWD" \
  --property="StandardOutput=append:$TAKE_DIR/course.log" \
  --property="StandardError=append:$TAKE_DIR/course.log" \
  --setenv="OFFICE_TAKE_DIR=$TAKE_DIR" --setenv="OFFICE_PLANNER=$PLANNER" --setenv="OFFICE_START_STAGE=$START_STAGE" \
  --setenv="ZENOH_SESSION_CONFIG_URI=$ZENOH_SESSION_CONFIG_URI" \
  /bin/bash -c 'source /opt/ros/humble/setup.bash; source genisom_roamerx_open/install/setup.bash; export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp; exec /usr/bin/python3 scripts/global_compare/follow_route.py --output "$OFFICE_TAKE_DIR/course" --routes config/global_compare/door_routes.json --planner "$OFFICE_PLANNER" --start-stage "$OFFICE_START_STAGE"'
systemd-run --user --quiet --collect --unit=go2w-live-audit \
  --property="WorkingDirectory=$PWD" --setenv="ZENOH_SESSION_CONFIG_URI=$ZENOH_SESSION_CONFIG_URI" \
  --setenv="OFFICE_TAKE_DIR=$TAKE_DIR" \
  /bin/bash -c 'source /opt/ros/humble/setup.bash; source genisom_roamerx_open/install/setup.bash; export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp; exec /usr/bin/python3 scripts/global_compare/audit_live.py --output "$OFFICE_TAKE_DIR" --duration 360'
systemd-run --user --quiet --collect --unit=go2w-auto-handoff \
  --property="WorkingDirectory=$PWD" --property="StandardOutput=append:$TAKE_DIR/handoff.log" \
  --property="StandardError=append:$TAKE_DIR/handoff.log" \
  /bin/bash "$PWD/scripts/finish_office_auto.sh" "$TAKE_DIR"
printf '\nAutomatic navigation started: %s, first stage %s\nRun directory: %s\n' "$PLANNER" "$START_STAGE" "$TAKE_DIR"
printf 'Save a four-window PNG: /usr/bin/python3 %q\n' "$PWD/scripts/capture_office_four_windows.py"
