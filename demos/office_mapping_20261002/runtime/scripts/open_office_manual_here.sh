#!/usr/bin/env bash
# Hand over the running OfficeWorld simulation without resetting its pose or views.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
source scripts/office_ros_environment.sh
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority

if [[ "${1:-}" == --dry-run ]]; then
  printf 'Preserve current OfficeWorld pose and four views.\n'
  printf 'Stop only go2w-course and the previous go2w-manual, then open compact WASD panel.\n'
  printf 'ROS domain: 89; configuration: %s\n' "$ZENOH_SESSION_CONFIG_URI"
  printf 'Entry point: %s\n' "$ROOT_DIR/scripts/start_go2w_manual.sh"
  exit 0
fi
if [[ $# -gt 0 ]]; then echo "Usage: $0 [--dry-run]" >&2; exit 2; fi
if [[ "$(cat .run/go2w/manual_context 2>/dev/null || true)" != global_compare ]]; then
  echo 'No current OfficeWorld global demo; start start_office_global_auto.sh first.' >&2
  exit 1
fi
if ! systemctl --user is-active --quiet go2w-navigation.service; then
  echo 'OfficeWorld simulation is not running; no services or robot state were changed.' >&2
  exit 1
fi
if ! /usr/bin/ss -H -ltn '( sport = :7759 )' | /usr/bin/grep -q .; then
  echo 'OfficeWorld private ROS router is unavailable; no robot state was changed.' >&2
  exit 1
fi

# The bridge holds zero immediately, including while the old route exits.
/usr/bin/python3 - "$ROOT_DIR/.run/go2w/manual_control" <<'PY'
from pathlib import Path
import sys
destination = Path(sys.argv[1])
temporary = destination.with_suffix('.handoff.tmp')
temporary.write_text('0 0\n')
temporary.replace(destination)
PY
for control_unit in go2w-course.service go2w-manual.service; do
  if systemctl --user is-active --quiet "$control_unit"; then
    systemctl --user stop "$control_unit"
  fi
done
systemctl --user reset-failed go2w-manual.service >/dev/null 2>&1 || true

OUTPUT_DIR="$(cat .run/go2w/current_output_dir 2>/dev/null || true)"
if [[ ! -d "$OUTPUT_DIR" ]]; then OUTPUT_DIR="$ROOT_DIR/logs"; fi
MANUAL_LOG="$OUTPUT_DIR/manual_here_$(date +%Y%m%d_%H%M%S).log"
systemd-run --user --quiet --collect --unit=go2w-manual \
  --property=KillMode=control-group --property=KillSignal=SIGINT \
  --property=TimeoutStopSec=5 --property="WorkingDirectory=$ROOT_DIR" \
  --property="StandardOutput=append:$MANUAL_LOG" \
  --property="StandardError=append:$MANUAL_LOG" \
  --setenv="ZENOH_SESSION_CONFIG_URI=$ZENOH_SESSION_CONFIG_URI" \
  --setenv=ROS_DOMAIN_ID=89 --setenv=RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  --setenv=GO2W_MANUAL_COMPACT=1 --setenv=DISPLAY=:0 \
  --setenv="XAUTHORITY=$XAUTHORITY" \
  /bin/bash "$ROOT_DIR/scripts/start_go2w_manual.sh"
printf 'WASD panel opened; current pose, map and four experiment views preserved.\n'
printf 'Click the panel and hold W/A/S/D or arrow keys. Release or lose focus to stop.\n'
printf 'Space/Esc: stop and lock; “接管手动” unlocks. R resets the simulation to its start.\n'
printf 'Log: %s\n' "$MANUAL_LOG"
