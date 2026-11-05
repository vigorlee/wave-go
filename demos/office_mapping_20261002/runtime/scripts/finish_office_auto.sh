#!/usr/bin/env bash
# At the verified door goal, retain all views and hand over keyboard control.
set -eo pipefail
cd "$(dirname "$0")/.."
TAKE_DIR="${1:?Run directory is required}"
deadline=$((SECONDS+660))
while ((SECONDS < deadline)); do
  state="$(/usr/bin/python3 - "$TAKE_DIR/course/result.json" <<'PY'
import json,sys
try:
 d=json.load(open(sys.argv[1]));print('door_arrived' if d.get('passed') and d['last_event']['stage_id']=='second_floor_door' else d.get('state','waiting'))
except (OSError,ValueError):print('waiting')
PY
)"
  if [[ "$state" == door_arrived ]]; then
    # Let the independently measured plaque/door association finish while the
    # automatic follower holds its verified standing pose at the door.
    # Failure to identify the plate is recorded explicitly, never inferred from
    # arrival at a preset coordinate. The manual panel remains available.
    /usr/bin/python3 - "$TAKE_DIR" <<'PY'
import json, sys, time
from pathlib import Path
root=Path(sys.argv[1]); source=root/'door_recognition/status.json'
deadline=time.monotonic()+40; verified=False; status={}
while time.monotonic()<deadline:
 try:
  status=json.loads(source.read_text());goal=status.get('goal_status',{})
  if goal.get('confirmed') and 0<=time.time()-goal.get('last_observed_stamp',0)<=4.0:
   verified=True;break
 except (OSError,ValueError): pass
 time.sleep(.5)
record={'time':time.time(),'physical_navigation_passed':True,
        'room_plate_goal_verified':verified,'goal_status':status.get('goal_status',{}),
        'source_status':str(source.resolve()),'verification_uses_ocr_and_measured_depth':True}
(root/'door_identity_handoff.json').write_text(json.dumps(record,indent=2)+'\n')
print('Door identity verified before manual handoff' if verified else 'Physical door arrival passed; room identity remains unverified',flush=True)
PY
    /usr/bin/python3 scripts/capture_office_four_windows.py --output-dir "$TAKE_DIR/screenshots" || true
    # Complete the experiment video before the user takes the controls.
    systemctl --user stop go2w-office-record.service >/dev/null 2>&1 || true
    bash scripts/open_office_manual_here.sh
    printf '%s Manual control opened at second-floor door.\n' "$(date --iso-8601=seconds)" > "$TAKE_DIR/manual_handoff.txt"
    exit 0
  elif [[ "$state" == failed || "$state" == canceled ]]; then
    systemctl --user stop go2w-office-record.service >/dev/null 2>&1 || true
    echo "Route $state; keyboard handoff withheld, robot remains stopped." >&2
    exit 1
  fi
  sleep 1
done
echo 'No successful door arrival within the handoff timeout.' >&2
exit 1
