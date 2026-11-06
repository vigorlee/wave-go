#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
systemctl --user stop go2w-exploration-debug.service >/dev/null 2>&1 || true
export GO2W_MAPLESS=1
export GO2W_RL_STAIR_APPROACH_ASSIST=0
export GO2W_EXPLORATION_OUTPUT="${GO2W_EXPLORATION_OUTPUT:-$ROOT_DIR/artifacts/exploration_$(date +%Y%m%d_%H%M%S)}"
export COSMOS_VLN_ARTIFACT_DIR="$GO2W_EXPLORATION_OUTPUT"
export ROAMERX_RVIZ_CONFIG="$ROOT_DIR/config/go2w_online_mapping.rviz"
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
mkdir -p "$GO2W_EXPLORATION_OUTPUT"
bash scripts/start_go2w_three_scenes_live.sh
/usr/bin/python3 - "$GO2W_EXPLORATION_OUTPUT" <<'PY'
import json,sys,time
from pathlib import Path
p=Path(sys.argv[1])/'summary.json'
for _ in range(30):
    if p.exists():
        data=json.loads(p.read_text())
        if data['map_points']>1000 and data['registered_scans']>5:
            print('[OK] Live LiDAR/IMU mapping and autonomous frontier explorer are publishing.')
            break
    time.sleep(1)
else:
    raise SystemExit('Mapping did not become ready; inspect logs/online_exploration.log')
PY
