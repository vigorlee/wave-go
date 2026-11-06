#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
systemctl --user stop go2w-course.service >/dev/null 2>&1 || true
export COSMOS_VLN_ARTIFACT_DIR="${COSMOS_VLN_ARTIFACT_DIR:-$ROOT_DIR/artifacts/Wavegov1_$(date +%Y%m%d_%H%M%S)}"
export GO2W_RL_STAIR_ALIGNMENT_MAX_HEADING=.40
export GO2W_RL_STAIR_ALIGNMENT_VX=.70
bash scripts/start_go2w_three_scenes_live.sh
mkdir -p "$COSMOS_VLN_ARTIFACT_DIR/course"
systemd-run --user --quiet --collect --unit=go2w-course \
  --property=KillMode=control-group --property=KillSignal=SIGINT \
  --property="WorkingDirectory=$ROOT_DIR" \
  --property="StandardOutput=append:$COSMOS_VLN_ARTIFACT_DIR/course.log" \
  --property="StandardError=append:$COSMOS_VLN_ARTIFACT_DIR/course.log" \
  /bin/bash -c 'source /opt/ros/humble/setup.bash; source genisom_roamerx_open/install/setup.bash; export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp; exec /usr/bin/python3 scripts/go2w_continuous_course.py --output "$1"' \
  wavegov1 "$COSMOS_VLN_ARTIFACT_DIR/course"
for _ in {1..20}; do
  if ! systemctl --user is-active --quiet go2w-course.service; then
    tail -n 30 "$COSMOS_VLN_ARTIFACT_DIR/course.log" >&2
    exit 1
  fi
  if grep -q 'continuous_goal_requested' "$COSMOS_VLN_ARTIFACT_DIR/course/events.jsonl" 2>/dev/null; then
    echo "[OK] Continuous Nav2 action accepted for dispatch."
    break
  fi
  sleep 1
done
grep -q 'continuous_goal_requested' "$COSMOS_VLN_ARTIFACT_DIR/course/events.jsonl"
echo "[OK] Continuous route started; reference, Nav2 plan and measured trail are visible in RViz."
