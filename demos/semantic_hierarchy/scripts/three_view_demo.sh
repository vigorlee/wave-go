#!/usr/bin/env bash
set -eo pipefail
SCENE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENE_DATA="${SCENE_DATASET_ROOT:-$SCENE_ROOT}"
SCENE_RUN="${SCENE_SESSION_DIR:-$SCENE_ROOT/.runtime}"
source "${SCENE_ROS_SETUP:-/home/unitree/flycar/tools/setup_noetic.bash}"
export ROS_MASTER_URI="${SCENE_ROS_MASTER:-http://127.0.0.1:11324}"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
export QT_QPA_PLATFORM_PLUGIN_PATH="${CONDA_PREFIX}/plugins"
mkdir -p "$SCENE_RUN"
for artifact in mapping_replay.npz model_semantics.npz replay_report.json; do
  if [[ ! -s "$SCENE_DATA/outputs/reference_demo/$artifact" ]]; then
    echo "Missing $artifact; follow docs/three_representations.md to prepare inputs." >&2
    exit 1
  fi
done
owned_master=''
if ! rosparam list >/dev/null 2>&1; then
  roscore -p "${ROS_MASTER_URI##*:}" > "$SCENE_RUN/three_master.log" 2>&1 &
  owned_master=$!
  for attempt in $(seq 1 30); do
    if rosparam list >/dev/null 2>&1; then break; fi
    sleep 0.2
  done
fi
python -u "$SCENE_ROOT/scripts/publish_reference_demo.py" > "$SCENE_RUN/three_publisher.log" 2>&1 &
publisher=$!
cleanup() {
  kill "$publisher" 2>/dev/null || true
  if [[ -n "$owned_master" ]]; then kill "$owned_master" 2>/dev/null || true; fi
}
trap cleanup EXIT
python "$SCENE_ROOT/scripts/three_view_demo.py" "$@"
