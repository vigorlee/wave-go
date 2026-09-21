#!/usr/bin/env bash
set -eo pipefail
SCENE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${SCENE_ROS_SETUP:-/home/unitree/flycar/tools/setup_noetic.bash}"
export ROS_MASTER_URI="${SCENE_ROS_MASTER:-http://127.0.0.1:11323}"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
export QT_QPA_PLATFORM_PLUGIN_PATH="${CONDA_PREFIX}/plugins"
mkdir -p "$SCENE_ROOT/.runtime"
owned_master=''
if ! rosparam list >/dev/null 2>&1; then
  roscore -p "${ROS_MASTER_URI##*:}" > "$SCENE_ROOT/.runtime/roscore.log" 2>&1 &
  owned_master=$!
  for attempt in $(seq 1 30); do
    if rosparam list >/dev/null 2>&1; then break; fi
    sleep 0.2
  done
fi
python "$SCENE_ROOT/scripts/publish_scene.py" > "$SCENE_ROOT/.runtime/publisher.log" 2>&1 &
publisher=$!
cleanup() {
  kill "$publisher" 2>/dev/null || true
  if [[ -n "$owned_master" ]]; then kill "$owned_master" 2>/dev/null || true; fi
}
trap cleanup EXIT
python "$SCENE_ROOT/scripts/view_app.py" "$@"
