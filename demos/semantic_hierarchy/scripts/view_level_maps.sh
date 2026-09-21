#!/usr/bin/env bash
set -eo pipefail
SCENE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /home/unitree/flycar/tools/setup_noetic.bash
export ROS_MASTER_URI=http://127.0.0.1:11326 DISPLAY="${DISPLAY:-:0}" XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
export QT_QPA_PLATFORM_PLUGIN_PATH="$CONDA_PREFIX/plugins"
SCENE_MASTER=''
if ! rosparam list >/dev/null 2>&1; then
 roscore -p 11326 > "$SCENE_ROOT/.runtime/level_view_master.log" 2>&1 &
 SCENE_MASTER=$!
 sleep 2
fi
trap 'if [[ -n "$SCENE_MASTER" ]]; then kill "$SCENE_MASTER" 2>/dev/null || true; fi' EXIT
python "$SCENE_ROOT/scripts/view_level_maps.py" "$@"
