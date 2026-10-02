#!/usr/bin/env bash
set -eo pipefail
scene_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scene_data="${SCENE_923_DATA:-$scene_root/outputs/imported_923_20260923}"
scene_runtime="$scene_root/.runtime/imported_923"
mkdir -p "$scene_data" "$scene_runtime"
if [[ ! -s "$scene_data/geometry_graph.json" || "$scene_data/923_rgb_intensity.pcd" -nt "$scene_data/geometry_graph.json" ]]; then
  python3 "$scene_root/scripts/process_imported_923.py" \
    --input "$scene_data/923_rgb_intensity.pcd" --output "$scene_data"
fi
source "${SCENE_ROS_SETUP:-/home/unitree/flycar/tools/setup_noetic.bash}"
export ROS_MASTER_URI="${SCENE_923_ROS_MASTER:-http://127.0.0.1:11330}"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
export QT_QPA_PLATFORM_PLUGIN_PATH="${CONDA_PREFIX}/plugins"
bash "$scene_root/scripts/build_chinese_labels.sh"
owned_master=''
if ! rosparam list >/dev/null 2>&1; then
  roscore -p "${ROS_MASTER_URI##*:}" > "$scene_runtime/master.log" 2>&1 &
  owned_master=$!
  for attempt in $(seq 1 40); do
    if rosparam list >/dev/null 2>&1; then break; fi
    sleep 0.2
  done
fi
python -u "$scene_root/scripts/publish_imported_923.py" --data "$scene_data" \
  > "$scene_runtime/publisher.log" 2>&1 &
publisher=$!
cleanup() {
  kill "$publisher" 2>/dev/null || true
  if [[ -n "$owned_master" ]]; then kill "$owned_master" 2>/dev/null || true; fi
}
trap cleanup EXIT
python -u "$scene_root/scripts/imported_923_demo.py" --data "$scene_data" "$@"
