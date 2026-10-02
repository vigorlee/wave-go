#!/usr/bin/env bash
set -eo pipefail

scene_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scene_name="${1:?Usage: show_roofless_import.sh 12|mt [--capture]}"
shift
case "$scene_name" in
  12)
    scene_data="$scene_root/outputs/imported_12_20260926"
    scene_input="$scene_data/12_rgb_intensity.pcd"
    scene_port=11331
    ;;
  mt)
    scene_data="$scene_root/outputs/imported_mt_20260926"
    scene_input="$scene_data/MT_rgb_intensity.pcd"
    scene_port=11332
    ;;
  *)
    printf 'Unknown dataset: %s (expected 12 or mt)\n' "$scene_name" >&2
    exit 2
    ;;
esac
scene_runtime="$scene_root/.runtime/imported_$scene_name"
mkdir -p "$scene_runtime"
if [[ ! -s "$scene_data/geometry_graph.json" || ! -s "$scene_data/full_no_roof.pcd" ||
      "$scene_input" -nt "$scene_data/geometry_graph.json" ||
      "$scene_root/scripts/process_roofless_import.py" -nt "$scene_data/geometry_graph.json" ]]; then
  python3 "$scene_root/scripts/process_roofless_import.py" \
    --input "$scene_input" --output "$scene_data" --name "$scene_name"
fi

source "${SCENE_ROS_SETUP:-/home/unitree/flycar/tools/setup_noetic.bash}"
export ROS_MASTER_URI="${SCENE_ROS_MASTER:-http://127.0.0.1:$scene_port}"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  export QT_QPA_PLATFORM_PLUGIN_PATH="$CONDA_PREFIX/plugins"
fi
bash "$scene_root/scripts/build_chinese_labels.sh"

owned_master=''
if ! rosparam list >/dev/null 2>&1; then
  roscore -p "${ROS_MASTER_URI##*:}" > "$scene_runtime/master.log" 2>&1 &
  owned_master=$!
  for attempt in $(seq 1 50); do
    if rosparam list >/dev/null 2>&1; then break; fi
    sleep 0.2
  done
fi
python -u "$scene_root/scripts/publish_roofless_import.py" --data "$scene_data" \
  > "$scene_runtime/publisher.log" 2>&1 &
publisher=$!
cleanup() {
  kill "$publisher" 2>/dev/null || true
  if [[ -n "$owned_master" ]]; then kill "$owned_master" 2>/dev/null || true; fi
}
trap cleanup EXIT
python -u "$scene_root/scripts/imported_923_demo.py" --data "$scene_data" "$@"
