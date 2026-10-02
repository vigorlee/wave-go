#!/usr/bin/env bash
set -eo pipefail

scene_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scene_name="${1:?Usage: show_z5_features.sh 12|mt|stitched [--capture]}"
shift
case "$scene_name" in
  12) scene_parent="$scene_root/outputs/imported_12_20260926"; scene_port=11333 ;;
  mt) scene_parent="$scene_root/outputs/imported_mt_20260926"; scene_port=11334 ;;
  stitched) scene_parent="$scene_root/outputs/imported_12_20260926"; scene_port=11335 ;;
  *) printf 'Unknown dataset: %s (expected 12, mt or stitched)\n' "$scene_name" >&2; exit 2 ;;
esac
scene_data="$scene_parent/z5_features"
if [[ "$scene_name" == stitched ]]; then
  scene_data="$scene_parent/z5_stitched_12_mt"
fi
scene_runtime="$scene_root/.runtime/z5_$scene_name"
mkdir -p "$scene_runtime"
if [[ "$scene_name" == stitched ]]; then
  if [[ ! -s "$scene_data/feature_graph.json" ||
        "$scene_root/scripts/stitch_z5_left_stairs.py" -nt "$scene_data/feature_graph.json" ||
        "$scene_parent/z5_features/feature_graph.json" -nt "$scene_data/feature_graph.json" ||
        "$scene_root/outputs/imported_mt_20260926/z5_features/feature_graph.json" -nt "$scene_data/feature_graph.json" ]]; then
    python3 "$scene_root/scripts/stitch_z5_left_stairs.py"
  fi
elif [[ ! -s "$scene_data/feature_graph.json" || ! -s "$scene_data/full_z5_rgb.pcd" ||
        "$scene_root/scripts/process_z5_features.py" -nt "$scene_data/feature_graph.json" ]]; then
  python3 "$scene_root/scripts/process_z5_features.py" --name "$scene_name"
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
python -u "$scene_root/scripts/publish_z5_features.py" --data "$scene_data" \
  > "$scene_runtime/publisher.log" 2>&1 &
publisher=$!
cleanup() {
  kill "$publisher" 2>/dev/null || true
  if [[ -n "$owned_master" ]]; then kill "$owned_master" 2>/dev/null || true; fi
}
trap cleanup EXIT
python -u "$scene_root/scripts/z5_feature_demo.py" --data "$scene_data" "$@"
