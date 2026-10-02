#!/usr/bin/env bash
# Display the bundled registered returns without starting robot control.
set -eo pipefail
scene_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scene_view=side
scene_check=0
for scene_arg in "$@"; do
  case "$scene_arg" in
    --check) scene_check=1 ;;
    --top) scene_view=top ;;
    --side) scene_view=side ;;
    *) echo "Usage: bash scripts/replay_map.sh [--check] [--top|--side]" >&2; exit 2 ;;
  esac
done
source "${WAVEGO_ROS_SETUP:-/opt/ros/humble/setup.bash}"
export ROS_DOMAIN_ID="${WAVEGO_REPLAY_DOMAIN_ID:-137}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
scene_cmd=(/usr/bin/python3 "$scene_root/runtime/scripts/global_compare/lio_map_review.py"
  --source "$scene_root/data/smac/capture/go2w_live_dense_3cm.npz"
  --alignment "$scene_root/data/smac/frame_alignment.json"
  --trajectory "$scene_root/data/smac/capture/lio_trajectory.jsonl"
  --semantic "$scene_root/data/smac/semantic/semantic_map.npz")
if [[ "$scene_check" == 1 ]]; then exec "${scene_cmd[@]}"; fi
scene_config="$scene_root/runtime/config/global_compare/lio_map_spectral_side_20261002.rviz"
if [[ "$scene_view" == top ]]; then
  scene_config="$scene_root/runtime/config/global_compare/lio_map_review_20261002.rviz"
fi
mkdir -p "$scene_root/.run"
"${scene_cmd[@]}" --publish > "$scene_root/.run/replay.log" 2>&1 &
scene_pid=$!
trap 'kill -INT "$scene_pid" 2>/dev/null || true; wait "$scene_pid" 2>/dev/null || true' EXIT
rviz2 -d "$scene_config"
