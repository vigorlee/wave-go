#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
PHYSICS_SCENE="${ROOT_DIR}/matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_yard_extended_nav.xml"
UE_SCENE="${ROOT_DIR}/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/scene_terrain_yard_extended_nav.xml"
SCENE_JSON="${ROOT_DIR}/matrix/scene/scene_go2w_extended_nav.json"
ROUTE_ID="farthest_end_via_stairs_ramps_cylinders_pedestrians"
MISSION="Complete both staircases while avoiding the center-course cylinders, weave around the post-stair barriers, avoid the cylinders centered on the uphill and downhill ramps, avoid the moving pedestrians and remote obstacles, then take the longest approved route to the farthest endpoint."
STAMP="$(date +%Y%m%d_%H%M%S)"
ARTIFACT_DIR="${COSMOS_VLN_ARTIFACT_DIR:-${ROOT_DIR}/artifacts/cosmos_extended_desktop_${STAMP}}"
VIDEO_PATH="${ARTIFACT_DIR}/matrix_go2w_cosmos_extended_navigation_15fps.mp4"
DISPLAY_SIZE="4096x2304"
FFMPEG_PID=""
RECORDER_PID=""
CLEANUP_DONE=0

if [[ "${EUID}" -eq 0 ]]; then
  echo "[ERROR] Run this recording as the desktop user, not with sudo." >&2
  exit 1
fi
if [[ ! -x "${ROAMERX_DIR}/install/setup.bash" && ! -f "${ROAMERX_DIR}/install/setup.bash" ]]; then
  echo "[ERROR] RoamerX install is missing: ${ROAMERX_DIR}/install/setup.bash" >&2
  exit 1
fi

export DISPLAY="${DISPLAY:-:0}"
if [[ -z "${XAUTHORITY:-}" && -r "/run/user/$(id -u)/gdm/Xauthority" ]]; then
  export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
fi
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="89"
export ROS2CLI_NO_DAEMON="1"
export MATRIX_ROBOT_INITIAL_X="0.2"
export MATRIX_ROBOT_INITIAL_Y="0.0"
export MATRIX_ROBOT_INITIAL_YAW_DEG="90"
export MATRIX_UE_OFFSCREEN="0"
export MATRIX_UE_NO_RHI_THREAD="${MATRIX_UE_NO_RHI_THREAD:-0}"
export MATRIX_UE_MAX_FPS="15"
export MATRIX_CAMERA_FREQUENCY="${MATRIX_CAMERA_FREQUENCY:-5}"
export MATRIX_CAMERA_WIDTH="960"
export MATRIX_CAMERA_HEIGHT="540"
export MATRIX_DISABLE_IMAGE_SENSORS="0"
export ROAMERX_RVIZ_CONFIG="${ROAMERX_RVIZ_CONFIG:-${ROOT_DIR}/config/go2w_extended_navigation.rviz}"
export MATRIX_MUJOCO_SCENE="go2w_lcm/scene_terrain_yard_extended_nav.xml"
export MATRIX_UE_SCENE_OVERRIDE="${UE_SCENE}"
export MATRIX_SCENE_JSON_OVERRIDE="${SCENE_JSON}"
export GO2W_WORLD_SCENE="${PHYSICS_SCENE}"
export GO2W_SCENE_JSON="${SCENE_JSON}"
export GO2W_MJ_STATE_RELAY=1
export MATRIX_WORLD_MODEL_VALIDATOR="${ROOT_DIR}/scripts/validate_go2w_extended_world.py"
export MATRIX_SKIP_STALE_PROCESS_CLEANUP="1"
export GO2W_RL_STOCHASTIC="${GO2W_RL_STOCHASTIC:-1}"
export GO2W_RL_STAIR_OBSTACLE_AVOIDANCE="${GO2W_RL_STAIR_OBSTACLE_AVOIDANCE:-1}"
export GO2W_RL_STAIR_PLANNER_YAW_BLEND="${GO2W_RL_STAIR_PLANNER_YAW_BLEND:-0.90}"
export GO2W_RL_STAIR_PLANNER_CENTERING_SCALE="${GO2W_RL_STAIR_PLANNER_CENTERING_SCALE:-0.25}"
export GO2W_RL_STAIR_PLANNER_MIN_VX="${GO2W_RL_STAIR_PLANNER_MIN_VX:-0.18}"
export GO2W_RL_STAIR_PLANNER_MAX_VX="${GO2W_RL_STAIR_PLANNER_MAX_VX:-0.80}"
export GO2W_RL_STAIR_PLANNER_TURN_THRESHOLD="${GO2W_RL_STAIR_PLANNER_TURN_THRESHOLD:-0.06}"
export GO2W_RL_STAIR_OBSTACLE_MIN_Y="${GO2W_RL_STAIR_OBSTACLE_MIN_Y:-2.55}"
export GO2W_RL_STAIR_OBSTACLE_MAX_Y="${GO2W_RL_STAIR_OBSTACLE_MAX_Y:-3.70}"
export COSMOS_VLN_MISSION_REQUIRED_ROUTE_ID="${ROUTE_ID}"
export COSMOS_VLN_MISSION_TIMEOUT_SEC="${COSMOS_VLN_MISSION_TIMEOUT_SEC:-1200}"
export COSMOS_VLN_ROUTE_NO_PROGRESS_SEC="${COSMOS_VLN_ROUTE_NO_PROGRESS_SEC:-45}"
export COSMOS_VLN_STAIR_MAX_HEADING="${COSMOS_VLN_STAIR_MAX_HEADING:-0.68}"
export COSMOS_VLN_NAV_MODE_ATTEMPTS="${COSMOS_VLN_NAV_MODE_ATTEMPTS:-4}"
export COSMOS_VLN_CONTINUOUS_ROUTE="${COSMOS_VLN_CONTINUOUS_ROUTE:-1}"
export COSMOS_VLN_CONTINUOUS_PASS_RADIUS="${COSMOS_VLN_CONTINUOUS_PASS_RADIUS:-0.68}"
export COSMOS_VLN_RAMP_MIN_ELEVATION_GAIN="${COSMOS_VLN_RAMP_MIN_ELEVATION_GAIN:-0.30}"
export COSMOS_VLN_RAMP_MIN_PITCH="${COSMOS_VLN_RAMP_MIN_PITCH:-0.07}"
export COSMOS_VLN_RAMP_CREST_MIN_Z="${COSMOS_VLN_RAMP_CREST_MIN_Z:-0.72}"
export COSMOS_VLN_RAMP_EXIT_MAX_Z="${COSMOS_VLN_RAMP_EXIT_MAX_Z:-0.58}"
export COSMOS_VLN_INFERENCE_TIMEOUT_SEC="${COSMOS_VLN_INFERENCE_TIMEOUT_SEC:-900}"
export COSMOS_VLN_VISUALIZATION_RVIZ="1"
export COSMOS_VLN_VISUALIZATION_IMAGE_WINDOW="1"
export COSMOS_VLN_RVIZ_CONFIG="${ROOT_DIR}/config/cosmos_vln_extended.rviz"

mkdir -p "${ARTIFACT_DIR}"
cleanup() {
  local status=$?
  set +e
  if (( CLEANUP_DONE )); then
    return "${status}"
  fi
  CLEANUP_DONE=1
  if [[ -n "${RECORDER_PID}" ]] && kill -0 "${RECORDER_PID}" 2>/dev/null; then
    kill -TERM "${RECORDER_PID}" 2>/dev/null || true
    wait "${RECORDER_PID}" 2>/dev/null || true
  fi
  if [[ -n "${FFMPEG_PID}" ]] && kill -0 "${FFMPEG_PID}" 2>/dev/null; then
    kill -INT "${FFMPEG_PID}" 2>/dev/null || true
    wait "${FFMPEG_PID}" 2>/dev/null || true
  fi
  if [[ "${GO2W_KEEP_LIVE:-0}" != 1 || "${status}" != 0 ]]; then
    timeout 45 "${ROOT_DIR}/scripts/stop_demo.sh" >"${ARTIFACT_DIR}/stop.log" 2>&1 || true
  fi
  return "${status}"
}
trap cleanup EXIT
cp "${ROOT_DIR}/config/cosmos_vln_routes.json" "${ARTIFACT_DIR}/route_catalog.json"
cp "${PHYSICS_SCENE}" "${ARTIFACT_DIR}/physics_scene.xml"
cp "${UE_SCENE}" "${ARTIFACT_DIR}/ue_scene.xml"
cp "${SCENE_JSON}" "${ARTIFACT_DIR}/scene.json"
printf '%s\n' "${MISSION}" >"${ARTIFACT_DIR}/mission.txt"
printf '%s\n' "GO2W_RL_STOCHASTIC=${GO2W_RL_STOCHASTIC}" >"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "GO2W_RL_STAIR_OBSTACLE_AVOIDANCE=${GO2W_RL_STAIR_OBSTACLE_AVOIDANCE}" >>"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "GO2W_RL_STAIR_PLANNER_YAW_BLEND=${GO2W_RL_STAIR_PLANNER_YAW_BLEND}" >>"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "GO2W_RL_STAIR_PLANNER_CENTERING_SCALE=${GO2W_RL_STAIR_PLANNER_CENTERING_SCALE}" >>"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "GO2W_RL_STAIR_PLANNER_MIN_VX=${GO2W_RL_STAIR_PLANNER_MIN_VX}" >>"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "GO2W_RL_STAIR_PLANNER_MAX_VX=${GO2W_RL_STAIR_PLANNER_MAX_VX}" >>"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "GO2W_RL_STAIR_PLANNER_TURN_THRESHOLD=${GO2W_RL_STAIR_PLANNER_TURN_THRESHOLD}" >>"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "GO2W_RL_STAIR_OBSTACLE_MIN_Y=${GO2W_RL_STAIR_OBSTACLE_MIN_Y}" >>"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "GO2W_RL_STAIR_OBSTACLE_MAX_Y=${GO2W_RL_STAIR_OBSTACLE_MAX_Y}" >>"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "COSMOS_VLN_STAIR_MAX_HEADING=${COSMOS_VLN_STAIR_MAX_HEADING}" >>"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "COSMOS_VLN_NAV_MODE_ATTEMPTS=${COSMOS_VLN_NAV_MODE_ATTEMPTS}" >>"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "COSMOS_VLN_CONTINUOUS_ROUTE=${COSMOS_VLN_CONTINUOUS_ROUTE}" >>"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "COSMOS_VLN_RAMP_MIN_ELEVATION_GAIN=${COSMOS_VLN_RAMP_MIN_ELEVATION_GAIN}" >>"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "COSMOS_VLN_RAMP_MIN_PITCH=${COSMOS_VLN_RAMP_MIN_PITCH}" >>"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "COSMOS_VLN_RAMP_CREST_MIN_Z=${COSMOS_VLN_RAMP_CREST_MIN_Z}" >>"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "COSMOS_VLN_RAMP_EXIT_MAX_Z=${COSMOS_VLN_RAMP_EXIT_MAX_Z}" >>"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "MATRIX_UE_OFFSCREEN=${MATRIX_UE_OFFSCREEN}" >>"${ARTIFACT_DIR}/runtime_env.txt"
printf '%s\n' "MATRIX_CAMERA_FREQUENCY=${MATRIX_CAMERA_FREQUENCY}" >>"${ARTIFACT_DIR}/runtime_env.txt"

cd "${ROOT_DIR}"
# Start from a clean graph/window set so an earlier failed take cannot steal a
# ROS topic, an RViz slot, or the mission supervisor PID check.
./scripts/stop_cosmos_vln_mission.sh >/dev/null 2>&1 || true
./scripts/stop_cosmos_vln_bridge.sh >/dev/null 2>&1 || true
./scripts/stop_cosmos_vln_visualization.sh >/dev/null 2>&1 || true
./scripts/stop_go2w_systemd.sh >/dev/null 2>&1 || true
./scripts/validate_go2w_extended_world.py >"${ARTIFACT_DIR}/world_contract.log" 2>&1

echo "[INFO] Starting extended Go2-W/Cosmos stack (on-screen)."
./scripts/start_go2w_cosmos_vln.sh >"${ARTIFACT_DIR}/startup.log" 2>&1
./scripts/start_cosmos_vln_visualization.sh >>"${ARTIFACT_DIR}/startup.log" 2>&1
if ! cmp -s "${UE_SCENE}" "${ROOT_DIR}/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/scene_terrain_yard.xml"; then
  echo "[ERROR] UE YardWorld active variant is not the extended scene." >&2
  exit 1
fi
# Only the annotated Cosmos camera window is part of the requested 2×2 view;
# close the auxiliary Cosmos RViz instance so it cannot reappear over the
# desktop while recording.
if [[ -f "${ROOT_DIR}/.run/cosmos_vln_rviz.pid" ]]; then
  auxiliary_rviz_pid="$(cat "${ROOT_DIR}/.run/cosmos_vln_rviz.pid" 2>/dev/null || true)"
  if [[ "${auxiliary_rviz_pid}" =~ ^[0-9]+$ ]]; then
    kill -TERM "${auxiliary_rviz_pid}" 2>/dev/null || true
  fi
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u

wait_for_window() {
  local pattern="$1"
  local timeout_sec="${2:-180}"
  local deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < deadline )); do
    if wmctrl -l 2>/dev/null | grep -Fq "${pattern}"; then
      return 0
    fi
    sleep 2
  done
  echo "[ERROR] Timed out waiting for window: ${pattern}" >&2
  wmctrl -lG >&2 || true
  return 1
}

wait_for_window "MuJoCo : go2w"
wait_for_window "zsibot_mujoco_ue"
wait_for_window "$(basename "${ROAMERX_RVIZ_CONFIG}")"
wait_for_window "Cosmos3-Edge VLN Camera Prediction"

find_window_id() {
  local pattern="$1"
  # Windows can disappear while wmctrl enumerates them (notably the
  # auxiliary RViz closed above). Retry layout instead of aborting the run.
  (wmctrl -l 2>/dev/null || true) | awk -v pattern="${pattern}" 'index($0, pattern) {print $1; exit}'
}

layout_windows() {
  local mujoco_id ue_id nav_id cosmos_id
  mujoco_id="$(find_window_id 'MuJoCo : go2w')"
  ue_id="$(find_window_id 'zsibot_mujoco_ue')"
  nav_id="$(find_window_id "$(basename "${ROAMERX_RVIZ_CONFIG}")")"
  cosmos_id="$(find_window_id 'Cosmos3-Edge VLN Camera Prediction')"
  for id in "${mujoco_id}" "${ue_id}" "${nav_id}" "${cosmos_id}"; do
    [[ -n "${id}" ]] || continue
    wmctrl -i -r "${id}" -b remove,maximized_vert,maximized_horz 2>/dev/null || true
    wmctrl -i -r "${id}" -b add,above 2>/dev/null || true
    wmctrl -i -a "${id}" 2>/dev/null || true
  done
  [[ -n "${mujoco_id}" ]] && wmctrl -i -r "${mujoco_id}" -e 0,100,70,1900,950 || true
  [[ -n "${nav_id}" ]] && wmctrl -i -r "${nav_id}" -e 0,2080,70,1900,950 || true
  [[ -n "${cosmos_id}" ]] && wmctrl -i -r "${cosmos_id}" -e 0,100,1110,1900,950 || true
  [[ -n "${ue_id}" ]] && wmctrl -i -r "${ue_id}" -e 0,2080,1110,1900,950 || true
  # The Cosmos marker RViz is auxiliary; hide it so the four requested
  # interfaces occupy the full 2x2 layout.
  while read -r id; do
    [[ -n "${id}" ]] || continue
    wmctrl -i -r "${id}" -b add,hidden 2>/dev/null || true
  done < <((wmctrl -l 2>/dev/null || true) | awk '/config\/cosmos_vln(_extended)?\.rviz/ {print $1}')
}

for _ in {1..5}; do
  layout_windows
  sleep 1
done
wmctrl -lG >"${ARTIFACT_DIR}/window_layout.txt" 2>/dev/null || true

ffmpeg -y -loglevel error -f x11grab -draw_mouse 1 -video_size "${DISPLAY_SIZE}" \
  -framerate 15 -i "${DISPLAY}+0,0" \
  -vf scale=2560:1440:flags=lanczos -c:v h264_nvenc -preset p4 -rc vbr -cq 26 -b:v 0 \
  -pix_fmt yuv420p -movflags +faststart "${VIDEO_PATH}" \
  >"${ARTIFACT_DIR}/screen_ffmpeg.log" 2>&1 &
FFMPEG_PID=$!
echo "${FFMPEG_PID}" >"${ARTIFACT_DIR}/screen_recorder.pid"
sleep 3
if ! kill -0 "${FFMPEG_PID}" 2>/dev/null; then
  echo "[ERROR] Desktop ffmpeg recorder failed to start." >&2
  tail -n 80 "${ARTIFACT_DIR}/screen_ffmpeg.log" >&2 || true
  exit 1
fi

"${ROOT_DIR}/scripts/check_demo.sh" >"${ARTIFACT_DIR}/check_demo.log" 2>&1 || true
"${ROOT_DIR}/scripts/check_cosmos_vln.sh" >"${ARTIFACT_DIR}/check_cosmos_vln.log" 2>&1 || true
/usr/bin/python3 "${ROOT_DIR}/scripts/validate_go2w_extended_route.py" \
  --output "${ARTIFACT_DIR}/path_validation.json" \
  >"${ARTIFACT_DIR}/path_validation.log" 2>&1

/usr/bin/python3 "${ROOT_DIR}/scripts/record_cosmos_vln_run.py" \
  --artifact-dir "${ARTIFACT_DIR}" \
  --timeout-sec "${COSMOS_VLN_MISSION_TIMEOUT_SEC}" \
  >"${ARTIFACT_DIR}/recorder.log" 2>&1 &
RECORDER_PID=$!
echo "${RECORDER_PID}" >"${ARTIFACT_DIR}/recorder.pid"
sleep 2
if ! kill -0 "${RECORDER_PID}" 2>/dev/null; then
  echo "[ERROR] Mission recorder failed to start." >&2
  tail -n 80 "${ARTIFACT_DIR}/recorder.log" >&2 || true
  kill -INT "${FFMPEG_PID}" 2>/dev/null || true
  exit 1
fi

echo "[INFO] Publishing mission; four windows are being recorded at 15 fps."
ros2 topic pub --once /cosmos_vln/mission std_msgs/msg/String \
  "{data: '${MISSION}'}" >"${ARTIFACT_DIR}/mission_publish.log" 2>&1

set +e
wait "${RECORDER_PID}"
RECORDER_STATUS=$?
set -e

# Keep the successful terminal state visible for a short hold in the video.
sleep 3
kill -INT "${FFMPEG_PID}" 2>/dev/null || true
set +e
wait "${FFMPEG_PID}"
FFMPEG_STATUS=$?
set -e

cp "${ROOT_DIR}/logs/cosmos_vln_bridge.log" "${ARTIFACT_DIR}/cosmos_vln_bridge.log" 2>/dev/null || true
cp "${ROOT_DIR}/logs/cosmos_vln_mission.log" "${ARTIFACT_DIR}/cosmos_vln_mission.log" 2>/dev/null || true
cp "${ROOT_DIR}/logs/cosmos_vln_visualizer.log" "${ARTIFACT_DIR}/cosmos_vln_visualizer.log" 2>/dev/null || true
cp "${ROOT_DIR}/logs/cosmos_vln_image_view.log" "${ARTIFACT_DIR}/cosmos_vln_image_view.log" 2>/dev/null || true
cp "${ROOT_DIR}/logs/go2w_systemd.log" "${ARTIFACT_DIR}/go2w_systemd.log" 2>/dev/null || true
cp "${ROOT_DIR}/logs/bridge.log" "${ARTIFACT_DIR}/controller.log" 2>/dev/null || true
cp "${ROOT_DIR}/logs/world_cloud.log" "${ARTIFACT_DIR}/world_cloud.log" 2>/dev/null || true
wmctrl -lG >"${ARTIFACT_DIR}/window_layout_final.txt" || true

if [[ -f "${VIDEO_PATH}" ]]; then
  ffprobe -v error -select_streams v:0 \
    -show_entries stream=width,height,avg_frame_rate,r_frame_rate,nb_frames,duration,codec_name \
    -of default=noprint_wrappers=1 "${VIDEO_PATH}" >"${ARTIFACT_DIR}/video_probe.txt" || true
fi
printf 'recorder_status=%s\nffmpeg_status=%s\nvideo=%s\n' \
  "${RECORDER_STATUS}" "${FFMPEG_STATUS}" "${VIDEO_PATH}" >"${ARTIFACT_DIR}/screen_status.txt"

if [[ ! -f "${ARTIFACT_DIR}/result.json" ]] || \
   [[ "$(jq -r '.acceptance_passed // false' "${ARTIFACT_DIR}/result.json")" != "true" ]]; then
  echo "[ERROR] Extended desktop mission did not succeed; evidence: ${ARTIFACT_DIR}" >&2
  exit 1
fi
if [[ ! -s "${VIDEO_PATH}" ]]; then
  echo "[ERROR] Desktop MP4 is missing or empty: ${VIDEO_PATH}" >&2
  exit 1
fi

echo "[OK] Extended desktop demo succeeded and was recorded at 15 fps."
echo "[OK] Artifact directory: ${ARTIFACT_DIR}"
echo "[OK] MP4: ${VIDEO_PATH}"
