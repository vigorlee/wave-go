#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/demo_env.sh"
require_runtime
source_ros
ROS_PYTHON_BIN="${ROS_PYTHON_BIN:-/usr/bin/python3}"

if [[ "${EUID}" -eq 0 ]]; then
  echo "[ERROR] Run the desktop demo as the normal workstation user, not sudo." >&2
  exit 1
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
ARTIFACT_DIR="${COSMOS_VLN_ARTIFACT_DIR:-${DEMO_ROOT}/artifacts/extended_navigation_${STAMP}}"
VIDEO_PATH="${ARTIFACT_DIR}/wave_go_extended_navigation_15fps.mp4"
MISSION="Complete both staircases while avoiding the center-course cylinders, weave around the post-stair barriers, avoid the cylinders centered on the uphill and downhill ramps, avoid the moving pedestrians and remote obstacles, then take the longest approved route to the farthest endpoint."
RECORDER_PID=""
FFMPEG_PID=""
CLEANUP_DONE=0

export MATRIX_ROBOT_INITIAL_X="${MATRIX_ROBOT_INITIAL_X:-0.2}"
export MATRIX_ROBOT_INITIAL_Y="${MATRIX_ROBOT_INITIAL_Y:-0.0}"
export MATRIX_ROBOT_INITIAL_YAW_DEG="${MATRIX_ROBOT_INITIAL_YAW_DEG:-90}"
export MATRIX_UE_OFFSCREEN="${MATRIX_UE_OFFSCREEN:-0}"
export MATRIX_UE_NO_RHI_THREAD="${MATRIX_UE_NO_RHI_THREAD:-0}"
export MATRIX_CAMERA_FREQUENCY="${MATRIX_CAMERA_FREQUENCY:-5}"
export MATRIX_CAMERA_WIDTH="${MATRIX_CAMERA_WIDTH:-960}"
export MATRIX_CAMERA_HEIGHT="${MATRIX_CAMERA_HEIGHT:-540}"
export MATRIX_DISABLE_IMAGE_SENSORS=0
export MATRIX_SKIP_STALE_PROCESS_CLEANUP="${MATRIX_SKIP_STALE_PROCESS_CLEANUP:-1}"
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
export COSMOS_VLN_MISSION_TIMEOUT_SEC="${COSMOS_VLN_MISSION_TIMEOUT_SEC:-1500}"
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
export COSMOS_VLN_VISUALIZATION_RVIZ="${COSMOS_VLN_VISUALIZATION_RVIZ:-1}"
export COSMOS_VLN_VISUALIZATION_IMAGE_WINDOW="${COSMOS_VLN_VISUALIZATION_IMAGE_WINDOW:-1}"

mkdir -p "${ARTIFACT_DIR}"
cp "${ROUTE_FILE}" "${ARTIFACT_DIR}/route_catalog.json"
cp "${PHYSICS_SCENE}" "${ARTIFACT_DIR}/physics_scene.xml"
cp "${UE_SCENE}" "${ARTIFACT_DIR}/ue_scene.xml"
cp "${SCENE_JSON}" "${ARTIFACT_DIR}/scene.json"
printf '%s\n' "${MISSION}" >"${ARTIFACT_DIR}/mission.txt"
env | grep -E '^(GO2W_RL_|COSMOS_VLN_|MATRIX_(UE|CAMERA|ROBOT|MUJOCO|SCENE)|WAVE_GO_)' \
  | sort >"${ARTIFACT_DIR}/runtime_env.txt"

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
  "${SCRIPT_DIR}/stop_extended_stack.sh" >"${ARTIFACT_DIR}/stop.log" 2>&1 || true
  return "${status}"
}
trap cleanup EXIT

"${SCRIPT_DIR}/validate_go2w_extended_world.py" >"${ARTIFACT_DIR}/world_contract.log" 2>&1
"${SCRIPT_DIR}/start_extended_stack.sh" >"${ARTIFACT_DIR}/startup.log" 2>&1
"${SCRIPT_DIR}/start_cosmos_vln_visualization.sh" >>"${ARTIFACT_DIR}/startup.log" 2>&1

if [[ -n "${DISPLAY:-}" ]] && command -v ffmpeg >/dev/null 2>&1; then
  display_size="${WAVE_GO_DISPLAY_SIZE:-4096x2304}"
  ffmpeg -y -loglevel error -f x11grab -draw_mouse 1 \
    -video_size "${display_size}" -framerate 15 -i "${DISPLAY}+0,0" \
    -vf scale=2560:1440:flags=lanczos -c:v libx264 -preset veryfast -crf 26 \
    -pix_fmt yuv420p -movflags +faststart "${VIDEO_PATH}" \
    >"${ARTIFACT_DIR}/screen_ffmpeg.log" 2>&1 &
  FFMPEG_PID=$!
fi

if [[ -x "${SCRIPT_DIR}/validate_go2w_extended_route.py" ]]; then
  # This is a live Nav2 preflight; retain its output even when a workstation
  # does not expose ComputePathToPose (the mission itself remains fail-closed).
  "${ROS_PYTHON_BIN}" "${SCRIPT_DIR}/validate_go2w_extended_route.py" \
    --output "${ARTIFACT_DIR}/path_validation.json" \
    >"${ARTIFACT_DIR}/path_validation.log" 2>&1 || true
fi

"${ROS_PYTHON_BIN}" "${DEMO_ROOT}/scripts/record_cosmos_vln_run.py" \
  --artifact-dir "${ARTIFACT_DIR}" \
  --timeout-sec "${COSMOS_VLN_MISSION_TIMEOUT_SEC}" \
  >"${ARTIFACT_DIR}/recorder.log" 2>&1 &
RECORDER_PID=$!
sleep 2
if ! kill -0 "${RECORDER_PID}" 2>/dev/null; then
  echo "[ERROR] mission recorder failed to start; see ${ARTIFACT_DIR}/recorder.log" >&2
  exit 1
fi

echo "[INFO] Publishing one high-level Cosmos mission; route execution is continuous."
ros2 topic pub --once /cosmos_vln/mission std_msgs/msg/String \
  "{data: '${MISSION}'}" >"${ARTIFACT_DIR}/mission_publish.log" 2>&1

set +e
wait "${RECORDER_PID}"
recorder_status=$?
set -e
sleep 3

if [[ -n "${FFMPEG_PID}" ]] && kill -0 "${FFMPEG_PID}" 2>/dev/null; then
  kill -INT "${FFMPEG_PID}" 2>/dev/null || true
  wait "${FFMPEG_PID}" 2>/dev/null || true
fi
if [[ -f "${VIDEO_PATH}" ]]; then
  ffprobe -v error -select_streams v:0 \
    -show_entries stream=width,height,avg_frame_rate,nb_frames,codec_name \
    -of default=noprint_wrappers=1 "${VIDEO_PATH}" >"${ARTIFACT_DIR}/video_probe.txt" 2>&1 || true
fi

if [[ "${recorder_status}" -ne 0 || ! -f "${ARTIFACT_DIR}/result.json" ]] || \
   [[ "$(jq -r '.result // "unknown"' "${ARTIFACT_DIR}/result.json" 2>/dev/null)" != "succeeded" ]]; then
  echo "[ERROR] extended navigation failed; inspect ${ARTIFACT_DIR}" >&2
  exit "${recorder_status:-1}"
fi
echo "[OK] Extended navigation succeeded: ${ARTIFACT_DIR}"
