#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
COSMOS_ROOT="/home/unitree/matrix_g1_lcm_demo"
ROUTE_ID="farthest_end_via_stairs_ramps_cylinders_pedestrians"
MISSION="Complete both staircases while avoiding the center-course cylinders, weave around the post-stair barriers, avoid the cylinders centered on the uphill and downhill ramps, avoid the moving pedestrians and remote obstacles, then take the longest approved route to the farthest endpoint."
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
ARTIFACT_DIR="${COSMOS_VLN_ARTIFACT_DIR:-${ROOT_DIR}/artifacts/cosmos_extended_navigation_${RUN_STAMP}}"
PHYSICS_SCENE="${ROOT_DIR}/matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_yard_extended_nav.xml"
UE_SCENE="${ROOT_DIR}/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/scene_terrain_yard_extended_nav.xml"
SCENE_JSON="${ROOT_DIR}/matrix/scene/scene_go2w_extended_nav.json"

if [[ "${EUID}" -eq 0 ]]; then
  echo "[ERROR] Run this script as the desktop user, not with sudo." >&2
  exit 1
fi
if [[ ! -x "${COSMOS_ROOT}/packages/cosmos-framework/.venv/bin/python" ]]; then
  echo "[ERROR] Cosmos inference environment is missing under ${COSMOS_ROOT}." >&2
  exit 1
fi
if [[ ! -d "${COSMOS_VLN_CHECKPOINT:-${COSMOS_ROOT}/Cosmos3-Edge}" ]]; then
  echo "[ERROR] Cosmos3-Edge checkpoint is missing." >&2
  echo "[INFO] Expected: ${COSMOS_VLN_CHECKPOINT:-${COSMOS_ROOT}/Cosmos3-Edge}" >&2
  exit 1
fi

export DISPLAY="${DISPLAY:-:0}"
if [[ -z "${XAUTHORITY:-}" && -r "/run/user/$(id -u)/gdm/Xauthority" ]]; then
  export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
fi

export MATRIX_ROBOT_INITIAL_X=0.2
export MATRIX_ROBOT_INITIAL_Y=0.0
export MATRIX_ROBOT_INITIAL_YAW_DEG=90
export MATRIX_UE_OFFSCREEN="${MATRIX_UE_OFFSCREEN:-0}"
export MATRIX_UE_NO_RHI_THREAD="${MATRIX_UE_NO_RHI_THREAD:-0}"
export MATRIX_UE_MAX_FPS="${MATRIX_UE_MAX_FPS:-30}"
export MATRIX_CAMERA_FREQUENCY="${MATRIX_CAMERA_FREQUENCY:-5}"
export MATRIX_MUJOCO_SCENE="go2w_lcm/scene_terrain_yard_extended_nav.xml"
export MATRIX_UE_SCENE_OVERRIDE="${UE_SCENE}"
export MATRIX_SCENE_JSON_OVERRIDE="${SCENE_JSON}"
export GO2W_WORLD_SCENE="${PHYSICS_SCENE}"
export GO2W_SCENE_JSON="${SCENE_JSON}"
export GO2W_MJ_STATE_RELAY=1
export MATRIX_WORLD_MODEL_VALIDATOR="${ROOT_DIR}/scripts/validate_go2w_extended_world.py"
export MATRIX_SKIP_STALE_PROCESS_CLEANUP="${MATRIX_SKIP_STALE_PROCESS_CLEANUP:-1}"
# Keep the validated Go2-W stair snapshot's stochastic policy setting.  The
# learned stair gait is materially more robust with this flag enabled; the
# exact environment and seed are copied into the artifact for replay.
export GO2W_RL_STOCHASTIC="${GO2W_RL_STOCHASTIC:-1}"
export GO2W_RL_STAIR_OBSTACLE_AVOIDANCE="${GO2W_RL_STAIR_OBSTACLE_AVOIDANCE:-1}"
export GO2W_RL_STAIR_PLANNER_YAW_BLEND="${GO2W_RL_STAIR_PLANNER_YAW_BLEND:-0.90}"
export GO2W_RL_STAIR_PLANNER_CENTERING_SCALE="${GO2W_RL_STAIR_PLANNER_CENTERING_SCALE:-0.25}"
export GO2W_RL_STAIR_PLANNER_MIN_VX="${GO2W_RL_STAIR_PLANNER_MIN_VX:-0.18}"
export GO2W_RL_STAIR_PLANNER_MAX_VX="${GO2W_RL_STAIR_PLANNER_MAX_VX:-0.80}"
export GO2W_RL_STAIR_PLANNER_TURN_THRESHOLD="${GO2W_RL_STAIR_PLANNER_TURN_THRESHOLD:-0.06}"
export GO2W_RL_STAIR_OBSTACLE_MIN_Y="${GO2W_RL_STAIR_OBSTACLE_MIN_Y:-2.55}"
export GO2W_RL_STAIR_OBSTACLE_MAX_Y="${GO2W_RL_STAIR_OBSTACLE_MAX_Y:-3.70}"
export ROAMERX_RVIZ_READ_ONLY=1
export ROAMERX_RVIZ_CONFIG="${ROOT_DIR}/config/go2w_extended_navigation.rviz"
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
export COSMOS_VLN_MISSION_REQUIRED_ROUTE_ID="${ROUTE_ID}"
export COSMOS_VLN_INFERENCE_TIMEOUT_SEC="${COSMOS_VLN_INFERENCE_TIMEOUT_SEC:-900}"
export COSMOS_VLN_VISUALIZATION_RVIZ="${COSMOS_VLN_VISUALIZATION_RVIZ:-1}"
export COSMOS_VLN_VISUALIZATION_IMAGE_WINDOW="${COSMOS_VLN_VISUALIZATION_IMAGE_WINDOW:-1}"
export COSMOS_VLN_RVIZ_CONFIG="${ROOT_DIR}/config/cosmos_vln_extended.rviz"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89
export ROS2CLI_NO_DAEMON=1
export ZENOH_ROUTER_CHECK_ATTEMPTS=10

mkdir -p "${ARTIFACT_DIR}"
cp "${ROOT_DIR}/config/cosmos_vln_routes.json" "${ARTIFACT_DIR}/route_catalog.json"
cp "${PHYSICS_SCENE}" "${ARTIFACT_DIR}/physics_scene.xml"
cp "${UE_SCENE}" "${ARTIFACT_DIR}/ue_scene.xml"
cp "${SCENE_JSON}" "${ARTIFACT_DIR}/scene.json"
printf '%s\n' "${MISSION}" >"${ARTIFACT_DIR}/mission.txt"

cd "${ROOT_DIR}"
echo "[INFO] Artifact directory: ${ARTIFACT_DIR}"
"${ROOT_DIR}/scripts/validate_go2w_extended_world.py" \
  >"${ARTIFACT_DIR}/world_contract.log" 2>&1
echo "[INFO] Resetting Go2-W at the centered stair entrance in the extended scene."
./scripts/start_go2w_cosmos_vln.sh
if ! cmp -s "${UE_SCENE}" "${ROOT_DIR}/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/scene_terrain_yard.xml"; then
  echo "[ERROR] UE YardWorld active variant is not the extended scene." >&2
  exit 1
fi
./scripts/start_cosmos_vln_visualization.sh

run_check_with_retries() {
  local check_script="$1"
  local attempt
  for attempt in {1..5}; do
    if "${check_script}"; then
      return 0
    fi
    echo "[WARN] ${check_script} failed during ROS discovery; retry ${attempt}/5." >&2
    sleep 3
  done
  return 1
}

run_check_with_retries ./scripts/check_demo.sh
run_check_with_retries ./scripts/check_cosmos_vln.sh

mission_script="${ROOT_DIR}/scripts/cosmos_vln_mission.py"
mission_pids=()
while IFS= read -r candidate_pid; do
  [[ -r "/proc/${candidate_pid}/cmdline" ]] || continue
  candidate_cmdline="$(tr '\0' ' ' < "/proc/${candidate_pid}/cmdline")"
  candidate_cmdline="${candidate_cmdline% }"
  if [[ "${candidate_cmdline}" == "/usr/bin/python3 ${mission_script}" ]]; then
    mission_pids+=("${candidate_pid}")
  fi
done < <(pgrep -f 'cosmos_vln_mission[.]py' 2>/dev/null || true)
if [[ "${#mission_pids[@]}" -ne 1 ]]; then
  ./scripts/stop_cosmos_vln_mission.sh >/dev/null 2>&1 || true
  echo "[ERROR] Expected exactly one Cosmos mission supervisor; found ${#mission_pids[@]}." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u

echo "[INFO] Rechecking every extended long-route segment with ComputePathToPose."
/usr/bin/python3 "${ROOT_DIR}/scripts/validate_go2w_extended_route.py" \
  --output "${ARTIFACT_DIR}/path_validation.json" \
  >"${ARTIFACT_DIR}/path_validation.log" 2>&1

if [[ -s /tmp/go2w_obstacle_validation.log ]]; then
  cp /tmp/go2w_obstacle_validation.log "${ARTIFACT_DIR}/obstacle_costmap_preflight.log"
fi

/usr/bin/python3 "${ROOT_DIR}/scripts/record_cosmos_vln_run.py" \
  --artifact-dir "${ARTIFACT_DIR}" \
  --timeout-sec "${COSMOS_VLN_MISSION_TIMEOUT_SEC}" \
  >"${ARTIFACT_DIR}/recorder.log" 2>&1 &
recorder_pid=$!
echo "${recorder_pid}" >"${ARTIFACT_DIR}/recorder.pid"
sleep 2
if ! kill -0 "${recorder_pid}" 2>/dev/null; then
  echo "[ERROR] Run recorder failed to start." >&2
  tail -n 100 "${ARTIFACT_DIR}/recorder.log" >&2 || true
  exit 1
fi

echo "[INFO] Submitting the English high-level mission:"
echo "[INFO] ${MISSION}"
ros2 topic pub --once /cosmos_vln/mission std_msgs/msg/String \
  "{data: '${MISSION}'}" >"${ARTIFACT_DIR}/mission_publish.log" 2>&1

set +e
wait "${recorder_pid}"
recorder_status=$?
set -e

cp "${ROOT_DIR}/logs/cosmos_vln_bridge.log" "${ARTIFACT_DIR}/cosmos_vln_bridge.log" 2>/dev/null || true
cp "${ROOT_DIR}/logs/cosmos_vln_mission.log" "${ARTIFACT_DIR}/cosmos_vln_mission.log" 2>/dev/null || true
cp "${ROOT_DIR}/logs/cosmos_vln_visualizer.log" "${ARTIFACT_DIR}/cosmos_vln_visualizer.log" 2>/dev/null || true
cp "${ROOT_DIR}/logs/go2w_systemd.log" "${ARTIFACT_DIR}/go2w_systemd.log" 2>/dev/null || true
cp "${ROOT_DIR}/logs/bridge.log" "${ARTIFACT_DIR}/controller.log" 2>/dev/null || true
cp "${ROOT_DIR}/logs/world_cloud.log" "${ARTIFACT_DIR}/world_cloud.log" 2>/dev/null || true

if [[ "${recorder_status}" -ne 0 ]]; then
  echo "[ERROR] The extended mixed-terrain mission did not succeed." >&2
  echo "[INFO] Evidence: ${ARTIFACT_DIR}" >&2
  tail -n 120 "${ARTIFACT_DIR}/recorder.log" >&2 || true
  exit "${recorder_status}"
fi

if [[ "$(jq -r '.acceptance_passed // false' "${ARTIFACT_DIR}/result.json")" != "true" ]]; then
  echo "[ERROR] Extended route did not pass physical/static/dynamic acceptance: ${ARTIFACT_DIR}" >&2
  exit 1
fi

echo "[OK] Cosmos selected and completed extended route ${ROUTE_ID}."
echo "[OK] Full evidence: ${ARTIFACT_DIR}"
echo "[INFO] Latest world-model visualization: ${ROOT_DIR}/.run/cosmos_vln/latest_visualization.jpg"
