#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
COSMOS_ROOT="/home/unitree/matrix_g1_lcm_demo"
MISSION="Complete the full stair course: climb to the upper landing, descend the far staircase, then continue to the farthest safe endpoint. Do not stop at the upper landing."

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

# Use the active desktop by default so UE, read-only RViz, and the annotated
# camera viewer are visible when launched from a local or SSH terminal.
export DISPLAY="${DISPLAY:-:0}"
if [[ -z "${XAUTHORITY:-}" && -r "/run/user/$(id -u)/gdm/Xauthority" ]]; then
  export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
fi

export MATRIX_ROBOT_INITIAL_X=0.2
export MATRIX_ROBOT_INITIAL_Y=0.0
export MATRIX_ROBOT_INITIAL_YAW_DEG=90
export MATRIX_UE_OFFSCREEN="${MATRIX_UE_OFFSCREEN:-1}"
export MATRIX_UE_NO_RHI_THREAD="${MATRIX_UE_NO_RHI_THREAD:-1}"
export MATRIX_CAMERA_FREQUENCY="${MATRIX_CAMERA_FREQUENCY:-1}"
export ROAMERX_RVIZ_READ_ONLY=1
export COSMOS_VLN_MISSION_TIMEOUT_SEC="${COSMOS_VLN_MISSION_TIMEOUT_SEC:-900}"
export COSMOS_VLN_MISSION_REQUIRED_ROUTE_ID=far_end_via_stairs
export COSMOS_VLN_INFERENCE_TIMEOUT_SEC="${COSMOS_VLN_INFERENCE_TIMEOUT_SEC:-900}"
export COSMOS_VLN_COMMAND_WAIT_SEC="${COSMOS_VLN_COMMAND_WAIT_SEC:-930}"
export COSMOS_VLN_VISUALIZATION_RVIZ=1
export COSMOS_VLN_VISUALIZATION_IMAGE_WINDOW=1

cd "${ROOT_DIR}"

echo "[INFO] Resetting the robot at the centered stair entrance."
./scripts/start_go2w_cosmos_vln.sh
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

mapfile -t mission_pids < <(
  pgrep -f -x "/usr/bin/python3 ${ROOT_DIR}/scripts/cosmos_vln_mission.py" \
    2>/dev/null || true
)
if [[ "${#mission_pids[@]}" -ne 1 ]]; then
  ./scripts/stop_cosmos_vln_mission.sh >/dev/null 2>&1 || true
  echo "[ERROR] Expected exactly one Cosmos mission supervisor; found ${#mission_pids[@]}." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89

echo "[INFO] Submitting the English high-level mission:"
echo "[INFO] ${MISSION}"
ros2 topic pub --once /cosmos_vln/mission std_msgs/msg/String \
  "{data: '${MISSION}'}"

echo "[INFO] Waiting for the Cosmos route command."
if ! route_message="$(
  timeout "${COSMOS_VLN_COMMAND_WAIT_SEC}" \
    ros2 topic echo --once --full-length \
      /cosmos_vln/route_command std_msgs/msg/String
)"; then
  ros2 topic pub --once /cosmos_vln/mission_cancel std_msgs/msg/Empty "{}" \
    >/dev/null 2>&1 || true
  echo "[ERROR] Cosmos did not return a route command before the timeout." >&2
  exit 1
fi

if ! route_selection="$(
  printf '%s\n' "${route_message}" | /usr/bin/python3 -c '
import json
import sys
import yaml

documents = [item for item in yaml.safe_load_all(sys.stdin.read()) if item]
if len(documents) != 1 or not isinstance(documents[0], dict):
    raise SystemExit("invalid ROS message")
payload = json.loads(documents[0]["data"])
print(payload.get("action", ""))
print(payload.get("route_id") or "")
'
)"; then
  ros2 topic pub --once /cosmos_vln/mission_cancel std_msgs/msg/Empty "{}" \
    >/dev/null 2>&1 || true
  echo "[ERROR] Cosmos returned an unreadable route command." >&2
  exit 1
fi

mapfile -t route_fields <<<"${route_selection}"
action="${route_fields[0]:-}"
route_id="${route_fields[1]:-}"
if [[ "${action}" != "navigate_to_route" || \
      "${route_id}" != "far_end_via_stairs" ]]; then
  ros2 topic pub --once /cosmos_vln/mission_cancel std_msgs/msg/Empty "{}" \
    >/dev/null 2>&1 || true
  echo "[ERROR] Unexpected Cosmos command: action=${action:-null} route_id=${route_id:-null}." >&2
  echo "[ERROR] The mission was cancelled instead of executing a partial route." >&2
  exit 1
fi

echo "[OK] Cosmos selected navigate_to_route / far_end_via_stairs."
echo "[OK] The verified three-stage mission is running; the stack remains active."
echo "[INFO] Mission status: ros2 topic echo --full-length /cosmos_vln/mission_status"
echo "[INFO] Route command: ros2 topic echo --full-length /cosmos_vln/route_command"
echo "[INFO] Latest visualization: ${ROOT_DIR}/.run/cosmos_vln/latest_visualization.jpg"
