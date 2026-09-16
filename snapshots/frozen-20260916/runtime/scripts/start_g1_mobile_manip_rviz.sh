#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
ROAMERX_LINK="${ROOT_DIR}/matrix/scripts/run_roamerx_lite_link.sh"
PYTHON_BIN="${MATRIX_G1_PYTHON:-/home/unitree/miniconda3/envs/unitree_lerobot/bin/python}"
BRIDGE="${ROOT_DIR}/scripts/g1_mobile_manip_ros_bridge.py"
MAP_FILE="${ROOT_DIR}/config/g1_stairs_map.yaml"
PARAMS_FILE="${ROOT_DIR}/config/g1_roamerx_params.yaml"
RUN_DIR="${ROOT_DIR}/.run"
LOG_DIR="${ROOT_DIR}/logs"
BRIDGE_PID_FILE="${RUN_DIR}/g1_mobile_manip_ros_bridge.pid"
BRIDGE_LOG="${LOG_DIR}/g1_mobile_manip_ros_bridge.log"

DURATION="${G1_RVIZ_DURATION:-3600}"
HEADLESS_RUNTIME=0
START_RVIZ=1
REPLACE=0

while (($#)); do
  case "$1" in
    --duration)
      [[ $# -ge 2 ]] || { echo "[ERROR] --duration requires seconds" >&2; exit 2; }
      DURATION="$2"; shift 2 ;;
    --headless-runtime) HEADLESS_RUNTIME=1; shift ;;
    --no-rviz) START_RVIZ=0; shift ;;
    --replace) REPLACE=1; shift ;;
    *)
      echo "Usage: $0 [--duration SECONDS] [--headless-runtime] [--no-rviz] [--replace]" >&2
      exit 2 ;;
  esac
done

mkdir -p "${RUN_DIR}" "${LOG_DIR}"

if (( REPLACE )); then
  "${ROOT_DIR}/scripts/stop_g1_mobile_manip_rviz.sh" >/dev/null 2>&1 || true
fi

if [[ -f "${BRIDGE_PID_FILE}" ]] && kill -0 "$(cat "${BRIDGE_PID_FILE}")" 2>/dev/null; then
  echo "[ERROR] G1 ROS bridge is already running, pid=$(cat "${BRIDGE_PID_FILE}")" >&2
  echo "[ERROR] Use --replace to restart the complete stack." >&2
  exit 1
fi
if [[ -f "${RUN_DIR}/g1_mobile_manip.pid" ]] && kill -0 "$(cat "${RUN_DIR}/g1_mobile_manip.pid")" 2>/dev/null; then
  echo "[ERROR] G1 full-body runtime is already running." >&2
  echo "[ERROR] Use --replace to restart it in ROS/RViz mode." >&2
  exit 1
fi
if [[ ! -x "${PYTHON_BIN}" || ! -f "${BRIDGE}" || ! -f "${MAP_FILE}" || ! -f "${PARAMS_FILE}" ]]; then
  echo "[ERROR] G1 ROS integration prerequisites are missing." >&2
  exit 1
fi

runtime_args=(--manual --realtime --duration "${DURATION}")
(( HEADLESS_RUNTIME )) && runtime_args+=(--headless)
"${ROOT_DIR}/scripts/start_g1_mobile_manip.sh" "${runtime_args[@]}"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"

ROAMERX_MAP="${MAP_FILE}" \
ROAMERX_PARAMS_FILE="${PARAMS_FILE}" \
ROAMERX_COMMUNICATION_TYPE=ROS \
ROAMERX_SKIP_PUBTF=1 \
ROAMERX_SKIP_FORWARD=1 \
ROAMERX_USE_ODOM_BRIDGE=0 \
ROAMERX_START_RVIZ="${START_RVIZ}" \
bash "${ROAMERX_LINK}" "${ROAMERX_DIR}" start

nohup bash -lc "set +u; source /opt/ros/humble/setup.bash && source '${ROAMERX_DIR}/install/setup.bash' && export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}' ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' && exec '${PYTHON_BIN}' '${BRIDGE}'" \
  >"${BRIDGE_LOG}" 2>&1 &
bridge_pid=$!
echo "${bridge_pid}" >"${BRIDGE_PID_FILE}"

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u

ready=0
for _ in {1..100}; do
  if ! kill -0 "${bridge_pid}" 2>/dev/null; then
    break
  fi
  if timeout 1 ros2 topic echo /odom/mujoco_odom --once >/dev/null 2>&1; then
    ready=1
    break
  fi
  sleep 0.1
done
if (( ! ready )); then
  echo "[ERROR] G1 ROS bridge did not publish odometry." >&2
  tail -80 "${BRIDGE_LOG}" >&2 || true
  "${ROOT_DIR}/scripts/stop_g1_mobile_manip_rviz.sh" >/dev/null 2>&1 || true
  exit 1
fi

echo "[OK] G1 full-body ROS 2 / RoamerX / RViz stack started."
echo "[INFO] G1 stair navigation parameters loaded from ${PARAMS_FILE}"
echo "[INFO] RViz 2D Goal Pose can target the stair corridor, for example x=7.2 y=0.0."
echo "[INFO] ROS upper body: /g1/upper_pose, /g1/joint_targets, /g1/payload_action"
echo "[INFO] bridge log=${BRIDGE_LOG}"
echo "[INFO] inspect: ./scripts/check_g1_mobile_manip_rviz.sh"
echo "[INFO] stop:    ./scripts/stop_g1_mobile_manip_rviz.sh"
