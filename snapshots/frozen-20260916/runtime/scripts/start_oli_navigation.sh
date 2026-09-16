#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="${ROOT_DIR}/.run"
LOG_DIR="${ROOT_DIR}/logs"
PYTHON="${ROOT_DIR}/.venv-oli310/bin/python"

if [[ -e "${RUN_DIR}/go2w_navigation.lock" ]]; then
  echo "[ERROR] Go2-W navigation is active; stop it before starting OLI navigation." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/genisom_roamerx_open/install/setup.bash"
source "${ROOT_DIR}/install-oli-description/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="89"
export ROBOT_TYPE="HU_D04_01"
export ROBOT_IP="127.0.0.1"
export MROS_IP_LIST="127.0.0.x"
export OLI_CMD_VEL_PORT="${OLI_CMD_VEL_PORT:-15001}"
export OLI_ROS_STATE_PORT="${OLI_ROS_STATE_PORT:-15002}"
export OLI_NAV_ARM_FILE="${ROOT_DIR}/.run/oli_navigation.armed"
export OLI_NAV_MAX_VX="${OLI_NAV_MAX_VX:-0.08}"
export OLI_NAV_MIN_FORWARD_VX="${OLI_NAV_MIN_FORWARD_VX:-0.08}"
export OLI_NAV_MAX_REVERSE_VX="${OLI_NAV_MAX_REVERSE_VX:-0.04}"
export OLI_NAV_MAX_VY="${OLI_NAV_MAX_VY:-0.0}"
export OLI_NAV_MAX_WZ="${OLI_NAV_MAX_WZ:-0.30}"
export OLI_NAV_MIN_TURN_WZ="${OLI_NAV_MIN_TURN_WZ:-0.24}"
export OLI_NAV_LINEAR_ACCEL="${OLI_NAV_LINEAR_ACCEL:-0.10}"
export OLI_NAV_LINEAR_DECEL="${OLI_NAV_LINEAR_DECEL:-0.20}"
export OLI_NAV_ANGULAR_ACCEL="${OLI_NAV_ANGULAR_ACCEL:-0.30}"
export OLI_NAV_BASE_YAW_OFFSET_DEG="${OLI_NAV_BASE_YAW_OFFSET_DEG:-0.0}"

MAP_FILE="${OLI_NAV_MAP:-${ROOT_DIR}/genisom_roamerx_open/map/map.yaml}"
URDF_FILE="${ROOT_DIR}/install-oli-description/HU_D04_description/share/HU_D04_description/urdf/HU_D04_01.urdf"
PARAMS_FILE="$(${PYTHON} "${ROOT_DIR}/scripts/prepare_oli_navigation_params.py")"

for name in oli_sim oli_control; do
  if [[ ! -f "${RUN_DIR}/${name}.pid" ]] || \
     ! kill -0 "$(cat "${RUN_DIR}/${name}.pid")" 2>/dev/null; then
    echo "[ERROR] OLI is not running; start it with scripts/start_oli_demo.sh first." >&2
    exit 1
  fi
done

mkdir -p "${RUN_DIR}" "${LOG_DIR}"
rm -f "${OLI_NAV_ARM_FILE}"

# Remove stale navigation/RViz nodes from a previous MATRIX run without touching
# the active OLI simulator and controller.
bash "${ROOT_DIR}/matrix/scripts/run_roamerx_lite_link.sh" \
  "${ROOT_DIR}/genisom_roamerx_open" stop >/dev/null 2>&1 || true

if ! pgrep -f 'rmw_zenoh_cpp/rmw_zenohd|rmw_zenoh_cpp rmw_zenohd' \
    >/dev/null 2>&1; then
  nohup setsid ros2 run rmw_zenoh_cpp rmw_zenohd \
    >"${LOG_DIR}/oli_zenohd.log" 2>&1 &
  echo "$!" >"${RUN_DIR}/oli_zenohd.pid"
  sleep 2
fi

if [[ -f "${RUN_DIR}/oli_cmd_vel_bridge.pid" ]]; then
  old_pid="$(cat "${RUN_DIR}/oli_cmd_vel_bridge.pid")"
  kill -TERM "${old_pid}" 2>/dev/null || true
fi

nohup setsid "${PYTHON}" "${ROOT_DIR}/scripts/oli_cmd_vel_bridge.py" \
  >"${LOG_DIR}/oli_cmd_vel_bridge.log" 2>&1 &
echo "$!" >"${RUN_DIR}/oli_cmd_vel_bridge.pid"

deadline=$((SECONDS + 10))
while ! grep -q 'Forwarding /cmd_vel' "${LOG_DIR}/oli_cmd_vel_bridge.log" 2>/dev/null; do
  if ! kill -0 "$(cat "${RUN_DIR}/oli_cmd_vel_bridge.pid")" 2>/dev/null; then
    echo "[ERROR] OLI cmd_vel bridge exited." >&2
    cat "${LOG_DIR}/oli_cmd_vel_bridge.log" >&2 || true
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "[ERROR] Timed out waiting for the OLI cmd_vel bridge." >&2
    exit 1
  fi
  sleep 0.2
done

deadline=$((SECONDS + 10))
while ! grep -q 'OLI state x=' "${LOG_DIR}/oli_cmd_vel_bridge.log" 2>/dev/null; do
  if (( SECONDS >= deadline )); then
    echo "[ERROR] No OLI MuJoCo state reached ROS; restart scripts/start_oli_demo.sh." >&2
    exit 1
  fi
  sleep 0.2
done

if ! kill -0 "$(cat "${RUN_DIR}/oli_cmd_vel_bridge.pid")" 2>/dev/null; then
  echo "[ERROR] OLI cmd_vel bridge did not remain running." >&2
  cat "${LOG_DIR}/oli_cmd_vel_bridge.log" >&2 || true
  exit 1
fi
if ! kill -0 "$(cat "${RUN_DIR}/oli_sim.pid")" 2>/dev/null; then
  echo "[ERROR] MuJoCo exited after the Walk handoff; velocity commands are disabled." >&2
  kill -TERM "$(cat "${RUN_DIR}/oli_cmd_vel_bridge.pid")" 2>/dev/null || true
  exit 1
fi

nohup setsid ros2 run robot_state_publisher robot_state_publisher "${URDF_FILE}" \
  >"${LOG_DIR}/oli_robot_state_publisher.log" 2>&1 &
echo "$!" >"${RUN_DIR}/oli_robot_state_publisher.pid"

nohup setsid ros2 launch robot_navigo navigation_bringup.launch.py \
  platform:=UE mc_controller_type:=RL_TRACK_VELOCITY communication_type:=LCM \
  use_composition:=False use_sim_time:=False map:="${MAP_FILE}" \
  params_file:="${PARAMS_FILE}" \
  >"${LOG_DIR}/oli_navigation.log" 2>&1 &
echo "$!" >"${RUN_DIR}/oli_nav.pid"

if [[ "${OLI_DISABLE_RVIZ:-0}" != "1" ]]; then
  nohup setsid env \
    -u SNAP -u SNAP_ARCH -u SNAP_COMMON -u SNAP_CONTEXT -u SNAP_COOKIE \
    -u SNAP_DATA -u SNAP_EUID -u SNAP_INSTANCE_NAME -u SNAP_LIBRARY_PATH \
    -u SNAP_NAME -u SNAP_REAL_HOME -u SNAP_REVISION -u SNAP_UID \
    -u SNAP_USER_COMMON -u SNAP_USER_DATA -u SNAP_VERSION \
    -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE -u GTK_PATH -u GTK_MODULES \
    rviz2 -d "${ROOT_DIR}/genisom_roamerx_open/src/navigation/src/robot_navigo/rviz/rviz2_config.rviz" \
    >"${LOG_DIR}/oli_rviz.log" 2>&1 &
  echo "$!" >"${RUN_DIR}/oli_rviz.pid"
fi

deadline=$((SECONDS + 30))
while (( SECONDS < deadline )); do
  if timeout 3 ros2 action list 2>/dev/null | grep -Fxq /navigate_to_pose; then
    break
  fi
  if ! kill -0 "$(cat "${RUN_DIR}/oli_nav.pid")" 2>/dev/null; then
    echo "[ERROR] Navigo exited during startup." >&2
    tail -n 100 "${LOG_DIR}/oli_navigation.log" >&2 || true
    exit 1
  fi
  sleep 1
done

if ! timeout 3 ros2 action list 2>/dev/null | grep -Fxq /navigate_to_pose; then
  echo "[ERROR] /navigate_to_pose did not become ready." >&2
  tail -n 100 "${LOG_DIR}/oli_navigation.log" >&2 || true
  exit 1
fi

echo "[OK] /cmd_vel is connected but DISARMED; OLI remains in Stand."
echo "[INFO] Flat-ground limits after arming: vx=0.08 vy=0.00 wz=0.30"
echo "[INFO] Navigo params: ${PARAMS_FILE}"
echo "[INFO] Bridge log: ${LOG_DIR}/oli_cmd_vel_bridge.log"
echo "[OK] RViz and /navigate_to_pose are ready for the full OLI model."
echo "[INFO] Arm motion explicitly with: scripts/arm_oli_navigation.sh"
