#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

resolve_workspace() {
  local arg_ws="${1:-}"
  if [ -n "${arg_ws}" ] && [ -d "${arg_ws}" ]; then
    printf '%s\n' "$(cd "${arg_ws}" && pwd)"
    return 0
  fi

  if [ -n "${GENISOM_ROAMERX_OPEN_WORKSPACE:-}" ] && [ -d "${GENISOM_ROAMERX_OPEN_WORKSPACE}" ]; then
    printf '%s\n' "$(cd "${GENISOM_ROAMERX_OPEN_WORKSPACE}" && pwd)"
    return 0
  fi

  if [ -n "${ROAMERX_OPEN_WORKSPACE:-}" ] && [ -d "${ROAMERX_OPEN_WORKSPACE}" ]; then
    printf '%s\n' "$(cd "${ROAMERX_OPEN_WORKSPACE}" && pwd)"
    return 0
  fi

  if [ -n "${ROAMERX_LITE_WORKSPACE:-}" ] && [ -d "${ROAMERX_LITE_WORKSPACE}" ]; then
    printf '%s\n' "$(cd "${ROAMERX_LITE_WORKSPACE}" && pwd)"
    return 0
  fi

  local candidates=(
    "${SCRIPT_DIR}/../../genisom_roamerx_open"
    "${SCRIPT_DIR}/../genisom_roamerx_open"
    "${SCRIPT_DIR}/../../../genisom_roamerx_open"
    "${SCRIPT_DIR}/../../zsibot_roamerx_lite"
    "${SCRIPT_DIR}/../zsibot_roamerx_lite"
    "${SCRIPT_DIR}/../../../zsibot_roamerx_lite"
    "${SCRIPT_DIR}/../../zsibot_roamer-x_lite"
    "${SCRIPT_DIR}/../zsibot_roamer-x_lite"
    "${SCRIPT_DIR}/../../../zsibot_roamer-x_lite"
  )

  local candidate
  for candidate in "${candidates[@]}"; do
    if [ -d "${candidate}" ]; then
      printf '%s\n' "$(cd "${candidate}" && pwd)"
      return 0
    fi
  done

  return 1
}

resolve_forward_install() {
  local workspace_candidates=()
  local prefer_system="${ROAMERX_FORWARD_PREFER_SYSTEM:-1}"

  if [ "${prefer_system}" != "1" ] && [ -n "${ROAMERX_FORWARD_WORKSPACE:-}" ] && [ -d "${ROAMERX_FORWARD_WORKSPACE}" ]; then
    workspace_candidates+=("${ROAMERX_FORWARD_WORKSPACE}")
  fi

  if [ "${prefer_system}" = "1" ]; then
    if [ -f "/opt/robot/robot-forward/install/setup.bash" ] && [ -x "/opt/robot/robot-forward/install/robot_forward/lib/robot_forward/robot_forward" ]; then
      printf '%s|%s\n' \
        "/opt/robot/robot-forward/install/setup.bash" \
        "/opt/robot/robot-forward/install/robot_forward/lib/robot_forward/robot_forward"
      return 0
    fi
  fi

  workspace_candidates+=(
    "${SCRIPT_DIR}/../../forward"
    "${SCRIPT_DIR}/../forward"
    "${SCRIPT_DIR}/../../../forward"
  )

  local workspace
  for workspace in "${workspace_candidates[@]}"; do
    if [ -f "${workspace}/install/setup.bash" ] && [ -x "${workspace}/install/robot_forward/lib/robot_forward/robot_forward" ]; then
      printf '%s|%s\n' \
        "$(cd "${workspace}" && pwd)/install/setup.bash" \
        "$(cd "${workspace}" && pwd)/install/robot_forward/lib/robot_forward/robot_forward"
      return 0
    fi
  done

  if [ "${prefer_system}" != "1" ] && [ -n "${ROAMERX_FORWARD_WORKSPACE:-}" ] && [ -d "${ROAMERX_FORWARD_WORKSPACE}" ]; then
    if [ -f "${ROAMERX_FORWARD_WORKSPACE}/install/setup.bash" ] && [ -x "${ROAMERX_FORWARD_WORKSPACE}/install/robot_forward/lib/robot_forward/robot_forward" ]; then
      printf '%s|%s\n' \
        "$(cd "${ROAMERX_FORWARD_WORKSPACE}" && pwd)/install/setup.bash" \
        "$(cd "${ROAMERX_FORWARD_WORKSPACE}" && pwd)/install/robot_forward/lib/robot_forward/robot_forward"
      return 0
    fi
  fi

  if [ -f "${ROAMERX_FORWARD_SETUP:-}" ] && [ -x "${ROAMERX_FORWARD_BIN:-}" ]; then
    printf '%s|%s\n' "${ROAMERX_FORWARD_SETUP}" "${ROAMERX_FORWARD_BIN}"
    return 0
  fi

  if [ -f "/opt/robot/robot-forward/install/setup.bash" ] && [ -x "/opt/robot/robot-forward/install/robot_forward/lib/robot_forward/robot_forward" ]; then
    printf '%s|%s\n' \
      "/opt/robot/robot-forward/install/setup.bash" \
      "/opt/robot/robot-forward/install/robot_forward/lib/robot_forward/robot_forward"
    return 0
  fi

  return 1
}

WORKSPACE_DIR="$(resolve_workspace "${1:-}" )" || {
  echo "[ERROR] GENISOM RoamerX Open workspace not found."
  echo "        Put genisom_roamerx_open beside matrix or export GENISOM_ROAMERX_OPEN_WORKSPACE."
  exit 1
}

MATRIX_BIN_DIR="${SCRIPT_DIR}/../bin"
STOP_NAV_SCRIPT="${WORKSPACE_DIR}/script/bash/stop_navigation.sh"
LOG_DIR="${MATRIX_BIN_DIR}"
STATE_FILE="${MATRIX_BIN_DIR}/roamerx_link.state"
LINK_LOG="${MATRIX_BIN_DIR}/roamerx_link.log"
PUBTF_LAUNCH="ros2 launch pub_tf pub_tf.launch.py tf_type:=mujoco_tf"
PUBTF_SETUP="${WORKSPACE_DIR}/install/setup.bash"
ODOM_BRIDGE_SCRIPT="${SCRIPT_DIR}/roamerx_odom_bridge.py"
# Disable composition here because repeated start/stop under rmw_zenoh_cpp
# can leave the component load service timing out on navigo_container.
ROAMERX_COMMUNICATION_TYPE="${ROAMERX_COMMUNICATION_TYPE:-LCM}"
# The active simulator launcher supplies a map whose collision geometry is
# derived from the same MATRiX scene.  Fall back to the project map for the
# stock launch path.
ROAMERX_MAP="${ROAMERX_MAP:-${WORKSPACE_DIR}/map/map.yaml}"
NAV_LAUNCH="ros2 launch robot_navigo navigation_bringup.launch.py platform:=UE mc_controller_type:=RL_TRACK_VELOCITY communication_type:=${ROAMERX_COMMUNICATION_TYPE} use_composition:=False map:='${ROAMERX_MAP}'"
ROAMERX_RVIZ_READ_ONLY="${ROAMERX_RVIZ_READ_ONLY:-0}"
RVIZ_GOAL_REMAPS=""
if [ "${ROAMERX_RVIZ_READ_ONLY}" = "1" ]; then
  RVIZ_GOAL_REMAPS="--ros-args -r /goal_pose:=/rviz_goal_pose_disabled -r /initialpose:=/rviz_initialpose_disabled"
fi
# Codex/VS Code may be launched from a Snap sandbox.  Its GTK variables make
# host RViz load core20 libraries, which is incompatible with the host glibc.
# Allow a robot/scene launcher to select an RViz profile.  The Nezha2 profile
# renders the static warehouse map, global plan, controller plan, local/global
# costmaps and the actual UE ray-cast PointCloud2 on /livox/lidar.
ROAMERX_RVIZ_CONFIG="${ROAMERX_RVIZ_CONFIG:-${WORKSPACE_DIR}/src/navigation/src/robot_navigo/rviz/rviz2_config.rviz}"
RVIZ_LAUNCH="env -u SNAP -u SNAP_ARCH -u SNAP_COMMON -u SNAP_CONTEXT -u SNAP_COOKIE -u SNAP_DATA -u SNAP_EUID -u SNAP_INSTANCE_NAME -u SNAP_LIBRARY_PATH -u SNAP_NAME -u SNAP_REAL_HOME -u SNAP_REVISION -u SNAP_UID -u SNAP_USER_COMMON -u SNAP_USER_DATA -u SNAP_VERSION -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE -u GTK_PATH -u GTK_MODULES rviz2 -d '${ROAMERX_RVIZ_CONFIG}' ${RVIZ_GOAL_REMAPS}"

FORWARD_INSTALL_INFO="$(resolve_forward_install || true)"
ROAMERX_FORWARD_SETUP="${FORWARD_INSTALL_INFO%%|*}"
ROAMERX_FORWARD_BIN="${FORWARD_INSTALL_INFO#*|}"
ROAMERX_FORWARD_CMD="${ROAMERX_FORWARD_CMD:-}"
ROAMERX_USE_ODOM_BRIDGE="${ROAMERX_USE_ODOM_BRIDGE:-0}"
ROAMERX_STOP_ON_RVIZ_EXIT="${ROAMERX_STOP_ON_RVIZ_EXIT:-0}"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"
export SDK_CLIENT_IP="${SDK_CLIENT_IP:-127.0.0.1}"

mkdir -p "${LOG_DIR}"
exec > >(tee -a "${LINK_LOG}") 2>&1

echo "[INFO] run_roamerx_lite_link.sh started at $(date '+%F %T')"

is_running() {
  pgrep -f "$1" >/dev/null 2>&1
}

zenoh_port_in_use() {
  if command -v ss >/dev/null 2>&1; then
    ss -ltn '( sport = :7447 )' 2>/dev/null | grep -q '7447'
  else
    is_running "rmw_zenohd"
  fi
}

ros_package_available() {
  local package="$1"
  [ -f "${WORKSPACE_DIR}/install/setup.bash" ] || return 1
  bash -lc "set +u; source '${WORKSPACE_DIR}/install/setup.bash' >/dev/null 2>&1 && ros2 pkg prefix '${package}' >/dev/null 2>&1"
}

is_link_running() {
  [ -f "${STATE_FILE}" ] || return 1

  local alive=0
  while IFS='=' read -r key pid; do
    [ -z "${key}" ] && continue
    [ -z "${pid}" ] && continue
    if kill -0 "${pid}" >/dev/null 2>&1; then
      alive=1
      break
    fi
  done < "${STATE_FILE}"

  [ "${alive}" -eq 1 ]
}

start_background() {
  local label="$1"
  local logfile="$2"
  shift 2

  mkdir -p "${LOG_DIR}"
  # The simulator launcher returns after constructing the linked stack.  Use
  # a separate session so ROS navigation is not terminated with that parent
  # shell; this matches the MATRiX physics launcher's process contract.
  nohup setsid bash -lc "$*" > "${logfile}" 2>&1 &
  local pid=$!
  echo "[INFO] started ${label}, pid=${pid}, log -> ${logfile}" >&2
  printf '%s\n' "${pid}"
}

cleanup_linked_stack() {
  local reason="${1:-manual}"

  echo "[INFO] Cleaning up linked stack (reason=${reason})"

  if [ -f "${STATE_FILE}" ]; then
    echo "[INFO] Loaded state file ${STATE_FILE}"
    while IFS='=' read -r key pid; do
      [ -z "${key}" ] && continue
      [ -z "${pid}" ] && continue
      if kill -0 "${pid}" >/dev/null 2>&1; then
        echo "[INFO] killing ${key} pid=${pid}"
        kill -9 "${pid}" >/dev/null 2>&1 || true
      else
        echo "[INFO] ${key} pid=${pid} already exited"
      fi
    done < "${STATE_FILE}"
    rm -f "${STATE_FILE}"
  fi

  pkill -f 'component_container_isolated.*__node:=navigo_container' >/dev/null 2>&1 || true
  pkill -f 'navigation_bringup.launch.py' >/dev/null 2>&1 || true
  pkill -f 'robot_navigo/.*/vel_cmd_udp_pub' >/dev/null 2>&1 || true
  pkill -f 'robot_navigo/.*/vel_cmd_lcm_pub' >/dev/null 2>&1 || true
  pkill -f 'robot_navigo/.*/mode_status_pub' >/dev/null 2>&1 || true
  pkill -f 'vel_cmd_udp_pub --ros-args' >/dev/null 2>&1 || true
  pkill -f 'vel_cmd_lcm_pub --ros-args' >/dev/null 2>&1 || true
  pkill -f 'mode_status_pub --ros-args' >/dev/null 2>&1 || true
  pkill -f '/navigo_map_server/lib/navigo_map_server/map_server' >/dev/null 2>&1 || true
  pkill -f '/navigo_path_controller/lib/navigo_path_controller/controller_server' >/dev/null 2>&1 || true
  pkill -f '/navigo_path_planner/lib/navigo_path_planner/planner_server' >/dev/null 2>&1 || true
  pkill -f '/navigo_behaviors/lib/navigo_behaviors/behavior_server' >/dev/null 2>&1 || true
  pkill -f '/navigo_velocity_optimizer/lib/navigo_velocity_optimizer/velocity_smoother' >/dev/null 2>&1 || true
  pkill -f '/navigo_bt_navigator/lib/navigo_bt_navigator/bt_navigator' >/dev/null 2>&1 || true
  pkill -f '/navigo_waypoint_follower/lib/navigo_waypoint_follower/waypoint_follower' >/dev/null 2>&1 || true
  pkill -f 'lifecycle_manager_navigation' >/dev/null 2>&1 || true
  pkill -f 'rviz2 -d .*/rviz2_config.rviz' >/dev/null 2>&1 || true
  pkill -f 'robot_forward' >/dev/null 2>&1 || true
  pkill -f 'pub_tf.launch.py' >/dev/null 2>&1 || true
  pkill -f 'pub_tf' >/dev/null 2>&1 || true
  pkill -f 'roamerx_odom_bridge.py' >/dev/null 2>&1 || true

  # Do not invoke the workspace's broad stop_navigation.sh here.  It uses
  # unrestricted `pgrep -f navigation_bringup` / `pgrep -f rviz2` searches
  # and can terminate the caller while it is shutting down a linked stack.
  # The PID state file and the explicit node patterns above already cover the
  # processes started by this launcher.

  echo "[INFO] RoamerX linked stack stopped."

  # Give Zenoh/ROS graph a moment to withdraw stale endpoints before the next start.
  sleep 2
}

start_rviz_monitor() {
  local rviz_pid="$1"
  (
    while kill -0 "${rviz_pid}" >/dev/null 2>&1; do
      sleep 1
    done
    echo "[INFO] rviz pid=${rviz_pid} exited; auto-stopping linked stack..."
    cleanup_linked_stack "rviz-exit"
  ) >>"${LINK_LOG}" 2>&1 &
  local watcher_pid=$!
  echo "[INFO] started rviz watcher, pid=${watcher_pid}" >&2
}

start_linked_stack() {
  mkdir -p "${LOG_DIR}"
  rm -f "${STATE_FILE}"

  echo "[INFO] RoamerX workspace: ${WORKSPACE_DIR}"
  echo "[INFO] Stop script: ${STOP_NAV_SCRIPT}"
  echo "[INFO] State file: ${STATE_FILE}"

  local zenohd_pid pubtf_pid odom_bridge_pid forward_pid nav_pid rviz_pid

  if zenoh_port_in_use; then
    echo "[WARN] rmw_zenohd port 7447 already in use; reuse existing zenohd and skip launching a new one."
    zenohd_pid=""
  else
    zenohd_pid="$(start_background "rmw_zenohd" "${LOG_DIR}/roamerx_zenohd.log" \
      "set +u; cd '${WORKSPACE_DIR}' && source install/setup.bash && exec ros2 run rmw_zenoh_cpp rmw_zenohd")"
  fi

  if ! is_running "ros2 launch pub_tf pub_tf.launch.py"; then
    if ros_package_available "pub_tf"; then
      if [ ! -f "${PUBTF_SETUP}" ]; then
        echo "[ERROR] pub_tf setup not found: ${PUBTF_SETUP}"
        echo "[ERROR] Please ensure pub_tf is built in the RoamerX workspace."
        return 1
      fi
      pubtf_pid="$(start_background "pub_tf" "${LOG_DIR}/roamerx_pub_tf.log" \
        "set +u; cd '${WORKSPACE_DIR}' && source install/setup.bash && exec ${PUBTF_LAUNCH}")"
    else
      echo "[WARN] Optional pub_tf package not found; skip it. robot_forward /robot_tf will publish TF from /odom/mujoco_odom when MATRiX is running."
      pubtf_pid=""
    fi
  else
    echo "[INFO] pub_tf already running; skip launching a new one."
    pubtf_pid=""
  fi

  if [ -n "${ROAMERX_FORWARD_CMD}" ]; then
    forward_pid="$(start_background "robot_forward" "${LOG_DIR}/roamerx_forward.log" \
      "${ROAMERX_FORWARD_CMD}")"
  elif [ -n "${ROAMERX_FORWARD_SETUP}" ] && [ -n "${ROAMERX_FORWARD_BIN}" ] && [ -f "${ROAMERX_FORWARD_SETUP}" ] && [ -x "${ROAMERX_FORWARD_BIN}" ]; then
    echo "[INFO] using robot_forward from ${ROAMERX_FORWARD_BIN}"
    forward_pid="$(start_background "robot_forward" "${LOG_DIR}/roamerx_forward.log" \
      "set +u; source '${ROAMERX_FORWARD_SETUP}' && exec '${ROAMERX_FORWARD_BIN}'")"
  else
    echo "[WARN] robot_forward binary or setup not found, skip it."
    forward_pid=""
  fi

  if [ -z "${forward_pid:-}" ] && [ "${ROAMERX_USE_ODOM_BRIDGE}" = "1" ]; then
    if [ ! -x "${ODOM_BRIDGE_SCRIPT}" ]; then
      echo "[ERROR] odom bridge script is missing or not executable: ${ODOM_BRIDGE_SCRIPT}"
      return 1
    fi

    if ! is_running "roamerx_odom_bridge.py"; then
      odom_bridge_pid="$(start_background "roamerx_odom_bridge" "${LOG_DIR}/roamerx_odom_bridge.log" \
        "set +u; cd '${WORKSPACE_DIR}' && source install/setup.bash && exec '${ODOM_BRIDGE_SCRIPT}'")"
    else
      echo "[INFO] roamerx_odom_bridge already running; skip launching a new one."
      odom_bridge_pid=""
    fi
  else
    odom_bridge_pid=""
  fi

  nav_pid="$(start_background "navigation stack" "${LOG_DIR}/roamerx_nav.log" \
    "set +u; cd '${WORKSPACE_DIR}' && source install/setup.bash && exec ${NAV_LAUNCH}")"

  rviz_pid="$(start_background "rviz2" "${LOG_DIR}/roamerx_rviz.log" \
    "set +u; cd '${WORKSPACE_DIR}' && source install/setup.bash && exec ${RVIZ_LAUNCH}")"

  {
    echo "zenohd=${zenohd_pid}"
    echo "pub_tf=${pubtf_pid:-}"
    echo "odom_bridge=${odom_bridge_pid:-}"
    echo "robot_forward=${forward_pid:-}"
    echo "navigation=${nav_pid}"
    echo "rviz=${rviz_pid}"
  } > "${STATE_FILE}"

  echo "[INFO] linked stack state saved to ${STATE_FILE}"
  if [ "${ROAMERX_STOP_ON_RVIZ_EXIT}" = "1" ]; then
    start_rviz_monitor "${rviz_pid}"
  fi
}

stop_linked_stack() {
  cleanup_linked_stack "manual"
}

case "${2:-toggle}" in
  start)
    echo "[INFO] run_roamerx_lite_link.sh invoked with workspace=${WORKSPACE_DIR} mode=start"
    if is_link_running; then
      echo ">>> RoamerX linked stack already running, keeping it alive."
    else
      echo ">>> Starting RoamerX linked stack..."
      start_linked_stack
    fi
    ;;
  toggle)
    echo "[INFO] run_roamerx_lite_link.sh invoked with workspace=${WORKSPACE_DIR} mode=toggle"
    if is_link_running; then
      echo ">>> RoamerX linked stack already running (state file present), stopping..."
      stop_linked_stack
    else
      echo ">>> Starting RoamerX linked stack..."
      start_linked_stack
    fi
    ;;
  stop)
    echo "[INFO] run_roamerx_lite_link.sh invoked with workspace=${WORKSPACE_DIR} mode=stop"
    echo ">>> Stopping RoamerX linked stack..."
    stop_linked_stack
    ;;
  print)
    echo "[INFO] run_roamerx_lite_link.sh invoked with workspace=${WORKSPACE_DIR} mode=print"
    cat <<EOF
# Terminal 0: matrix simulation first
cd matrix_clean/bin && ./sim_launcher

# Terminal 1: linked RoamerX stack
bash scripts/run_roamerx_lite_link.sh "${WORKSPACE_DIR}"
EOF
    ;;
  *)
    echo "Usage: $0 <roamerx_workspace> [toggle|start|stop|print]"
    exit 1
    ;;
esac
