#! /usr/bin/env bash
set -euo pipefail

# ============================================================
# 启动 / 关闭 ROS2 Navigation Stack 与 RViz2
# - 无参数时：切换模式（运行则关闭，不运行则启动）
# - nav / rviz / all / print：保留原有手动模式
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORKSPACE_DIR="${PROJECT_DIR}"
MAP_PATH="${PROJECT_DIR}/map/map.yaml"
RVIZ_CONFIG="${PROJECT_DIR}/src/navigation/src/robot_navigo/rviz/rviz2_config.rviz"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"
export SDK_CLIENT_IP="${SDK_CLIENT_IP:-127.0.0.1}"
export COMMUNICATION_TYPE="${COMMUNICATION_TYPE:-LCM}"

on_interrupt() {
  echo "[INFO] Caught interrupt, stopping child processes..."
  pkill -P $$ >/dev/null 2>&1 || true
  exit 130
}

trap on_interrupt INT TERM

if [ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  echo "【ERROR】找不到 ${WORKSPACE_DIR}/install/setup.bash"
  echo "请先执行: colcon build"
  exit 1
fi

print_usage() {
  echo "Usage: $0 {toggle|nav|rviz|all|stop|print}"
  echo "  toggle: 默认模式；若导航已运行则关闭，否则启动"
  echo "  nav   : 启动导航栈（当前终端前台运行）"
  echo "  rviz  : 启动 RViz2（当前终端前台运行）"
  echo "  all   : 有 gnome-terminal 时自动开两个终端（nav+rviz）；无则后台启动"
  echo "  stop  : 关闭导航栈与 RViz2"
  echo "  print : 打印 UE 推荐的三终端命令（sim+nav+rviz）"
}

is_roamer_running() {
  pgrep -f "ros2 launch robot_navigo navigation_bringup.launch.py" >/dev/null 2>&1 || \
    pgrep -f "rviz2 -d ${RVIZ_CONFIG}" >/dev/null 2>&1 || \
    pgrep -f "rviz2" >/dev/null 2>&1
}

start_nav() {
  set +u
  source "${WORKSPACE_DIR}/install/setup.bash"
  set -u
  exec ros2 launch robot_navigo navigation_bringup.launch.py \
    platform:=UE \
    mc_controller_type:=RL_TRACK_VELOCITY \
    communication_type:="${COMMUNICATION_TYPE}" \
    map:="${MAP_PATH}"
}

start_rviz() {
  set +u
  source "${WORKSPACE_DIR}/install/setup.bash"
  set -u
  exec env \
    -u SNAP -u SNAP_ARCH -u SNAP_COMMON -u SNAP_CONTEXT -u SNAP_COOKIE \
    -u SNAP_DATA -u SNAP_EUID -u SNAP_INSTANCE_NAME -u SNAP_LIBRARY_PATH \
    -u SNAP_NAME -u SNAP_REAL_HOME -u SNAP_REVISION -u SNAP_UID \
    -u SNAP_USER_COMMON -u SNAP_USER_DATA -u SNAP_VERSION \
    -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE -u GTK_PATH -u GTK_MODULES \
    rviz2 -d "${RVIZ_CONFIG}"
}

start_detached_stack() {
  mkdir -p "${PROJECT_DIR}/log"
  echo "[INFO] Starting RoamerX Lite detached stack..."

  nohup bash -lc "cd '${PROJECT_DIR}' && source install/setup.bash && exec ros2 launch robot_navigo navigation_bringup.launch.py platform:=UE mc_controller_type:=RL_TRACK_VELOCITY communication_type:='${COMMUNICATION_TYPE}' map:='${MAP_PATH}'" \
    > "${PROJECT_DIR}/log/roamerx_nav.log" 2>&1 &

  nohup bash -lc "cd '${PROJECT_DIR}' && source install/setup.bash && exec env -u SNAP -u SNAP_ARCH -u SNAP_COMMON -u SNAP_CONTEXT -u SNAP_COOKIE -u SNAP_DATA -u SNAP_EUID -u SNAP_INSTANCE_NAME -u SNAP_LIBRARY_PATH -u SNAP_NAME -u SNAP_REAL_HOME -u SNAP_REVISION -u SNAP_UID -u SNAP_USER_COMMON -u SNAP_USER_DATA -u SNAP_VERSION -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE -u GTK_PATH -u GTK_MODULES rviz2 -d '${RVIZ_CONFIG}'" \
    > "${PROJECT_DIR}/log/roamerx_rviz.log" 2>&1 &

  echo "[INFO] RoamerX Lite detached stack started."
}

stop_roamer_stack() {
  if [ -x "${SCRIPT_DIR}/stop_navigation.sh" ]; then
    bash "${SCRIPT_DIR}/stop_navigation.sh"
  else
    echo "[ERROR] stop_navigation.sh not found: ${SCRIPT_DIR}/stop_navigation.sh" >&2
    exit 1
  fi
}

print_split_commands() {
  echo "# 终端0：启动 MATRiX 仿真（提供 odom）"
  echo "cd ../matrix && bash run_sim.sh 1 3"
  echo
  echo "# 终端1：启动导航栈"
  echo "bash script/bash/start_navigation.sh nav"
  echo
  echo "# 终端2：启动 RViz2"
  echo "bash script/bash/start_navigation.sh rviz"
}

MODE="${1:-toggle}"

case "${MODE}" in
  toggle|start)
    if is_roamer_running; then
      echo ">>> RoamerX Lite 已在运行，正在关闭..."
      stop_roamer_stack
    else
      echo ">>> RoamerX Lite 未运行，正在启动..."
      start_detached_stack
    fi
    ;;
  nav)
    echo ">>> Launching Navigation Stack..."
    start_nav
    ;;
  rviz)
    echo ">>> Launching RViz2..."
    start_rviz
    ;;
  stop)
    echo ">>> Stopping Navigation Stack..."
    stop_roamer_stack
    ;;
  print)
    print_split_commands
    ;;
  all)
    if command -v gnome-terminal >/dev/null 2>&1; then
      gnome-terminal -- bash -c "cd '${PROJECT_DIR}' && bash script/bash/start_navigation.sh nav; exec bash"
      gnome-terminal -- bash -c "cd '${PROJECT_DIR}' && bash script/bash/start_navigation.sh rviz; exec bash"
    else
      start_detached_stack
    fi
    ;;
  *)
    print_usage
    exit 1
    ;;
esac
