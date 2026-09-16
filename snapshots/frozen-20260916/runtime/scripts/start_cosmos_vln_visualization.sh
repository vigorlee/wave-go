#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
RUN_DIR="${ROOT_DIR}/.run"
LOG_DIR="${ROOT_DIR}/logs"
VISUALIZER_PID_FILE="${RUN_DIR}/cosmos_vln_visualizer.pid"
RVIZ_PID_FILE="${RUN_DIR}/cosmos_vln_rviz.pid"
IMAGE_VIEW_PID_FILE="${RUN_DIR}/cosmos_vln_image_view.pid"

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"

mkdir -p "${RUN_DIR}/cosmos_vln" "${LOG_DIR}"
if [[ -f "${VISUALIZER_PID_FILE}" ]] && \
   kill -0 "$(cat "${VISUALIZER_PID_FILE}")" 2>/dev/null; then
  echo "[ERROR] Cosmos VLN visualizer is already running." >&2
  exit 1
fi

nohup setsid /usr/bin/python3 "${ROOT_DIR}/scripts/cosmos_vln_visualizer.py" \
  >"${LOG_DIR}/cosmos_vln_visualizer.log" 2>&1 &
echo "$!" >"${VISUALIZER_PID_FILE}"
sleep 1
if ! kill -0 "$(cat "${VISUALIZER_PID_FILE}")" 2>/dev/null; then
  echo "[ERROR] Cosmos VLN visualizer failed to start." >&2
  tail -n 80 "${LOG_DIR}/cosmos_vln_visualizer.log" >&2 || true
  exit 1
fi

if [[ "${COSMOS_VLN_VISUALIZATION_RVIZ:-1}" == "1" && -n "${DISPLAY:-}" ]]; then
  nohup env \
    -u SNAP -u SNAP_ARCH -u SNAP_COMMON -u SNAP_CONTEXT -u SNAP_COOKIE \
    -u SNAP_DATA -u SNAP_EUID -u SNAP_INSTANCE_NAME -u SNAP_LIBRARY_PATH \
    -u SNAP_NAME -u SNAP_REAL_HOME -u SNAP_REVISION -u SNAP_UID \
    -u SNAP_USER_COMMON -u SNAP_USER_DATA -u SNAP_VERSION \
    -u SNAP_LAUNCHER_ARCH_TRIPLET \
    -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE -u GTK_PATH -u GTK_MODULES \
    -u QT_ACCESSIBILITY -u QT_IM_MODULE \
    setsid bash --noprofile --norc -c \
    "set +u; source /opt/ros/humble/setup.bash; source '${ROAMERX_DIR}/install/setup.bash'; set -u; export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=89 QT_X11_NO_MITSHM=1; exec rviz2 -d '${COSMOS_VLN_RVIZ_CONFIG:-${ROOT_DIR}/config/cosmos_vln.rviz}'" \
    >"${LOG_DIR}/cosmos_vln_rviz.log" 2>&1 &
  echo "$!" >"${RVIZ_PID_FILE}"
fi

if [[ "${COSMOS_VLN_VISUALIZATION_IMAGE_WINDOW:-1}" == "1" && -n "${DISPLAY:-}" ]]; then
  nohup env \
    -u SNAP -u SNAP_ARCH -u SNAP_COMMON -u SNAP_CONTEXT -u SNAP_COOKIE \
    -u SNAP_DATA -u SNAP_EUID -u SNAP_INSTANCE_NAME -u SNAP_LIBRARY_PATH \
    -u SNAP_NAME -u SNAP_REAL_HOME -u SNAP_REVISION -u SNAP_UID \
    -u SNAP_USER_COMMON -u SNAP_USER_DATA -u SNAP_VERSION \
    -u SNAP_LAUNCHER_ARCH_TRIPLET \
    -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE -u GTK_PATH -u GTK_MODULES \
    -u QT_ACCESSIBILITY -u QT_IM_MODULE \
    setsid bash --noprofile --norc -c \
    "set +u; source /opt/ros/humble/setup.bash; source '${ROAMERX_DIR}/install/setup.bash'; set -u; export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=89 QT_X11_NO_MITSHM=1; exec /usr/bin/python3 '${ROOT_DIR}/scripts/cosmos_vln_image_view.py'" \
    >"${LOG_DIR}/cosmos_vln_image_view.log" 2>&1 &
  echo "$!" >"${IMAGE_VIEW_PID_FILE}"
  sleep 1
  if ! kill -0 "$(cat "${IMAGE_VIEW_PID_FILE}")" 2>/dev/null; then
    echo "[ERROR] Cosmos annotated image window failed to start." >&2
    tail -n 80 "${LOG_DIR}/cosmos_vln_image_view.log" >&2 || true
    exit 1
  fi
fi

echo "[OK] Cosmos VLN visualization started."
echo "[INFO] RViz markers: /cosmos_vln/markers"
echo "[INFO] Annotated image: /cosmos_vln/annotated_image"
echo "[INFO] Latest JPEG: ${RUN_DIR}/cosmos_vln/latest_visualization.jpg"
