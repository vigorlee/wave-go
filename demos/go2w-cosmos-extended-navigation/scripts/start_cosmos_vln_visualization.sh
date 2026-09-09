#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/demo_env.sh"
require_runtime
source_ros

VISUALIZER_PID_FILE="${RUN_DIR}/cosmos_vln_visualizer.pid"
RVIZ_PID_FILE="${RUN_DIR}/cosmos_vln_rviz.pid"
IMAGE_VIEW_PID_FILE="${RUN_DIR}/cosmos_vln_image_view.pid"
mkdir -p "${RUN_DIR}/cosmos_vln" "${LOG_DIR}"

if [[ -f "${VISUALIZER_PID_FILE}" ]] && kill -0 "$(cat "${VISUALIZER_PID_FILE}")" 2>/dev/null; then
  echo "[ERROR] Cosmos visualizer already running." >&2
  exit 1
fi
nohup setsid /usr/bin/python3 "${DEMO_ROOT}/scripts/cosmos_vln_visualizer.py" \
  >"${LOG_DIR}/cosmos_vln_visualizer.log" 2>&1 &
echo "$!" >"${VISUALIZER_PID_FILE}"
sleep 1
if ! kill -0 "$(cat "${VISUALIZER_PID_FILE}")" 2>/dev/null; then
  echo "[ERROR] Cosmos visualizer failed to start." >&2
  tail -n 80 "${LOG_DIR}/cosmos_vln_visualizer.log" >&2 || true
  exit 1
fi

if [[ "${COSMOS_VLN_VISUALIZATION_RVIZ:-1}" == "1" && -n "${DISPLAY:-}" ]]; then
  nohup setsid bash --noprofile --norc -c \
    "set +u; source /opt/ros/humble/setup.bash; source '${ROAMERX_DIR}/install/setup.bash'; set -u; export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}' ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' QT_X11_NO_MITSHM=1; exec rviz2 -d '${COSMOS_VLN_RVIZ_CONFIG}'" \
    >"${LOG_DIR}/cosmos_vln_rviz.log" 2>&1 &
  echo "$!" >"${RVIZ_PID_FILE}"
fi

if [[ "${COSMOS_VLN_VISUALIZATION_IMAGE_WINDOW:-1}" == "1" && -n "${DISPLAY:-}" ]]; then
  nohup setsid bash --noprofile --norc -c \
    "set +u; source /opt/ros/humble/setup.bash; source '${ROAMERX_DIR}/install/setup.bash'; set -u; export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}' ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' QT_X11_NO_MITSHM=1; exec /usr/bin/python3 '${DEMO_ROOT}/scripts/cosmos_vln_image_view.py'" \
    >"${LOG_DIR}/cosmos_vln_image_view.log" 2>&1 &
  echo "$!" >"${IMAGE_VIEW_PID_FILE}"
fi
echo "[OK] Cosmos visualization started."
