#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
X="${1:-0.0}"
Y="${2:-0.8}"
YAW="${3:-1.570796327}"
WITH_MATRIX_UE="${OLI_TEST_WITH_MATRIX_UE:-1}"
KEEP_RUNNING="${OLI_TEST_KEEP_RUNNING:-0}"

cleanup() {
  local status=$?
  if [[ -e "${ROOT_DIR}/.run/oli_navigation.armed" ]]; then
    "${ROOT_DIR}/scripts/disarm_oli_navigation.sh" >/dev/null 2>&1 || true
  fi
  if (( status != 0 )) || [[ "${KEEP_RUNNING}" != "1" ]]; then
    "${ROOT_DIR}/scripts/stop_oli_demo.sh" >/dev/null 2>&1 || true
  fi
  return "${status}"
}
trap cleanup EXIT

"${ROOT_DIR}/scripts/stop_oli_demo.sh" >/dev/null 2>&1 || true
OLI_START_MATRIX_UE="${WITH_MATRIX_UE}" \
  "${ROOT_DIR}/scripts/start_oli_demo.sh"
"${ROOT_DIR}/scripts/start_oli_navigation.sh"
"${ROOT_DIR}/scripts/arm_oli_navigation.sh"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/genisom_roamerx_open/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="89"

"${ROOT_DIR}/.venv-oli310/bin/python" \
  "${ROOT_DIR}/scripts/oli_navigation_benchmark.py" \
  "${X}" "${Y}" "${YAW}"

"${ROOT_DIR}/scripts/disarm_oli_navigation.sh"
"${ROOT_DIR}/.venv-oli310/bin/python" \
  "${ROOT_DIR}/scripts/oli_navigation_hold_check.py" \
  --duration "${OLI_TEST_HOLD_SECONDS:-10}" \
  --result "${ROOT_DIR}/artifacts/oli_navigation/latest_result.json"
echo "[OK] OLI flat-ground navigation benchmark passed."
echo "[INFO] Result: ${ROOT_DIR}/artifacts/oli_navigation/latest_result.json"
if [[ "${KEEP_RUNNING}" == "1" ]]; then
  echo "[INFO] MATRIX/RViz remain open for inspection."
fi
