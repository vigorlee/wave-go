#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT_DIR}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
ROS_PYTHON_BIN="${ROS_PYTHON_BIN:-/usr/bin/python3}"

"${PYTHON_BIN}" scripts/validate_go2w_extended_world.py
"${PYTHON_BIN}" -m json.tool config/cosmos_vln_routes.json >/dev/null
"${PYTHON_BIN}" -m json.tool config/go2w_terrain_policy_profiles.json >/dev/null
"${PYTHON_BIN}" -m json.tool scene/scene_go2w_extended_nav.json >/dev/null
"${PYTHON_BIN}" -m py_compile scripts/*.py
bash -n scripts/*.sh

if [[ "${WAVE_GO_RUN_ROS_TESTS:-0}" == "1" ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  PYTHONPATH="${ROOT_DIR}/scripts${PYTHONPATH:+:${PYTHONPATH}}" \
    "${ROS_PYTHON_BIN}" -m unittest discover -s tests -p 'test_*.py'
else
  echo "[INFO] ROS regression tests skipped (set WAVE_GO_RUN_ROS_TESTS=1 to run them)."
fi
echo "[OK] portable extended-demo validation passed."
