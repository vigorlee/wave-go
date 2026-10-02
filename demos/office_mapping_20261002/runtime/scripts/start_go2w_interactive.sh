#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export ROAMERX_RVIZ_READ_ONLY="${ROAMERX_RVIZ_READ_ONLY:-0}"

if [[ "${ROAMERX_RVIZ_READ_ONLY}" == "1" ]]; then
  echo "[INFO] RViz is read-only; external goal topics are disabled."
else
  echo "[INFO] RViz interactive goals enabled (/goal_pose)."
fi
exec "${ROOT_DIR}/scripts/start_demo.sh" "$@"
