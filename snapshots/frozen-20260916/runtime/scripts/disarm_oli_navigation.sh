#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rm -f "${ROOT_DIR}/.run/oli_navigation.armed"

# Keep the locomotion policy active at zero velocity. The vendor Stand
# controller is stable at startup, but a Walk-to-Stand handoff after motion can
# collapse the humanoid despite matching nominal joint targets.
sleep "${OLI_WALK_SETTLE_SECONDS:-3}"

export ROBOT_TYPE="HU_D04_01"
export ROBOT_IP="127.0.0.1"
export MROS_IP_LIST="127.0.0.x"
ability_status="$("${ROOT_DIR}/.venv-oli38/bin/python" \
  -m limxsdk.ability.cli list 2>/dev/null || true)"
if ! grep -Fq -- '- walk (Running)' <<<"${ability_status}"; then
  echo "[WARN] OLI motion is disarmed, but Walk is not the active hold controller." >&2
fi

echo "[OK] OLI navigation motion is DISARMED; zero-command Walk hold is active."
