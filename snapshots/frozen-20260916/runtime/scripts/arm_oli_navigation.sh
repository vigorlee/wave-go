#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${ROOT_DIR}/.venv-oli38/bin/python"
ARM_FILE="${ROOT_DIR}/.run/oli_navigation.armed"

export ROBOT_TYPE="HU_D04_01"
export ROBOT_IP="127.0.0.1"
export MROS_IP_LIST="127.0.0.x"

if [[ ! -f "${ROOT_DIR}/.run/oli_sim.pid" ]] || \
   ! kill -0 "$(cat "${ROOT_DIR}/.run/oli_sim.pid")" 2>/dev/null; then
  echo "[ERROR] OLI simulator is not running." >&2
  exit 1
fi
if ! grep -q 'Ability loaded: walk' "${ROOT_DIR}/logs/oli_control.log" 2>/dev/null; then
  echo "[ERROR] OLI walk policy is not loaded; navigation stays locked." >&2
  exit 1
fi

# A restarted logger can leave a sparse NUL region while the previous process
# closes its descriptor.  Treat the file as text so the newest state line is
# still usable by the upright interlock.
last_state="$(grep -a 'OLI state x=' "${ROOT_DIR}/logs/oli_cmd_vel_bridge.log" | tail -n 1 || true)"
height="$(sed -n 's/.* z=\([-0-9.]*\).*/\1/p' <<<"${last_state}")"
if [[ -z "${height}" ]] || ! awk -v z="${height}" 'BEGIN { exit !(z > 0.70) }'; then
  echo "[ERROR] OLI is not upright (base z=${height:-unknown}); navigation stays locked." >&2
  exit 1
fi

ability_status="$("${PYTHON}" -m limxsdk.ability.cli list 2>/dev/null || true)"
if ! grep -Fq -- '- walk (Running)' <<<"${ability_status}"; then
  "${PYTHON}" -m limxsdk.ability.cli \
    switch 'stand damping mimic' 'walk' >/dev/null
fi

deadline=$((SECONDS + 10))
while true; do
  ability_status="$("${PYTHON}" -m limxsdk.ability.cli list 2>/dev/null || true)"
  if grep -Fq -- '- walk (Running)' <<<"${ability_status}"; then
    break
  fi
  if (( SECONDS >= deadline )); then
    echo "[ERROR] OLI walk controller did not start; navigation stays locked." >&2
    exit 1
  fi
  sleep 0.2
done

# Give the policy history buffer time to settle before accepting Nav2 motion.
sleep 2

if ! kill -0 "$(cat "${ROOT_DIR}/.run/oli_sim.pid")" 2>/dev/null; then
  echo "[ERROR] MuJoCo exited during the Walk handoff." >&2
  exit 1
fi

last_state="$(grep -a 'OLI state x=' "${ROOT_DIR}/logs/oli_cmd_vel_bridge.log" | tail -n 1 || true)"
height="$(sed -n 's/.* z=\([-0-9.]*\).*/\1/p' <<<"${last_state}")"
if [[ -z "${height}" ]] || ! awk -v z="${height}" 'BEGIN { exit !(z > 0.70) }'; then
  echo "[ERROR] OLI lost posture during Walk handoff (z=${height:-unknown})." >&2
  exit 1
fi

touch "${ARM_FILE}"
echo "[OK] OLI navigation motion is ARMED."
