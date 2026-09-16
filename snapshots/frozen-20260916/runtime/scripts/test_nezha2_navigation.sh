#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BRIDGE_LOG="${ROOT_DIR}/logs/nezha2/bridge.log"
export ROS_DOMAIN_ID="${NEZHA2_ROS_DOMAIN_ID:-90}"

"${ROOT_DIR}/scripts/check_nezha2_navigation.sh"
if [[ ! -f "${BRIDGE_LOG}" ]]; then
  echo "[ERROR] bridge log not found: ${BRIDGE_LOG}" >&2
  exit 1
fi

last_state="$(grep '^state ' "${BRIDGE_LOG}" | tail -1 || true)"
start_y="$(sed -n 's/.* y=\([^ ]*\).*/\1/p' <<<"${last_state}")"
if [[ -z "${start_y}" ]] || ! awk -v y="${start_y}" 'BEGIN { exit !(y < 0.7) }'; then
  echo "[ERROR] benchmark requires a fresh start below the first stair; current y=${start_y:-unknown}." >&2
  echo "[INFO] restart with scripts/stop_nezha2_navigation.sh && scripts/start_nezha2_navigation.sh" >&2
  exit 1
fi

start_line="$(wc -l < "${BRIDGE_LOG}")"
"${ROOT_DIR}/scripts/set_nezha2_stair_nav_mode.sh" up
timeout --signal=TERM --kill-after=10s "${NEZHA2_STAIR_TIMEOUT:-360}" \
  "${ROOT_DIR}/scripts/send_nezha2_goal.sh" 0.65 4.8 1.570796

# Let the controller settle on the platform before checking attitude.
sleep 5

result="$(tail -n "+$((start_line + 1))" "${BRIDGE_LOG}" | grep '^STAIR_SUCCESS ' | tail -1 || true)"
if [[ -z "${result}" ]]; then
  echo "[FAIL] Nav2 returned without a physical stair success marker." >&2
  tail -n 30 "${BRIDGE_LOG}" >&2
  exit 1
fi

final_state="$(grep '^state ' "${BRIDGE_LOG}" | tail -1 || true)"
final_y="$(sed -n 's/.* y=\([^ ]*\).*/\1/p' <<<"${final_state}")"
final_z="$(sed -n 's/.* z=\([^ ]*\).*/\1/p' <<<"${final_state}")"
final_roll="$(sed -n 's/.* roll=\([^ ]*\).*/\1/p' <<<"${final_state}")"
final_pitch="$(sed -n 's/.* pitch=\([^ ]*\).*/\1/p' <<<"${final_state}")"
if ! awk -v y="${final_y:-nan}" -v z="${final_z:-nan}" \
    -v r="${final_roll:-nan}" -v p="${final_pitch:-nan}" \
    'BEGIN { exit !(y > 4.7 && z > 0.9 && r > -0.2 && r < 0.2 && p > -0.2 && p < 0.2) }'; then
  echo "[FAIL] robot reached the marker but did not settle stably on the platform." >&2
  echo "${final_state}" >&2
  exit 1
fi

echo "[OK] Nezha2 MATRiX stair navigation"
echo "${result}"
echo "[OK] platform stable: y=${final_y} z=${final_z} roll=${final_roll} pitch=${final_pitch}"
