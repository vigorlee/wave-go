#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="${ROOT_DIR}/.run"
failed=0

pid_is_live() {
  local pid="$1"
  local state

  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  kill -0 "${pid}" 2>/dev/null || return 1
  state="$(ps -o stat= -p "${pid}" 2>/dev/null || true)"
  [[ -n "${state}" && "${state}" != Z* ]]
}

check_tracked_process() {
  local label="$1"
  local pid_file="$2"
  local fallback_pattern="$3"
  local pid=""

  if [[ -e "${pid_file}" ]]; then
    IFS= read -r pid <"${pid_file}" || true
    if pid_is_live "${pid}"; then
      echo "[OK] process: ${label} (tracked pid=${pid})"
    else
      echo "[FAIL] process: ${label} (stale PID file: ${pid_file})"
      failed=1
    fi
    return
  fi

  if pgrep -f -- "${fallback_pattern}" >/dev/null; then
    echo "[OK] process: ${label} (untracked fallback)"
  else
    echo "[FAIL] process: ${label}"
    failed=1
  fi
}

check_tracked_process "G1 RL bridge" \
  "${RUN_DIR}/g1_stair_bridge.pid" '[g]1_rl_bridge([[:space:]]|$)'
check_tracked_process "MATRiX MuJoCo" \
  "${RUN_DIR}/g1_stair_sim.pid" \
  '[r]obot_mujoco(_g1)?([[:space:]]|$)|[r]un_sim\.sh[[:space:]]+g1[[:space:]]+3[01]'
check_tracked_process "Zenoh router" \
  "${RUN_DIR}/g1_stair_zenohd.pid" '[r]mw_zenohd([[:space:]]|$)'

echo
echo "Latest controller state:"
if ! rg 'state z=|stage=|Policy control enabled|SAFETY' "${ROOT_DIR}/logs/g1_stair_bridge.log" | tail -12; then
  echo "[FAIL] no G1 controller state was logged"
  failed=1
fi

if rg -q '^STAIR_FAILED |Fall protection latched' \
  "${ROOT_DIR}/logs/g1_stair_bridge.log"; then
  echo "[FAIL] benchmark: controller reported a failure"
  rg '^STAIR_FAILED |Fall protection latched' \
    "${ROOT_DIR}/logs/g1_stair_bridge.log" | tail -3
  failed=1
elif rg -q '^STAIR_SUCCESS ' "${ROOT_DIR}/logs/g1_stair_bridge.log"; then
  echo "[OK] benchmark: complete up-platform-down traversal"
  rg '^STAIR_SUCCESS ' "${ROOT_DIR}/logs/g1_stair_bridge.log" | tail -1
else
  echo "[INFO] benchmark: still in progress"
fi

exit "${failed}"
