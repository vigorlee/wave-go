#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PID_FILE="${ROOT_DIR}/.run/cosmos_vln_mission.pid"
MISSION_SCRIPT="${ROOT_DIR}/scripts/cosmos_vln_mission.py"

stop_process() {
  local pid="$1"
  if kill -0 "${pid}" 2>/dev/null; then
    kill -TERM "${pid}" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.25
    done
    kill -KILL "${pid}" 2>/dev/null || true
  fi
}

is_mission_process() {
  local pid="$1"
  local command_line
  [[ -r "/proc/${pid}/cmdline" ]] || return 1
  command_line="$(tr '\0' ' ' <"/proc/${pid}/cmdline")"
  [[ "${command_line}" == "/usr/bin/python3 ${MISSION_SCRIPT} " ]]
}

if [[ -f "${PID_FILE}" ]]; then
  pid="$(cat "${PID_FILE}")"
  if is_mission_process "${pid}"; then
    stop_process "${pid}"
  fi
fi
rm -f "${PID_FILE}"

# A stale or missing PID file must not leave an older supervisor subscribed to
# the same mission and Nav2 interfaces.
while read -r pid; do
  [[ -n "${pid}" ]] || continue
  stop_process "${pid}"
done < <(pgrep -f -x "/usr/bin/python3 ${MISSION_SCRIPT}" 2>/dev/null || true)

echo "[OK] Cosmos VLN mission node stopped."
