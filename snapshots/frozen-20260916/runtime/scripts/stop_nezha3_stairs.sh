#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for name in nezha3_sim nezha3_bridge; do
  pid_file="${ROOT_DIR}/.run/${name}.pid"
  if [[ -f "${pid_file}" ]]; then
    pid="$(cat "${pid_file}")"
    if kill -0 "${pid}" 2>/dev/null; then kill -TERM "${pid}" 2>/dev/null || true; fi
    rm -f "${pid_file}"
  fi
done
echo "[OK] Nezha3 stair processes stopped."
