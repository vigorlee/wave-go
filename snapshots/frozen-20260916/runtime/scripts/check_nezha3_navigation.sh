#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"${ROOT_DIR}/scripts/check_demo.sh"
if rg -q '^STAIR_SUCCESS ' "${ROOT_DIR}/logs/bridge.log"; then
  echo "[OK] Nezha3 stair result"
  rg '^STAIR_SUCCESS ' "${ROOT_DIR}/logs/bridge.log" | tail -1
else
  echo "[INFO] Nezha3 stair goal not completed yet"
fi
