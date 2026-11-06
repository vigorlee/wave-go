#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
systemctl --user stop go2w-navigation.service >/dev/null 2>&1 || true
"${ROOT_DIR}/scripts/stop_demo.sh" >/dev/null 2>&1 || true
echo "[OK] persistent Go2-W stack stopped."
