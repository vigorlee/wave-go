#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# Native Backspace reset crashes this packaged simulator. Use a fresh simulation
# and mapping session, keeping every old recording and point cloud untouched.
context="$(cat .run/go2w/manual_context 2>/dev/null || true)"
if [[ "$context" == global_compare ]]; then
  bash scripts/start_office_global_demo.sh "office_global_recovery_$(date +%Y%m%d_%H%M%S)"
else
  bash scripts/start_office_inline_demo.sh "office_recovery_$(date +%Y%m%d_%H%M%S)"
fi
systemd-run --user --quiet --collect --unit=go2w-manual --property="WorkingDirectory=$PWD" /bin/bash "$PWD/scripts/start_go2w_manual.sh"
