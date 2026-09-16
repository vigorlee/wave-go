#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FEEDBACK=()
if [[ "${1:-}" == "--feedback" ]]; then
  FEEDBACK=(--feedback)
elif [[ -n "${1:-}" ]]; then
  echo "Usage: $0 [--feedback]" >&2
  exit 2
fi
# YardWorld's adapted Nezha3 staircase runs along world +Y.
exec "${ROOT_DIR}/scripts/send_goal.sh" 0.2 4.8 1.570796 "${FEEDBACK[@]}"
