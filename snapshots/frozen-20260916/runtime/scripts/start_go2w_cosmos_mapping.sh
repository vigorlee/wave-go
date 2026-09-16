#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export GO2W_COSMOS_FARTHEST=1
exec bash "$ROOT_DIR/scripts/start_go2w_exploration.sh"
