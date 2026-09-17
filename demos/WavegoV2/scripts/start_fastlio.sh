#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/common.sh"
exec python "$V2_ROOT/scripts/run_demo.py" fastlio "$@"
