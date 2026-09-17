#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/common.sh"
pcd="${1:-$V2_ROOT/data/office_map.pcd}"
if [[ $# -gt 0 ]]; then shift; fi
out="$V2_ROOT/.run/grid"
mkdir -p "$out"
env -u PYTHONPATH -u LD_LIBRARY_PATH "${WAVEGO_V2_ANALYSIS_PYTHON:-$V2_ENV/bin/python}" "$V2_ROOT/scripts/build_adaptive_voxels.py" "$pcd" "$out"
exec python "$V2_ROOT/scripts/run_demo.py" grid "$@"
