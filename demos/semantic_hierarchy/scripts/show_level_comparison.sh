#!/usr/bin/env bash
set -eo pipefail
SCENE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SCENE_RUN="$SCENE_ROOT/outputs/level_fix/fresh_20260919_02"
exec bash "$SCENE_ROOT/scripts/view_level_maps.sh" --new "$SCENE_RUN/structural_calibrated/map_arrays.npz" --measured "$SCENE_RUN/export/map_arrays.npz" "$@"
