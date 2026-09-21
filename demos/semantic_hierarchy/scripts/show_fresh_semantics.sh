#!/usr/bin/env bash
set -eo pipefail
SCENE_CODE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SCENE_DATASET_ROOT="${SCENE_DATASET_ROOT:-$SCENE_CODE/outputs/fresh_semantic_20260919}"
export SCENE_SESSION_DIR="${SCENE_SESSION_DIR:-$SCENE_CODE/.runtime/fresh_semantic_view}"
export SCENE_ROS_MASTER="${SCENE_ROS_MASTER:-http://127.0.0.1:11328}"
export SCENE_CHINESE="${SCENE_CHINESE:-1}"
# Add display-only Chinese fields while retaining English ontology keys.
python3 "$SCENE_CODE/scripts/translate_outputs_cn.py" --dataset "$SCENE_DATASET_ROOT" >/dev/null
exec bash "$SCENE_CODE/scripts/three_view_demo.sh" --style fastlio --show-features --mode combined "$@"
