#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export GO2W_ENABLE_SEMANTIC_MAP=1 GO2W_SEMANTIC_FRAME=lio_map GO2W_SEMANTIC_ODOM_TOPIC=/wavego/navigation/odom
export ROAMERX_RVIZ_CONFIG="$ROOT_DIR/config/go2w_semantic_map.rviz"
export COSMOS_VLN_ARTIFACT_DIR="${COSMOS_VLN_ARTIFACT_DIR:-$ROOT_DIR/artifacts/semantic_v1_$(date +%Y%m%d_%H%M%S)}"
exec bash "$ROOT_DIR/scripts/start_go2w_all_obstacles_corner.sh"
