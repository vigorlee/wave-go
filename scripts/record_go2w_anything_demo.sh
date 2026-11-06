#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
export COSMOS_VLN_ARTIFACT_DIR="${COSMOS_VLN_ARTIFACT_DIR:-$ROOT_DIR/artifacts/anything_$(date +%Y%m%d_%H%M%S)}"
export GO2W_PRETRAINED=1
export ROAMERX_RVIZ_CONFIG="$ROOT_DIR/config/go2w_anything_navigation.rviz"
mkdir -p "$COSMOS_VLN_ARTIFACT_DIR/anything"
systemctl --user stop go2w-anything-models.service >/dev/null 2>&1 || true
systemd-run --user --quiet --collect --unit=go2w-anything-models --property=KillMode=control-group \
 --property="WorkingDirectory=$ROOT_DIR" --property="StandardOutput=append:$COSMOS_VLN_ARTIFACT_DIR/anything.log" --property="StandardError=append:$COSMOS_VLN_ARTIFACT_DIR/anything.log" \
 /bin/bash "$ROOT_DIR/scripts/run_go2w_anything_service.sh" "$COSMOS_VLN_ARTIFACT_DIR/anything"
cleanup() { local status=$?; if [[ "${GO2W_KEEP_LIVE:-0}" != 1 || "$status" != 0 ]]; then systemctl --user stop go2w-anything-models.service >/dev/null 2>&1 || true; fi; }
trap cleanup EXIT
bash scripts/record_go2w_dense_demo.sh
