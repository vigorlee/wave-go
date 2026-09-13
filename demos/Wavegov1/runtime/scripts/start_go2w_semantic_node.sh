#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${COSMOS_VLN_ARTIFACT_DIR:-$ROOT_DIR/artifacts/semantic_v1}"
mkdir -p "$OUT/semantic"
systemctl --user stop go2w-semantic-map.service >/dev/null 2>&1 || true
systemd-run --user --quiet --collect --unit=go2w-semantic-map --property=KillMode=control-group \
 --property="WorkingDirectory=$ROOT_DIR" --property="StandardOutput=append:$OUT/semantic.log" --property="StandardError=append:$OUT/semantic.log" \
 /bin/bash -c 'source /opt/ros/humble/setup.bash; source genisom_roamerx_open/install/setup.bash; export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp; exec /usr/bin/python3 scripts/go2w_semantic_map.py --output "$1"' semantic "$OUT/semantic"
