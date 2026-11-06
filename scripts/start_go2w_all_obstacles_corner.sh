#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
systemctl --user stop go2w-course.service go2w-mapping-sidecar.service go2w-exploration-debug.service >/dev/null 2>&1 || true
export GO2W_MAPLESS=0 GO2W_COSMOS_FARTHEST=0
export GO2W_RL_STAIR_APPROACH_ASSIST=1 GO2W_RL_STAIR_ALIGNMENT_MAX_HEADING=.40 GO2W_RL_STAIR_ALIGNMENT_VX=.70
export COSMOS_VLN_ARTIFACT_DIR="${COSMOS_VLN_ARTIFACT_DIR:-$ROOT_DIR/artifacts/all_obstacles_corner_$(date +%Y%m%d_%H%M%S)}"
export ROAMERX_RVIZ_CONFIG="${ROAMERX_RVIZ_CONFIG:-$ROOT_DIR/config/go2w_corner_mapping.rviz}"
export COSMOS_VLN_ROUTES_FILE="$ROOT_DIR/config/go2w_all_obstacles_corner.json"
export COSMOS_VLN_MISSION_REQUIRED_ROUTE_ID=all_obstacles_to_east_corner
export COSMOS_VLN_JOBS_DIR="$COSMOS_VLN_ARTIFACT_DIR/cosmos_jobs"
export COSMOS_VLN_MAX_NEW_TOKENS=1536
mkdir -p "$COSMOS_VLN_ARTIFACT_DIR/course" "$COSMOS_VLN_ARTIFACT_DIR/mapping"
bash scripts/start_go2w_three_scenes_live.sh
systemd-run --user --quiet --collect --unit=go2w-mapping-sidecar --property=KillMode=control-group \
 --property="WorkingDirectory=$ROOT_DIR" --property="StandardOutput=append:$COSMOS_VLN_ARTIFACT_DIR/mapping.log" --property="StandardError=append:$COSMOS_VLN_ARTIFACT_DIR/mapping.log" \
 --setenv="GO2W_MAPPING_OUTPUT=$COSMOS_VLN_ARTIFACT_DIR/mapping" \
 /bin/bash -c 'source /opt/ros/humble/setup.bash; source genisom_roamerx_open/install/setup.bash; export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp; exec ros2 launch scripts/go2w_mapping_sidecar.launch.py'
if [[ "${GO2W_ENABLE_SEMANTIC_MAP:-0}" == "1" ]]; then bash scripts/start_go2w_semantic_node.sh; fi
if [[ "${GO2W_SEMANTIC_PREVIEW_ONLY:-0}" == "1" ]]; then exit 0; fi
bash scripts/start_cosmos_vln_bridge.sh
systemd-run --user --quiet --collect --unit=go2w-course --property=KillMode=control-group --property=KillSignal=SIGINT \
 --property="WorkingDirectory=$ROOT_DIR" --property="StandardOutput=append:$COSMOS_VLN_ARTIFACT_DIR/course.log" --property="StandardError=append:$COSMOS_VLN_ARTIFACT_DIR/course.log" \
 --setenv="COSMOS_VLN_ROUTES_FILE=$COSMOS_VLN_ROUTES_FILE" --setenv=GO2W_COURSE_ROUTE_ID=all_obstacles_to_east_corner --setenv=GO2W_COURSE_USE_COSMOS=1 \
 --setenv=COSMOS_VLN_MISSION_REQUIRED_ROUTE_ID=all_obstacles_to_east_corner --setenv=COSMOS_VLN_MISSION_TIMEOUT_SEC=900 \
 /bin/bash -c 'source /opt/ros/humble/setup.bash; source genisom_roamerx_open/install/setup.bash; export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp; exec /usr/bin/python3 scripts/go2w_continuous_course.py --output "$1"' corner "$COSMOS_VLN_ARTIFACT_DIR/course"
echo "[OK] Cosmos full-obstacle mission started; results: $COSMOS_VLN_ARTIFACT_DIR"
