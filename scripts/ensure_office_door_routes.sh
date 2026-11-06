#!/usr/bin/env bash
# Solve both upstream Nav2 routes whenever the terrain or mission changes.
set -eo pipefail
cd "$(dirname "$0")/.."
source scripts/office_ros_environment.sh
PLAN_OUTPUT="${1:?A dated output directory is required}"
ROUTES=config/global_compare/door_routes.json
INPUT_HASH="$(sha256sum scripts/global_compare/maps.py scripts/go2w_office_inline_run.py config/global_compare/planners.yaml matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_office_inline.xml | sha256sum | cut -d' ' -f1)"
mkdir -p "$PLAN_OUTPUT"
if [[ -f "$ROUTES" && "$(cat config/global_compare/door_routes.sha256 2>/dev/null)" == "$INPUT_HASH" ]]; then
  cp "$ROUTES" "$PLAN_OUTPUT/routes.json"
  exit 0
fi
bash scripts/global_compare/start_planners.sh
source /opt/ros/humble/setup.bash
source genisom_roamerx_open/install/setup.bash
export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp
/usr/bin/python3 scripts/global_compare/maps.py
/usr/bin/python3 scripts/global_compare/plan_routes.py --output "$PLAN_OUTPUT"
cp "$PLAN_OUTPUT/routes.json" "$ROUTES"
printf '%s\n' "$INPUT_HASH" > config/global_compare/door_routes.sha256
