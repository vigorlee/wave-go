#!/usr/bin/env bash
# Recreate a lost private router; never stop a router belonging to another run.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$(dirname "$0")/office_ros_environment.sh"
export ROS_DOMAIN_ID=489 RMW_IMPLEMENTATION=rmw_zenoh_cpp
while true; do
  if ! ss -H -ltn 'sport = :7759' | grep -q .; then
    printf '%s Restoring private ROS router on port 7759\n' "$(date --iso-8601=seconds)"
    /opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd || true
  fi
  sleep 1
done
