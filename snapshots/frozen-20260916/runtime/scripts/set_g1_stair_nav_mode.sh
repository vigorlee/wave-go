#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
MODE="${1:-status}"

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"
export ROS2CLI_NO_DAEMON="1"

set_param() {
  ros2 param set "$1" "$2" "$3" >/dev/null
}

show_status() {
  echo -n "obstacle_layer="
  ros2 param get /local_costmap/local_costmap obstacle_layer.enabled
  echo -n "obstacle_height="
  ros2 param get /local_costmap/local_costmap obstacle_layer.min_obstacle_height
  ros2 param get /local_costmap/local_costmap obstacle_layer.max_obstacle_height
  echo -n "controller_vx_max="
  ros2 param get /controller_server FollowPath.vx_max
  echo -n "controller_vx_min="
  ros2 param get /controller_server FollowPath.vx_min
  echo -n "velocity_limits="
  ros2 param get /velocity_optimizer max_velocity
}

case "${MODE}" in
  avoid)
    set_param /local_costmap/local_costmap obstacle_layer.enabled true
    # MATRiX course slide 30: keep PointCloud2 obstacles from 0.10 m to 1.20 m.
    set_param /local_costmap/local_costmap obstacle_layer.min_obstacle_height 0.10
    set_param /local_costmap/local_costmap obstacle_layer.max_obstacle_height 1.20
    set_param /local_costmap/local_costmap obstacle_layer.pointcloud.min_obstacle_height 0.10
    set_param /local_costmap/local_costmap obstacle_layer.pointcloud.max_obstacle_height 1.20
    set_param /controller_server FollowPath.vx_max "${AVOID_NAV_VX_MAX:-0.45}"
    set_param /controller_server FollowPath.vx_min "${AVOID_NAV_VX_MIN:--0.15}"
    set_param /controller_server FollowPath.wz_max "${AVOID_NAV_WZ_MAX:-0.60}"
    set_param /controller_server FollowPath.vx_std "${AVOID_NAV_VX_STD:-0.18}"
    set_param /controller_server FollowPath.wz_std "${AVOID_NAV_WZ_STD:-0.45}"
    set_param /velocity_optimizer max_velocity "${AVOID_NAV_MAX_VELOCITY:-[0.45, 0.0, 0.60]}"
    set_param /velocity_optimizer min_velocity "${AVOID_NAV_MIN_VELOCITY:-[-0.15, 0.0, -0.60]}"
    set_param /velocity_optimizer max_accel "${AVOID_NAV_MAX_ACCEL:-[0.70, 0.0, 0.80]}"
    set_param /velocity_optimizer max_decel "${AVOID_NAV_MAX_DECEL:-[-0.90, 0.0, -0.80]}"
    ;;
  up)
    set_param /local_costmap/local_costmap obstacle_layer.enabled true
    set_param /local_costmap/local_costmap obstacle_layer.min_obstacle_height 0.65
    set_param /local_costmap/local_costmap obstacle_layer.max_obstacle_height 1.20
    set_param /local_costmap/local_costmap obstacle_layer.pointcloud.min_obstacle_height 0.65
    set_param /local_costmap/local_costmap obstacle_layer.pointcloud.max_obstacle_height 1.20
    set_param /controller_server FollowPath.vx_max "${STAIR_NAV_VX_MAX:-0.80}"
    set_param /controller_server FollowPath.vx_min "${STAIR_NAV_VX_MIN:-0.55}"
    set_param /controller_server FollowPath.wz_max "${STAIR_NAV_WZ_MAX:-0.35}"
    set_param /controller_server FollowPath.vx_std "${STAIR_NAV_VX_STD:-0.20}"
    set_param /controller_server FollowPath.wz_std "${STAIR_NAV_WZ_STD:-0.40}"
    set_param /velocity_optimizer max_velocity "${STAIR_NAV_MAX_VELOCITY:-[0.80, 0.0, 0.35]}"
    set_param /velocity_optimizer min_velocity "${STAIR_NAV_MIN_VELOCITY:-[-0.20, 0.0, -0.35]}"
    set_param /velocity_optimizer max_accel "${STAIR_NAV_MAX_ACCEL:-[0.80, 0.0, 0.80]}"
    set_param /velocity_optimizer max_decel "${STAIR_NAV_MAX_DECEL:-[-0.80, 0.0, -0.80]}"
    ;;
  down)
    set_param /local_costmap/local_costmap obstacle_layer.enabled true
    set_param /local_costmap/local_costmap obstacle_layer.min_obstacle_height 0.65
    set_param /local_costmap/local_costmap obstacle_layer.max_obstacle_height 1.20
    set_param /local_costmap/local_costmap obstacle_layer.pointcloud.min_obstacle_height 0.65
    set_param /local_costmap/local_costmap obstacle_layer.pointcloud.max_obstacle_height 1.20
    # Reset FollowPath as well as the velocity smoother. A direct up -> down
    # transition must not inherit the uphill minimum forward velocity.
    set_param /controller_server FollowPath.vx_max "${STAIR_DOWN_VX_MAX:-0.35}"
    set_param /controller_server FollowPath.vx_min "${STAIR_DOWN_VX_MIN:--0.15}"
    set_param /controller_server FollowPath.wz_max "${STAIR_DOWN_WZ_MAX:-0.50}"
    set_param /controller_server FollowPath.vx_std "${STAIR_DOWN_VX_STD:-0.16}"
    set_param /controller_server FollowPath.wz_std "${STAIR_DOWN_WZ_STD:-0.40}"
    set_param /velocity_optimizer max_velocity "${STAIR_DOWN_MAX_VELOCITY:-[0.35, 0.0, 0.50]}"
    set_param /velocity_optimizer min_velocity "${STAIR_DOWN_MIN_VELOCITY:-[-0.15, 0.0, -0.50]}"
    set_param /velocity_optimizer max_accel "${STAIR_DOWN_MAX_ACCEL:-[0.60, 0.0, 0.60]}"
    set_param /velocity_optimizer max_decel "${STAIR_DOWN_MAX_DECEL:-[-0.60, 0.0, -0.60]}"
    ;;
  slope)
    # Ignore the load-bearing ramp plane itself while retaining the upper
    # portion of each center-course cylinder in the local obstacle layer.
    # Unlike stair mode this profile permits MPPI to slow down and steer
    # freely, so the robot follows one continuous NavigateThroughPoses path.
    set_param /local_costmap/local_costmap obstacle_layer.enabled true
    set_param /local_costmap/local_costmap obstacle_layer.min_obstacle_height 0.55
    set_param /local_costmap/local_costmap obstacle_layer.max_obstacle_height 1.40
    set_param /local_costmap/local_costmap obstacle_layer.pointcloud.min_obstacle_height 0.55
    set_param /local_costmap/local_costmap obstacle_layer.pointcloud.max_obstacle_height 1.40
    set_param /controller_server FollowPath.vx_max "${SLOPE_NAV_VX_MAX:-0.48}"
    set_param /controller_server FollowPath.vx_min "${SLOPE_NAV_VX_MIN:-0.10}"
    set_param /controller_server FollowPath.wz_max "${SLOPE_NAV_WZ_MAX:-0.55}"
    set_param /controller_server FollowPath.vx_std "${SLOPE_NAV_VX_STD:-0.16}"
    set_param /controller_server FollowPath.wz_std "${SLOPE_NAV_WZ_STD:-0.40}"
    set_param /velocity_optimizer max_velocity "${SLOPE_NAV_MAX_VELOCITY:-[0.48, 0.0, 0.55]}"
    set_param /velocity_optimizer min_velocity "${SLOPE_NAV_MIN_VELOCITY:-[0.05, 0.0, -0.55]}"
    set_param /velocity_optimizer max_accel "${SLOPE_NAV_MAX_ACCEL:-[0.65, 0.0, 0.75]}"
    set_param /velocity_optimizer max_decel "${SLOPE_NAV_MAX_DECEL:-[-0.75, 0.0, -0.75]}"
    ;;
  flat)
    set_param /local_costmap/local_costmap obstacle_layer.enabled true
    set_param /local_costmap/local_costmap obstacle_layer.min_obstacle_height 0.10
    set_param /local_costmap/local_costmap obstacle_layer.max_obstacle_height 1.20
    set_param /local_costmap/local_costmap obstacle_layer.pointcloud.min_obstacle_height 0.10
    set_param /local_costmap/local_costmap obstacle_layer.pointcloud.max_obstacle_height 1.20
    set_param /controller_server FollowPath.vx_max 0.95
    set_param /controller_server FollowPath.vx_min -0.45
    set_param /controller_server FollowPath.wz_max 1.00
    set_param /controller_server FollowPath.vx_std 0.20
    set_param /controller_server FollowPath.wz_std 0.40
    set_param /velocity_optimizer max_velocity "[0.95, 0.0, 1.00]"
    set_param /velocity_optimizer min_velocity "[-0.45, 0.0, -1.00]"
    set_param /velocity_optimizer max_accel "[1.20, 0.0, 1.20]"
    set_param /velocity_optimizer max_decel "[-1.00, 0.0, -1.20]"
    ;;
  status)
    show_status
    exit 0
    ;;
  *)
    echo "Usage: $0 {avoid|up|down|slope|flat|status}" >&2
    exit 2
    ;;
esac

ros2 service call /local_costmap/clear_entirely_local_costmap \
  nav2_msgs/srv/ClearEntireCostmap '{}' >/dev/null
if [[ -n "${STAIR_NAV_MODE_FILE:-}" ]]; then
  mkdir -p "$(dirname "${STAIR_NAV_MODE_FILE}")"
  printf '%s\n' "${MODE}" >"${STAIR_NAV_MODE_FILE}"
fi
echo "[OK] stair navigation mode: ${MODE}"
show_status
