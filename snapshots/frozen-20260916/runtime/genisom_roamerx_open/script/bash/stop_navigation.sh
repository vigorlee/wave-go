#!/bin/bash
# ============================================================
# 终止 ROS2 Navigation Stack 与 RViz2 相关进程
# ============================================================

echo ">>> 正在查找并终止导航相关进程..."

# Kill the ros2 launch wrapper first so it does not respawn children.
PROCESS_NAMES=(
  "ros2 launch robot_navigo navigation_bringup.launch.py"
  "navigation_bringup"
  "rviz2"
)

for pname in "${PROCESS_NAMES[@]}"; do
  pids=$(pgrep -f "$pname")
  if [ -n "$pids" ]; then
    echo ">>> 终止进程: $pname (PID: $pids)"
    kill -9 $pids 2>/dev/null
  else
    echo ">>> 未检测到进程: $pname"
  fi
done

# Kill child nodes that become orphaned when the launch wrapper is killed.
# These must be stopped before the next start or lifecycle_manager will
# reach stale nodes and fail with "Failed to change state".
NAV_PATTERNS=(
  "navigo_map_server/lib/navigo_map_server/map_server"
  "navigo_path_controller/lib/navigo_path_controller/controller_server"
  "navigo_path_planner/lib/navigo_path_planner/planner_server"
  "navigo_behaviors/lib/navigo_behaviors/behavior_server"
  "navigo_velocity_optimizer/lib/navigo_velocity_optimizer/velocity_smoother"
  "navigo_bt_navigator/lib/navigo_bt_navigator/bt_navigator"
  "navigo_waypoint_follower/lib/navigo_waypoint_follower/waypoint_follower"
  "lifecycle_manager_navigation"
  "vel_cmd_udp_pub --ros-args"
  "mode_status_pub --ros-args"
  "component_container_isolated.*navigo_container"
)

for pat in "${NAV_PATTERNS[@]}"; do
  pkill -9 -f "${pat}" 2>/dev/null && echo ">>> 已清理: ${pat}" || true
done

echo "  所有导航相关进程已终止。"
