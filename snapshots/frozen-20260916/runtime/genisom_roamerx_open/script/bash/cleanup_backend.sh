#!/usr/bin/env bash

set -euo pipefail

CONTAINER_NAME="${1:-zsibot_roamerx_matrix007_jbz}"

run_cleanup() {
	set +e

	# 1) graceful stop
	pkill -f "ros2 launch robot_navigo navigation_bringup.launch.py" || true
	pkill -f "component_container_isolated" || true
	pkill -f "vel_cmd_udp_pub|mode_status_pub" || true
	pkill -f "custom_odom_baselink_node|odom_to_tf_broadcaster" || true
	pkill -f "run_sim.sh|rviz2|gzserver|gzclient" || true
	sleep 1

	# 2) force stop
	pkill -9 -f "ros2 launch robot_navigo navigation_bringup.launch.py" || true
	pkill -9 -f "component_container_isolated" || true
	pkill -9 -f "vel_cmd_udp_pub|mode_status_pub" || true
	pkill -9 -f "custom_odom_baselink_node|odom_to_tf_broadcaster" || true
	pkill -9 -f "run_sim.sh|rviz2|gzserver|gzclient" || true

	# 3) stop ros2 daemon
	source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true
	ros2 daemon stop >/dev/null 2>&1 || true

	# 4) verify (no output means clean)
	echo "[cleanup] verify:"
	ps -eo pid,ppid,stat,cmd | grep -E "navigation_bringup|component_container_isolated|vel_cmd_udp_pub|mode_status_pub|custom_odom_baselink_node|odom_to_tf_broadcaster|run_sim.sh|rviz2|gzserver|gzclient|ros2-daemon|\\[ros2\\] <defunct>" | grep -v grep || true
}

if [[ -f /.dockerenv ]]; then
	echo "[cleanup] running inside container"
	run_cleanup
else
	echo "[cleanup] target container: ${CONTAINER_NAME}"
	docker exec -u 0 "${CONTAINER_NAME}" bash -lc "$(declare -f run_cleanup); run_cleanup"
fi

echo "[cleanup] done"
