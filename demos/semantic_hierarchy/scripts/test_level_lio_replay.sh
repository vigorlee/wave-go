#!/usr/bin/env bash
set -eo pipefail
SCENE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SCENE_RUN="$SCENE_ROOT/outputs/level_fix/replay_${1:-gravity30}"
[[ ! -e "$SCENE_RUN" ]] || { echo 'Output exists; use a new run name'; exit 1; }
mkdir -p "$SCENE_RUN"
source /opt/ros/humble/setup.bash
source /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/genisom_roamerx_open/install/setup.bash
source "$SCENE_ROOT/.runtime/level_lio_install/setup.bash"
export ROS_DOMAIN_ID=190 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=1
"$SCENE_ROOT/.runtime/level_lio_install/robot_slam/lib/robot_slam/mapping" --ros-args --params-file "${SCENE_LIO_CONFIG:-$SCENE_ROOT/config/level_lio.yaml}" -r /world_points:=/wavego/level/registered_points -r /slam_odom:=/wavego/level/odom -r /tf:=/wavego/level/tf > "$SCENE_RUN/lio.log" 2>&1 &
lio_pid=$!
/usr/bin/python3 "$SCENE_ROOT/scripts/collect_level_map.py" --output "$SCENE_RUN/capture" > "$SCENE_RUN/collector.log" 2>&1 &
collector_pid=$!
cleanup() { kill -INT "$collector_pid" "$lio_pid" 2>/dev/null || true; wait "$collector_pid" "$lio_pid" 2>/dev/null || true; }
trap cleanup EXIT
ros2 service call /slam_state_service robots_dog_msgs/srv/MapState '{data: 3}' > "$SCENE_RUN/start_service.log" 2>&1
ros2 bag play /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_rosbag_20260916_01/pointcloud_odom_rosbag2 --topics /livox/lidar_raw /imu/data_raw --rate "${SCENE_REPLAY_RATE:-1.0}" --read-ahead-queue-size 1000 > "$SCENE_RUN/replay.log" 2>&1
sleep 2
