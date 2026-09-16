#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SOURCE_ROOT="$PROJECT_ROOT/third_party/nezha2_rl_sar/rl_sar_nezha2_0115"
PACKAGE_ROOT="$SOURCE_ROOT/src/robots/nezha_description"
VIEW_ROOT="$PROJECT_ROOT/third_party/nezha2_rl_sar/view"
URDF_PATH="$VIEW_ROOT/nezha2_view.urdf"
RVIZ_CONFIG="$PROJECT_ROOT/config/nezha2_urdf.rviz"
AMENT_VIEW_ROOT="$VIEW_ROOT/ament"
PID_FILE="$PROJECT_ROOT/logs/nezha2_urdf_view.pid"
LOG_DIR="$PROJECT_ROOT/logs/nezha2_urdf_view"

if [[ ! -f "$URDF_PATH" ]]; then
    echo "[ERROR] View URDF not found: $URDF_PATH" >&2
    exit 1
fi

if [[ ! -f "$PACKAGE_ROOT/meshes/trunk.STL" ]]; then
    echo "[ERROR] Nezha2 assets are incomplete: $PACKAGE_ROOT" >&2
    exit 1
fi

if [[ -f "$PID_FILE" ]]; then
    while read -r old_pid; do
        if [[ "$old_pid" =~ ^[0-9]+$ ]] && kill -0 "$old_pid" 2>/dev/null; then
            kill -TERM "$old_pid" 2>/dev/null || true
        fi
    done < "$PID_FILE"
fi
pkill -TERM -f '^/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher --ros-args -r __node:=nezha2_view_state_publisher ' \
    2>/dev/null || true

mkdir -p "$AMENT_VIEW_ROOT/share/ament_index/resource_index/packages" \
    "$AMENT_VIEW_ROOT/share" "$LOG_DIR" "$(dirname "$PID_FILE")"
: > "$AMENT_VIEW_ROOT/share/ament_index/resource_index/packages/nezha_description"
ln -sfn "$PACKAGE_ROOT" "$AMENT_VIEW_ROOT/share/nezha_description"

set +u
source /opt/ros/humble/setup.bash
set -u

export ROS_DOMAIN_ID="${NEZHA2_VIEW_ROS_DOMAIN_ID:-92}"
export AMENT_PREFIX_PATH="$AMENT_VIEW_ROOT${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"

PIDS=()
cleanup() {
    for pid in "${PIDS[@]:-}"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    rm -f "$PID_FILE"
}
trap cleanup EXIT INT TERM HUP

robot_description="$(<"$URDF_PATH")"
/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher \
    --ros-args -r __node:=nezha2_view_state_publisher \
    -p "robot_description:=$robot_description" \
    >"$LOG_DIR/robot_state_publisher.log" 2>&1 &
PIDS+=("$!")

joint_names='[FL_hip_joint, FL_thigh_joint, FL_calf_joint, FL_foot_joint, FR_hip_joint, FR_thigh_joint, FR_calf_joint, FR_foot_joint, RL_hip_joint, RL_thigh_joint, RL_calf_joint, RL_foot_joint, RR_hip_joint, RR_thigh_joint, RR_calf_joint, RR_foot_joint]'
joint_positions='[0.0, 0.925, -1.85, 0.0, 0.0, 0.925, -1.85, 0.0, 0.0, -0.925, 1.85, 0.0, 0.0, -0.925, 1.85, 0.0]'
ros2 topic pub --rate 20 --qos-durability volatile --qos-reliability best_effort \
    /joint_states sensor_msgs/msg/JointState \
    "{header: auto, name: $joint_names, position: $joint_positions}" \
    >"$LOG_DIR/joint_states.log" 2>&1 &
PIDS+=("$!")

printf '%s\n' "${PIDS[@]}" > "$PID_FILE"
sleep 2

if ! kill -0 "${PIDS[0]}" 2>/dev/null; then
    echo "[ERROR] robot_state_publisher failed:" >&2
    tail -n 40 "$LOG_DIR/robot_state_publisher.log" >&2
    exit 1
fi

echo "[INFO] Nezha2 URDF viewer"
echo "[INFO] URDF: $URDF_PATH"
echo "[INFO] ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "[INFO] Opening RViz..."

env -u GTK_PATH -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE \
    -u SNAP -u SNAP_NAME -u SNAP_INSTANCE_NAME \
    rviz2 -d "$RVIZ_CONFIG" --ros-args -r __node:=nezha2_urdf_rviz \
    >"$LOG_DIR/rviz.log" 2>&1 &
PIDS+=("$!")
printf '%s\n' "${PIDS[@]}" > "$PID_FILE"

wait "${PIDS[-1]}"
