#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${ROOT_DIR}/logs"
UNIT="go2w-navigation.service"

mkdir -p "${LOG_DIR}"
systemctl --user stop "${UNIT}" >/dev/null 2>&1 || true
"${ROOT_DIR}/scripts/stop_demo.sh" >/dev/null 2>&1 || true
systemctl --user reset-failed "${UNIT}" >/dev/null 2>&1 || true

: >"${LOG_DIR}/go2w_systemd.log"
SYSTEMD_ENV=()
for name in GO2W_RL_STAIR_ALIGNMENT_MAX_HEADING GO2W_RL_STAIR_ALIGNMENT_VX MATRIX_LIDAR_X MATRIX_LIDAR_Y MATRIX_LIDAR_Z MATRIX_LIDAR_ROLL MATRIX_LIDAR_PITCH MATRIX_LIDAR_YAW MATRIX_LIDAR_FREQUENCY MATRIX_MUJOCO_EXIT_TRACE MATRIX_UE_RES_X MATRIX_UE_RES_Y MATRIX_UE_MAX_FPS GO2W_RL_STAIR_ROUTE_CENTER_X GO2W_RL_STAIR_OUTBOUND_MIN_Y GO2W_RL_STAIR_OUTBOUND_MAX_Y GO2W_RL_STAIR_OUTBOUND_FAST_Y GO2W_RL_STAIR_OUTBOUND_SLOW_Y GO2W_RL_STAIR_RETURN_MIN_Y GO2W_RL_STAIR_RETURN_MAX_Y GO2W_RL_STAIR_EXIT_VX ROAMERX_MAP MATRIX_UE_EXTRA_EXEC_COMMANDS GO2W_NATIVE_PEOPLE MATRIX_MAP_ID MATRIX_ROBOT_INITIAL_X MATRIX_ROBOT_INITIAL_Y MATRIX_ROBOT_INITIAL_YAW_DEG \
            MATRIX_UE_OFFSCREEN MATRIX_UE_NO_RHI_THREAD ROAMERX_RVIZ_READ_ONLY ROAMERX_RVIZ_CONFIG \
            MATRIX_DISABLE_IMAGE_SENSORS \
            MATRIX_CAMERA_WIDTH MATRIX_CAMERA_HEIGHT MATRIX_CAMERA_FREQUENCY \
            MATRIX_DEPTH_WIDTH MATRIX_DEPTH_HEIGHT MATRIX_DEPTH_FREQUENCY \
            MATRIX_MUJOCO_SCENE MATRIX_UE_SCENE_OVERRIDE MATRIX_SCENE_JSON_OVERRIDE \
            GO2W_WORLD_SCENE GO2W_SCENE_JSON MATRIX_WORLD_MODEL_VALIDATOR \
            GO2W_MJ_STATE_RELAY \
            MATRIX_SKIP_STALE_PROCESS_CLEANUP \
            GO2W_NAV_MODE_FILE GO2W_RL_STOCHASTIC GO2W_RL_HISTORY_MODE \
            GO2W_RL_TERRAIN_GUARD GO2W_RL_STAIR_ASSIST GO2W_RL_STAIR_APPROACH_ASSIST \
            GO2W_RL_STAIR_OBSTACLE_AVOIDANCE GO2W_RL_STAIR_PLANNER_YAW_BLEND \
            GO2W_RL_STAIR_PLANNER_CENTERING_SCALE GO2W_RL_STAIR_PLANNER_MIN_VX \
            GO2W_RL_STAIR_PLANNER_MAX_VX GO2W_RL_STAIR_PLANNER_TURN_THRESHOLD \
            GO2W_RL_STAIR_OBSTACLE_MIN_Y GO2W_RL_STAIR_OBSTACLE_MAX_Y \
            COSMOS_VLN_NAV_MODE_ATTEMPTS; do
  if [[ -n "${!name:-}" ]]; then
    SYSTEMD_ENV+=("--setenv=${name}=${!name}")
  fi
done

systemd-run --user --quiet --unit="${UNIT%.service}" --collect \
  "${SYSTEMD_ENV[@]}" \
  --property=Type=simple \
  --property=KillMode=control-group \
  --property=TimeoutStopSec=15 \
  --property="WorkingDirectory=${ROOT_DIR}" \
  --property="StandardOutput=append:${LOG_DIR}/go2w_systemd.log" \
  --property="StandardError=append:${LOG_DIR}/go2w_systemd.log" \
  "${ROOT_DIR}/scripts/start_go2w_interactive.sh" --foreground

deadline=$((SECONDS + 120))
while (( SECONDS < deadline )); do
  if ! systemctl --user is-active --quiet "${UNIT}"; then
    echo "[ERROR] Go2-W systemd stack exited during startup." >&2
    tail -n 100 "${LOG_DIR}/go2w_systemd.log" >&2 || true
    exit 1
  fi
  if grep -q 'MATRiX + Go2-W (rl) + RoamerX LCM demo started' \
      "${LOG_DIR}/go2w_systemd.log" 2>/dev/null; then
    echo "[OK] Go2-W stack is persistent under ${UNIT}."
    echo "[INFO] ${LOG_DIR}/go2w_systemd.log"
    exit 0
  fi
  sleep 1
done

echo "[ERROR] Timed out waiting for Go2-W systemd startup." >&2
exit 1
