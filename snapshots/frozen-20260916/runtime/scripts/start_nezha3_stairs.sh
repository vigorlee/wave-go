#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MATRIX_DIR="${ROOT_DIR}/matrix"
RUN_DIR="${ROOT_DIR}/.run"
LOG_DIR="${ROOT_DIR}/logs"
FOREGROUND=0
TEST=0
for arg in "$@"; do
  case "$arg" in
    --foreground) FOREGROUND=1 ;;
    --test) TEST=1 ;;
    *) echo "Usage: $0 [--test] [--foreground]" >&2; exit 2 ;;
  esac
done
mkdir -p "${RUN_DIR}" "${LOG_DIR}"

if (( TEST )); then
  exec "${ROOT_DIR}/scripts/test_nezha3_stairs.sh"
fi

if pgrep -f '(/robot_mujoco($| )|nezha3_rl_bridge|zsibot_mujoco_ue|mc_ctrl)' >/dev/null; then
  echo "[ERROR] another MATRiX simulator/controller is running; stop it first." >&2
  exit 1
fi
if [[ ! -x "${ROOT_DIR}/install/bin/nezha3_rl_bridge" ]]; then
  echo "[ERROR] Nezha3 bridge missing; run scripts/build_demo.sh first." >&2
  exit 1
fi

ln -sfn ../custom "${MATRIX_DIR}/src/robot_mujoco/zsibot_robots/xgw/nezha3"
sed -i 's/^robot: .*/robot: "xgw"/' "${MATRIX_DIR}/src/robot_mujoco/simulate/config.yaml"
sed -i 's|^robot_scene: .*|robot_scene: "nezha3/scene_terrain_custom.xml"|' \
  "${MATRIX_DIR}/src/robot_mujoco/simulate/config.yaml"

export MATRIX_STATE_PORT="${MATRIX_STATE_PORT:-25301}"
export MATRIX_CMD_PORT="${MATRIX_CMD_PORT:-25302}"
nohup stdbuf -oL -eL "${ROOT_DIR}/install/bin/nezha3_rl_bridge" \
  >"${LOG_DIR}/nezha3_bridge.log" 2>&1 &
echo "$!" >"${RUN_DIR}/nezha3_bridge.pid"

MUJOCO_BUILD="${MATRIX_DIR}/src/robot_mujoco/simulate/build"
MUJOCO_LD_PATH="${MUJOCO_BUILD}:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:${MATRIX_DIR}/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux:${MATRIX_DIR}/src/UeSim/Linux/Engine/Binaries/Linux"
(
  cd "${MUJOCO_BUILD}"
  nohup env LD_LIBRARY_PATH="${MUJOCO_LD_PATH}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
    ./robot_mujoco >"${LOG_DIR}/nezha3_mujoco.log" 2>&1 &
  echo "$!" >"${RUN_DIR}/nezha3_sim.pid"
)

deadline=$((SECONDS + 25))
while (( SECONDS < deadline )); do
  if rg -q 'Policy control enabled' "${LOG_DIR}/nezha3_bridge.log" 2>/dev/null; then
    echo "[OK] Nezha3 policy active (ports ${MATRIX_STATE_PORT}/${MATRIX_CMD_PORT})."
    echo "[INFO] logs: ${LOG_DIR}/nezha3_bridge.log, ${LOG_DIR}/nezha3_mujoco.log"
    if (( FOREGROUND )); then
      trap '"${ROOT_DIR}/scripts/stop_nezha3_stairs.sh" >/dev/null 2>&1 || true; exit 0' INT TERM
      while kill -0 "$(cat "${RUN_DIR}/nezha3_sim.pid")" 2>/dev/null &&
            kill -0 "$(cat "${RUN_DIR}/nezha3_bridge.pid")" 2>/dev/null; do
        sleep 1
      done
      exit 1
    fi
    exit 0
  fi
  if ! kill -0 "$(cat "${RUN_DIR}/nezha3_sim.pid")" 2>/dev/null ||
     ! kill -0 "$(cat "${RUN_DIR}/nezha3_bridge.pid")" 2>/dev/null; then
    tail -80 "${LOG_DIR}/nezha3_mujoco.log" >&2 || true
    tail -80 "${LOG_DIR}/nezha3_bridge.log" >&2 || true
    exit 1
  fi
  sleep 1
done
echo "[ERROR] Nezha3 policy did not activate." >&2
exit 1
