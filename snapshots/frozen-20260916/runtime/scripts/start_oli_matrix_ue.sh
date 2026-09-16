#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MATRIX_DIR="${ROOT_DIR}/matrix"
UE_DIR="${MATRIX_DIR}/src/UeSim/Linux"
RUN_DIR="${ROOT_DIR}/.run"
LOG_DIR="${ROOT_DIR}/logs"
PYTHON="${ROOT_DIR}/.venv-oli310/bin/python"
UDP_SHIM_SOURCE="${ROOT_DIR}/scripts/ue_udp_bind_remap.c"
UDP_SHIM="${RUN_DIR}/libue_udp_bind_remap.so"

mkdir -p "${RUN_DIR}" "${LOG_DIR}"
if [[ -e "${RUN_DIR}/go2w_navigation.lock" ]]; then
  echo "[ERROR] Go2-W navigation is active; refusing to replace its UE process." >&2
  exit 1
fi

if [[ ! -f "${UDP_SHIM}" || "${UDP_SHIM_SOURCE}" -nt "${UDP_SHIM}" ]]; then
  gcc -shared -fPIC -O2 -Wall -Wextra "${UDP_SHIM_SOURCE}" \
    -o "${UDP_SHIM}" -ldl
fi
"${PYTHON}" "${ROOT_DIR}/scripts/prepare_oli_matrix_ue.py" \
  >"${LOG_DIR}/oli_matrix_ue_prepare.log"

# Stage only OLI's private render proxy into the packaged UE dispatch
# directory.  Go2-W's scene_terrain_yard.xml and go2w.xml remain untouched.
OLI_UE_DIR="${MATRIX_DIR}/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/oli"
GO2W_UE_RUNTIME_DIR="${MATRIX_DIR}/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w"
cp -a "${OLI_UE_DIR}/oli_meshes" "${GO2W_UE_RUNTIME_DIR}/"
cp "${OLI_UE_DIR}/oli_HU_D04_01_proxy.xml" "${GO2W_UE_RUNTIME_DIR}/"
cp "${OLI_UE_DIR}/scene_terrain_oli_flat.xml" \
  "${GO2W_UE_RUNTIME_DIR}/scene_terrain.xml"

if [[ -f "${RUN_DIR}/oli_matrix_ue.pid" ]]; then
  old_pid="$(cat "${RUN_DIR}/oli_matrix_ue.pid")"
  kill -TERM -- "-${old_pid}" 2>/dev/null || kill -TERM "${old_pid}" 2>/dev/null || true
  rm -f "${RUN_DIR}/oli_matrix_ue.pid"
fi
pkill -TERM -f '/zsibot_mujoco_ue/Binaries/Linux/zsibot_mujoco_ue' 2>/dev/null || true
systemctl --user stop oli-matrix-ue.service >/dev/null 2>&1 || true
systemctl --user reset-failed oli-matrix-ue.service >/dev/null 2>&1 || true

UE_BIN_DIR="${UE_DIR}/zsibot_mujoco_ue/Binaries/Linux"
ENGINE_BIN_DIR="${UE_DIR}/Engine/Binaries/Linux"
OPENCV_DIR="${UE_DIR}/Engine/Plugins/Runtime/OpenCV/Binaries/ThirdParty/Linux"
UE_LD_PATH="${UE_BIN_DIR}:${ENGINE_BIN_DIR}:${OPENCV_DIR}"

: >"${LOG_DIR}/oli_matrix_ue.log"
systemd-run --user --quiet --unit=oli-matrix-ue --collect \
  --property=Type=simple \
  --property="WorkingDirectory=${UE_DIR}" \
  --property="StandardOutput=append:${LOG_DIR}/oli_matrix_ue.log" \
  --property="StandardError=append:${LOG_DIR}/oli_matrix_ue.log" \
  --setenv=ROS_DOMAIN_ID=90 \
  --setenv="VK_ICD_FILENAMES=${MATRIX_VK_ICD:-/usr/share/vulkan/icd.d/nvidia_icd.json}" \
  --setenv="CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-0}" \
  --setenv=MATRIX_REMAP_DUP_UDP_9999=1 \
  --setenv="LD_PRELOAD=${UDP_SHIM}" \
  --setenv="LD_LIBRARY_PATH=${UE_LD_PATH}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
  "${UE_DIR}/zsibot_mujoco_ue.sh" -game /Game/Maps/YardWorld \
    "-ExecCmds=t.MaxFPS ${MATRIX_UE_MAX_FPS:-20},sg.ViewDistanceQuality 1,sg.AntiAliasingQuality 1,sg.ShadowQuality 1,sg.GlobalIlluminationQuality 1,sg.ReflectionQuality 1,sg.PostProcessQuality 1,sg.TextureQuality 2,sg.EffectsQuality 1,sg.FoliageQuality 1,r.ScreenPercentage 75,r.RayTracing 0,r.VSync 0" \
    "-ResX=${MATRIX_UE_RES_X:-960}" "-ResY=${MATRIX_UE_RES_Y:-540}" \
    -windowed -nosplash -norhithread

pid="$(systemctl --user show oli-matrix-ue.service --property=MainPID --value)"
echo "${pid}" >"${RUN_DIR}/oli_matrix_ue.pid"

deadline=$((SECONDS + 45))
while (( SECONDS < deadline )); do
  pid="$(cat "${RUN_DIR}/oli_matrix_ue.pid")"
  if ! systemctl --user is-active --quiet oli-matrix-ue.service || \
     ! kill -0 "${pid}" 2>/dev/null; then
    echo "[ERROR] MATRIX UE YardWorld exited during startup." >&2
    tail -n 100 "${LOG_DIR}/oli_matrix_ue.log" >&2 || true
    exit 1
  fi
  # Loading the custom OLI meshes takes roughly ten seconds.  Wait for the
  # packaged renderer to finish both the MJCF import and the UE map load so a
  # successful return never leaves the user looking at the temporary black
  # startup window.
  if rg -q 'LoadMap Load map complete /Game/Maps/YardWorld' \
      "${LOG_DIR}/oli_matrix_ue.log" 2>/dev/null; then
    echo "[OK] MATRIX UE5 YardWorld started for the full OLI model."
    echo "[INFO] UE log: ${LOG_DIR}/oli_matrix_ue.log"
    exit 0
  fi
  sleep 1
done

echo "[ERROR] Timed out while starting MATRIX UE YardWorld." >&2
exit 1
