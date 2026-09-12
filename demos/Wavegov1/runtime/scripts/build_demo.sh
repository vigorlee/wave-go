#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MATRIX_DIR="${ROOT_DIR}/matrix"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
BRIDGE_SOURCE="${ROOT_DIR}/controllers/go2w_lcm_bridge"
BRIDGE_BUILD="${ROOT_DIR}/build/go2w_lcm_bridge"
RL_BRIDGE_SOURCE="${ROOT_DIR}/controllers/go2w_rl_bridge"
RL_BRIDGE_BUILD="${ROOT_DIR}/build/go2w_rl_bridge"
TORCH_CMAKE_PREFIX="${TORCH_CMAKE_PREFIX:-/home/unitree/miniconda3/envs/unitree_lerobot/lib/python3.10/site-packages/torch/share/cmake}"
CUDA_COMPILER="${CUDACXX:-/usr/local/cuda/bin/nvcc}"

export PATH="/usr/bin:/bin:${PATH}"

set +u
source /opt/ros/humble/setup.bash
set -u

GO2W_DIR="${MATRIX_DIR}/src/robot_mujoco/zsibot_robots/go2w"
XGW_DIR="${MATRIX_DIR}/src/robot_mujoco/zsibot_robots/xgw"
if [[ ! -f "${GO2W_DIR}/go2w.xml" || ! -d "${XGW_DIR}" ]]; then
  echo "[ERROR] MATRiX Go2-W/XGW runtime models are missing. Install release assets first." >&2
  exit 1
fi

ln -sfn ../go2w "${XGW_DIR}/go2w_lcm"

cmake -S "${BRIDGE_SOURCE}" -B "${BRIDGE_BUILD}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${ROOT_DIR}/install"
cmake --build "${BRIDGE_BUILD}" -j"$(nproc)"
cmake --install "${BRIDGE_BUILD}"

if [[ ! -f "${TORCH_CMAKE_PREFIX}/Torch/TorchConfig.cmake" ]]; then
  echo "[ERROR] LibTorch CMake config missing: ${TORCH_CMAKE_PREFIX}" >&2
  exit 1
fi
if [[ ! -x "${CUDA_COMPILER}" ]]; then
  echo "[ERROR] CUDA compiler required by the installed PyTorch package is missing: ${CUDA_COMPILER}" >&2
  exit 1
fi

cmake -S "${RL_BRIDGE_SOURCE}" -B "${RL_BRIDGE_BUILD}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${ROOT_DIR}/install" \
  -DCMAKE_PREFIX_PATH="${TORCH_CMAKE_PREFIX}" \
  -DCMAKE_CUDA_COMPILER="${CUDA_COMPILER}"
cmake --build "${RL_BRIDGE_BUILD}" -j"$(nproc)"
cmake --install "${RL_BRIDGE_BUILD}"

cd "${ROAMERX_DIR}"
./build.sh all

echo "[OK] Go2-W PD/LCM bridge, DreamWaQ RL bridge, and RoamerX workspace built."
