#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_ROOT}/.venv-oli-train/bin/activate"
export PYTHONPATH="${PROJECT_ROOT}:${PROJECT_ROOT}/third_party/mujoco_playground${PYTHONPATH:+:${PYTHONPATH}}"
export XLA_PYTHON_CLIENT_PREALLOCATE=false
export MUJOCO_GL=egl

FLAT_STRONG_STEPS="${OLI_FLAT_STRONG_STEPS:-5000000}"
FLAT_WEAK_STEPS="${OLI_FLAT_WEAK_STEPS:-5000000}"
FLAT_ZERO_STEPS="${OLI_FLAT_ZERO_STEPS:-10000000}"
H05_STEPS="${OLI_H05_STEPS:-20000000}"
H10_STEPS="${OLI_H10_STEPS:-30000000}"
H15_STEPS="${OLI_H15_STEPS:-50000000}"
NUM_ENVS="${OLI_TRAIN_ENVS:-2048}"
RUN_ROOT="${OLI_TRAIN_RUN_ROOT:-${PROJECT_ROOT}/artifacts/oli_stairs/curriculum}"

python -m training.oli_stairs.train \
  --stage flat \
  --timesteps "${FLAT_STRONG_STEPS}" \
  --num-envs "${NUM_ENVS}" \
  --learning-rate 0.00003 \
  --balance-assist-min 0.75 \
  --balance-assist-max 1.0 \
  --output-dir "${RUN_ROOT}/flat_strong"

python -m training.oli_stairs.train \
  --stage flat \
  --timesteps "${FLAT_WEAK_STEPS}" \
  --num-envs "${NUM_ENVS}" \
  --learning-rate 0.00003 \
  --balance-assist-min 0.20 \
  --balance-assist-max 0.45 \
  --restore "${RUN_ROOT}/flat_strong/checkpoints" \
  --output-dir "${RUN_ROOT}/flat_weak"

python -m training.oli_stairs.train \
  --stage flat \
  --timesteps "${FLAT_ZERO_STEPS}" \
  --num-envs "${NUM_ENVS}" \
  --learning-rate 0.00002 \
  --balance-assist-min 0.0 \
  --balance-assist-max 0.0 \
  --restore "${RUN_ROOT}/flat_weak/checkpoints" \
  --output-dir "${RUN_ROOT}/flat"

python -m training.oli_stairs.train \
  --stage h05 \
  --timesteps "${H05_STEPS}" \
  --num-envs "${NUM_ENVS}" \
  --learning-rate 0.00005 \
  --restore "${RUN_ROOT}/flat/checkpoints" \
  --output-dir "${RUN_ROOT}/h05"

python -m training.oli_stairs.train \
  --stage h10 \
  --timesteps "${H10_STEPS}" \
  --num-envs "${NUM_ENVS}" \
  --learning-rate 0.00005 \
  --restore "${RUN_ROOT}/h05/checkpoints" \
  --output-dir "${RUN_ROOT}/h10"

python -m training.oli_stairs.train \
  --stage h15 \
  --timesteps "${H15_STEPS}" \
  --num-envs "${NUM_ENVS}" \
  --learning-rate 0.00003 \
  --restore "${RUN_ROOT}/h10/checkpoints" \
  --output-dir "${RUN_ROOT}/h15"

printf '%s\n' "OLI stair policy: ${RUN_ROOT}/h15/policy.onnx"
