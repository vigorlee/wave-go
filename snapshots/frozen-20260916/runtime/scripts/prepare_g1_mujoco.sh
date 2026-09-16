#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/matrix/src/robot_mujoco/simulate/build"
SOURCE="${BUILD_DIR}/robot_mujoco"
TARGET="${BUILD_DIR}/robot_mujoco_g1"
EXPECTED_SHA256="6cd7d5b37ba2b2c94be94408acee5286c5d35e919413a09c46d39e7203c3ef5c"
RUN_DEFAULT_OPCODE_OFFSET=$((0x25bbd))

if [[ ! -x "${SOURCE}" ]]; then
  echo "[ERROR] MATRiX robot_mujoco binary is missing: ${SOURCE}" >&2
  exit 1
fi

actual_sha256="$(sha256sum "${SOURCE}" | awk '{print $1}')"
if [[ "${actual_sha256}" != "${EXPECTED_SHA256}" ]]; then
  echo "[ERROR] unsupported robot_mujoco build: ${actual_sha256}" >&2
  echo "[ERROR] refusing to patch an unverified binary." >&2
  exit 1
fi

opcode="$(od -An -t x1 -j "${RUN_DEFAULT_OPCODE_OFFSET}" -N 1 "${SOURCE}" | tr -d ' ')"
if [[ "${opcode}" != "8b" ]]; then
  echo "[ERROR] unexpected Simulate::run initialization opcode: ${opcode}" >&2
  exit 1
fi

cp -a "${SOURCE}" "${TARGET}"
printf '\x83' | dd of="${TARGET}" bs=1 seek="${RUN_DEFAULT_OPCODE_OFFSET}" count=1 \
  conv=notrunc status=none
chmod +x "${TARGET}"

patched_opcode="$(od -An -t x1 -j "${RUN_DEFAULT_OPCODE_OFFSET}" -N 1 "${TARGET}" | tr -d ' ')"
if [[ "${patched_opcode}" != "83" ]]; then
  echo "[ERROR] failed to create paused G1 MuJoCo runtime." >&2
  exit 1
fi

echo "[OK] prepared paused-start G1 runtime: ${TARGET}"
