#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${NEZHA3_PYTHON:-/home/unitree/miniconda3/envs/unitree_lerobot/bin/python}"
exec "${PYTHON}" "${ROOT_DIR}/scripts/nezha3_stair_benchmark.py" "$@"
