#!/usr/bin/env bash
# Run the real MuJoCo/Nezha2 stair-obstacle demo on this workstation.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${NEZHA2_PYTHON:-/home/unitree/miniconda3/envs/unitree_lerobot/bin/python3}"
output_dir="${1:-${repo_root}/artifacts/nezha2_stair_obstacle_demo_run}"

exec env MUJOCO_GL="${MUJOCO_GL:-egl}" "${python_bin}" \
  "${repo_root}/scripts/nezha2_stair_obstacle_demo.py" \
  --seed "${NEZHA2_DEMO_SEED:-15}" \
  --output-dir "${output_dir}"
