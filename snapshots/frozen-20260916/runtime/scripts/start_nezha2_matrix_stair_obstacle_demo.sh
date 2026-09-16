#!/usr/bin/env bash
# Start the MATRiX physical runtime (robot_mujoco + UE + Nezha2 RL bridge)
# with the stair-obstacle scene selected on both MuJoCo and UE sides.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export MATRIX_NEZHA2_SCENE="scene_terrain_stairs_obstacles.xml"
export MATRIX_SCENE_ID=0
export MATRIX_SCENE_JSON_OVERRIDE="${root_dir}/matrix/scene/scene_nezha2_stair_obstacles.json"
export MATRIX_DISABLE_UE=1
export NEZHA2_MATRIX_DEMO_ONLY=1
# The smoke test ends on the upper landing. Long-range navigation launchers
# leave this unset so the same RL bridge can continue along a Nav2 route.
export NEZHA2_RL_DEMO_SUCCESS_HOLD=1
# Controlled forward command is enabled only in this simulation-demo launcher.
export NEZHA2_RL_TEST_VX="${NEZHA2_RL_TEST_VX:-0.18}"
export NEZHA2_RL_TEST_DELAY="${NEZHA2_RL_TEST_DELAY:-5.0}"
exec "${root_dir}/scripts/start_nezha2_navigation.sh" --no-rviz
