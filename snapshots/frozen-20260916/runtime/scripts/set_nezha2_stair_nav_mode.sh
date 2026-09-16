#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${NEZHA2_ROS_DOMAIN_ID:-90}"
export STAIR_NAV_MODE_FILE="${NEZHA2_NAV_MODE_FILE:-${ROOT_DIR}/.run/nezha2/nav_mode}"

# Nezha2 needs enough forward command to engage the learned stair gait, but a
# high minimum velocity makes MPPI drive into each riser before the legs settle.
export STAIR_NAV_VX_MAX="${NEZHA2_STAIR_VX_MAX:-0.72}"
export STAIR_NAV_VX_MIN="${NEZHA2_STAIR_VX_MIN:-0.00}"
export STAIR_NAV_WZ_MAX="${NEZHA2_STAIR_WZ_MAX:-0.15}"
export STAIR_NAV_VX_STD="${NEZHA2_STAIR_VX_STD:-0.15}"
export STAIR_NAV_WZ_STD="${NEZHA2_STAIR_WZ_STD:-0.15}"
export STAIR_NAV_MAX_VELOCITY="${NEZHA2_STAIR_MAX_VELOCITY:-[0.72, 0.0, 0.15]}"
export STAIR_NAV_MIN_VELOCITY="${NEZHA2_STAIR_MIN_VELOCITY:-[-0.12, 0.0, -0.15]}"
export STAIR_NAV_MAX_ACCEL="${NEZHA2_STAIR_MAX_ACCEL:-[0.65, 0.0, 0.30]}"
export STAIR_NAV_MAX_DECEL="${NEZHA2_STAIR_MAX_DECEL:-[-0.70, 0.0, -0.30]}"

exec "${ROOT_DIR}/scripts/set_g1_stair_nav_mode.sh" "${1:-status}"
