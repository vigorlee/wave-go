#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-up}"
FEEDBACK_ARG=()
if [[ "${2:-}" == "--feedback" ]]; then
  FEEDBACK_ARG=(--feedback)
elif [[ -n "${2:-}" ]]; then
  echo "Usage: $0 {up|down|return} [--feedback]" >&2
  exit 2
fi

case "${MODE}" in
  up)
    # Keep normal PointCloud2 avoidance while turning on flat ground.  Raise
    # the obstacle floor only for the tread traversal, then always restore it.
    "${ROOT_DIR}/scripts/set_go2w_nav_mode.sh" avoid
    # Rotate before translating, then enter the staircase on its centerline.
    # A direct goal from the initial +X heading makes MPPI take a wide arc and
    # bypass the outside edge of the staircase instead of climbing it.
    "${ROOT_DIR}/scripts/send_goal.sh" 0.0 0.0 1.570796 --feedback
    "${ROOT_DIR}/scripts/set_go2w_nav_mode.sh" up
    trap '"${ROOT_DIR}/scripts/set_go2w_nav_mode.sh" avoid >/dev/null 2>&1 || true' EXIT
    "${ROOT_DIR}/scripts/send_goal.sh" 0.2 0.8 1.570796 --feedback
    "${ROOT_DIR}/scripts/send_goal.sh" 0.2 4.8 1.570796 "${FEEDBACK_ARG[@]}"
    ;;
  down)
    "${ROOT_DIR}/scripts/set_go2w_nav_mode.sh" down
    trap '"${ROOT_DIR}/scripts/set_go2w_nav_mode.sh" avoid >/dev/null 2>&1 || true' EXIT
    # The last descending tread ends at y=9.25; leave a full body length before stopping.
    "${ROOT_DIR}/scripts/send_goal.sh" 0.2 10.2 1.570796 "${FEEDBACK_ARG[@]}"
    ;;
  return)
    "${ROOT_DIR}/scripts/set_go2w_nav_mode.sh" avoid
    # Turn around on flat ground before approaching the far staircase in the -Y direction.
    "${ROOT_DIR}/scripts/send_goal.sh" 0.2 9.7 -1.570796 "${FEEDBACK_ARG[@]}"
    "${ROOT_DIR}/scripts/set_go2w_nav_mode.sh" up
    trap '"${ROOT_DIR}/scripts/set_go2w_nav_mode.sh" avoid >/dev/null 2>&1 || true' EXIT
    "${ROOT_DIR}/scripts/send_goal.sh" 0.2 5.2 -1.570796 "${FEEDBACK_ARG[@]}"
    ;;
  *)
    echo "Usage: $0 {up|down|return} [--feedback]" >&2
    exit 2
    ;;
esac
