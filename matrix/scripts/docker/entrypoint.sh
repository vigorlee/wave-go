#!/bin/bash
set -eo pipefail

# Source ROS if available. Some ROS setup scripts reference variables
# that may be undefined when `set -u` is active; temporarily disable -u
# while sourcing and then restore strictness if desired.
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
if [ -f /workspace/install/setup.bash ]; then
  # shellcheck disable=SC1091
  set +u
  source /workspace/install/setup.bash
  set -u
fi

# X11: assume host mounts Xauthority directly into the user's home directory.
# Do not attempt to copy /root/.Xauthority (requires root privileges).

# Ensure XDG_RUNTIME_DIR exists (created as the container user)
if [ -n "${XDG_RUNTIME_DIR:-}" ]; then
  mkdir -p "$XDG_RUNTIME_DIR" || true
  chmod 700 "$XDG_RUNTIME_DIR" || true
fi

echo "Starting container as: $(id -un) (uid=$(id -u), gid=$(id -g))"

# Default: run provided command
exec "$@"
