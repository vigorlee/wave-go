#!/usr/bin/env bash
set -euo pipefail

# Launch MATRiX in a GPU-enabled Docker container.
# Run this script from the repository root so the current checkout is mounted
# into the container at /workspace.

IMAGE_NAME="${MATRIX_DOCKER_IMAGE:-zsibot/matrix:latest}"
CONTAINER_NAME="${MATRIX_DOCKER_CONTAINER:-matrix-sim}"

if [[ $# -eq 0 ]]; then
    CONTAINER_CMD=("./bin/sim_launcher")
    echo "[INFO] No custom command provided. Defaulting to: ${CONTAINER_CMD[*]}"
else
    CONTAINER_CMD=("$@")
    echo "[INFO] Using custom container command: ${CONTAINER_CMD[*]}"
fi

if ! command -v docker >/dev/null 2>&1; then
    echo "[ERROR] docker command not found. Install Docker first." >&2
    exit 1
fi

if [[ -z "${DISPLAY:-}" ]]; then
    echo "[ERROR] DISPLAY is not set. Start from a Linux desktop/X11 session." >&2
    exit 1
fi

echo "[INFO] Checking if container '${CONTAINER_NAME}' already exists..."
if docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
    echo "[INFO] Container exists. Stopping and removing old container..."
    docker stop "${CONTAINER_NAME}" >/dev/null 2>&1 || true
    docker rm "${CONTAINER_NAME}" >/dev/null 2>&1 || true
fi

echo "[INFO] Allowing X11 connections from local root containers..."
xhost +local:root >/dev/null 2>&1 || echo "[WARN] xhost failed; X11 forwarding may not work."

cleanup() {
    xhost -local:root >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[INFO] Starting container '${CONTAINER_NAME}' from image '${IMAGE_NAME}'..."
docker run --gpus all -it --rm \
    --name "${CONTAINER_NAME}" \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json" \
    --env="ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
    --env="ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/workspace" \
    --volume="/usr/share/vulkan:/usr/share/vulkan:ro" \
    --volume="/usr/share/glvnd:/usr/share/glvnd:ro" \
    --device=/dev/dri \
    --device=/dev/input \
    --device=/dev/uinput \
    --volume=/run/udev:/run/udev:ro \
    --group-add=input \
    --volume=/dev/bus/usb:/dev/bus/usb \
    --network host \
    --privileged \
    --workdir=/workspace \
    "${IMAGE_NAME}" \
    "${CONTAINER_CMD[@]}"
