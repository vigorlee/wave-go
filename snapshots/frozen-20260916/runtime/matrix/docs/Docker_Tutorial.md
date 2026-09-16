# Docker Tutorial

This guide explains how to run MATRiX in a GPU-enabled Docker container on Linux with X11 display forwarding.

## What This Setup Does

The provided Docker run script:

- starts a container with NVIDIA GPU passthrough;
- forwards the host X11 display for GUI applications;
- mounts the current repository into the container at `/workspace`;
- uses host networking for ROS, MuJoCo, and launcher communication;
- passes through common input, USB, DRI, Vulkan, and GLVND devices/libraries.

The launch script is:

```bash
scripts/docker/docker_run_gpu.sh
```

## Script Relationship

There are two Docker-related scripts with different responsibilities:

- `scripts/docker/docker_run_gpu.sh` runs on the host machine. It starts the Docker container, mounts the repository to `/workspace`, enables GPU/X11/device passthrough, and forwards the command to run inside the container.
- `scripts/docker/entrypoint.sh` runs inside the Docker image as the container entrypoint. It sources ROS and workspace setup files, prepares runtime environment variables/directories, then executes the command provided by `docker_run_gpu.sh`.

The normal launch flow is:

```text
host: scripts/docker/docker_run_gpu.sh
  -> docker run zsibot/matrix:latest ...
    -> container: scripts/docker/entrypoint.sh
      -> ./bin/sim_launcher
```

Users normally run only `scripts/docker/docker_run_gpu.sh`; `entrypoint.sh` is wired into the Docker image.

## Prerequisites

Install these on the host machine first:

- Docker: https://docs.docker.com/engine/install/
- NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html
- A Linux desktop session with X11 display forwarding available.
- A MATRiX Docker image. The default image name is `zsibot/matrix:latest`.

If your image has a different name, set `MATRIX_DOCKER_IMAGE` when running the script.

## Start MATRiX

Run from the repository root:

```bash
bash scripts/docker/docker_run_gpu.sh
```

By default this runs:

```bash
./bin/sim_launcher
```

To use a different image:

```bash
MATRIX_DOCKER_IMAGE=matrix-dev:latest bash scripts/docker/docker_run_gpu.sh
```

To run a custom command inside the container:

```bash
bash scripts/docker/docker_run_gpu.sh bash
```

## Container Name

The default container name is `matrix-sim`. Override it if needed:

```bash
MATRIX_DOCKER_CONTAINER=matrix-test bash scripts/docker/docker_run_gpu.sh
```

## Join a Running Container

If the container is still running and you need another shell:

```bash
docker exec -it matrix-sim bash
```

## Notes

- Run the script from the repository root. It mounts `$(pwd)` into `/workspace`.
- The script removes an existing container with the same name before starting a new one.
- The script temporarily allows local root X11 access with `xhost +local:root` and restores it on exit.
- If Docker requires sudo on your host, run the script with `sudo -E` so `DISPLAY` is preserved.
