#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cb="${CONDA_EXE:-$(command -v conda || true)}"
if [[ -z "$cb" ]]; then cb="$HOME/miniconda3/bin/conda"; fi
[[ -x "$cb" ]] || { echo 'Install Miniconda or set CONDA_EXE.' >&2; exit 1; }
envdir="${WAVEGO_V2_ENV:-$root/.runtime/noetic}"
[[ ! -d "$envdir/conda-meta" ]] || { echo "Environment already exists: $envdir; use build.sh, or select a new WAVEGO_V2_ENV."; exit 0; }
"$cb" create --prefix "$envdir" --override-channels -c conda-forge -c robostack-noetic ros-noetic-desktop ros-noetic-pcl-ros ros-noetic-tf2-sensor-msgs compilers "cmake<4" catkin_tools make ninja pkg-config eigen apr gtsam "empy=3.3.4" scipy numpy matplotlib -y
