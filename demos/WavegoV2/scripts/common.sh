#!/usr/bin/env bash
set -eo pipefail
V2_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export V2_ROOT
CONDA_BIN="${CONDA_EXE:-$(command -v conda || true)}"
if [[ -z "$CONDA_BIN" ]]; then CONDA_BIN="$HOME/miniconda3/bin/conda"; fi
if [[ ! -x "$CONDA_BIN" ]]; then echo 'Install Miniconda or set CONDA_EXE.' >&2; return 1; fi
CONDA_BASE="$($CONDA_BIN info --base)"
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH
source "$CONDA_BASE/etc/profile.d/conda.sh"
V2_ENV="${WAVEGO_V2_ENV:-$V2_ROOT/.runtime/noetic}"
conda activate "$V2_ENV"
export V2_ENV
export CMAKE_PREFIX_PATH="${WAVEGO_V2_SDK_PREFIX:-$V2_ROOT/.runtime/sdk}:$CONDA_PREFIX"
export LD_LIBRARY_PATH="${WAVEGO_V2_SDK_PREFIX:-$V2_ROOT/.runtime/sdk}/lib:$CONDA_PREFIX/lib"
export QT_QPA_PLATFORM_PLUGIN_PATH="$CONDA_PREFIX/plugins"
if [[ -f "$V2_ROOT/workspace/devel/setup.bash" ]]; then source "$V2_ROOT/workspace/devel/setup.bash"; fi
