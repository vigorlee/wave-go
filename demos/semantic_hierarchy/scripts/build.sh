#!/usr/bin/env bash
set -euo pipefail
SCENE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENE_PYTHON="${SCENE_PYTHON:-$SCENE_ROOT/.runtime/venv/bin/python}"
if [[ ! -x "$SCENE_PYTHON" ]]; then
  echo "Create the Python 3.11 environment described in README.md first." >&2
  exit 1
fi
env -u PYTHONPATH -u LD_LIBRARY_PATH "$SCENE_PYTHON" "$SCENE_ROOT/scripts/build_scene.py" "$@"
