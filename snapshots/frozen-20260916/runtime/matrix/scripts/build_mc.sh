#!/bin/bash
set -e

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "$PROJECT_ROOT"

ROBOT_MC_DIR="src/robot_mc"

if [ ! -d "$ROBOT_MC_DIR" ]; then
    echo "ERROR: Robot MC directory not found: $ROBOT_MC_DIR"
    exit 1
fi

cd "$ROBOT_MC_DIR"
bash build.sh s

echo "MC build complete."