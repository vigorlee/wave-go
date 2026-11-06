#!/bin/bash
set -e

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "$PROJECT_ROOT"

bash "$SCRIPT_DIR/install_deps.sh"
# bash "$SCRIPT_DIR/download_uesim.sh"

# mc modify config and compile
# bash "$SCRIPT_DIR/modify_config.sh"
# bash "$SCRIPT_DIR/build_mc.sh"
# bash "$SCRIPT_DIR/build_navigo.sh"

echo "Initialization complete."