#! /bin/bash
set -e

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "$PROJECT_ROOT"

NAVIGO_DIR="src/navigo"

if [ ! -d "$NAVIGO_DIR" ]; then
    echo "ERROR: Navigo directory not found: $NAVIGO_DIR"
    exit 1
fi

cd "$NAVIGO_DIR"

colcon build --packages-select pub_tf robots_dog_msgs

echo "navigo package complete."