#!/bin/bash
set -e

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "$PROJECT_ROOT"

CONFIG_DIR="src/robot_mc/build/export/config"

if [ ! -d "$CONFIG_DIR" ]; then
    echo "ERROR: Config directory not found: $CONFIG_DIR"
    exit 1
fi

sed -i 's/motor_platform_type: *7/motor_platform_type: 5/' "$CONFIG_DIR/xg-user-parameters.yaml"

sed -i 's/imu_type: *0/imu_type: 2/' "$CONFIG_DIR/default-user-parameters.yaml"
sed -i 's/motor_platform_type: *0/motor_platform_type: 1/' "$CONFIG_DIR/default-user-parameters.yaml"

sed -i 's/use_gamepad: *0/use_gamepad: 1/' "$CONFIG_DIR/robot-defaults.yaml"

sed -i 's/enable_stunts: *0/enable_stunts: 1/' "$CONFIG_DIR/robot-defaults.yaml"

echo "MC config modified"
