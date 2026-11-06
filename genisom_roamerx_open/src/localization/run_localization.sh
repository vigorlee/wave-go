#! /bin/bash

# Absolute path of this script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Strategy 1: auto-detect workspace path
find_workspace() {
    local current_dir="$1"
    
    # If running at the workspace root
    if [ -f "$current_dir/install/setup.bash" ]; then
        echo "$current_dir"
        return 0
    fi
    
    # If running inside the install directory
    if [[ "$current_dir" == */install/* ]]; then
        local workspace_dir="$(dirname "$(dirname "$(dirname "$current_dir")")")"
        if [ -f "$workspace_dir/install/setup.bash" ]; then
            echo "$workspace_dir"
            return 0
        fi
    fi
    
    # If running inside the src directory
    if [[ "$current_dir" == */src/* ]]; then
        local workspace_dir="$(dirname "$(dirname "$current_dir")")"
        if [ -f "$workspace_dir/install/setup.bash" ]; then
            echo "$workspace_dir"
            return 0
        fi
    fi
    
    return 1
}

# Try finding the workspace starting from current directory
WORKSPACE_DIR=""
if find_workspace "$(pwd)"; then
    WORKSPACE_DIR="$(find_workspace "$(pwd)")"
    echo "Detected workspace from current directory: $WORKSPACE_DIR"
else
    # Strategy 2: fallback to two levels above the script directory
    WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
    echo "Using fallback workspace path: $WORKSPACE_DIR"
    
    # Validate the detected workspace path
    if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        echo "Error: Workspace not found. Please ensure the script is in the correct location"
        echo "Script path: $SCRIPT_DIR"
        echo "Tried workspace path: $WORKSPACE_DIR"
        exit 1
    fi
fi

echo "Using workspace path: $WORKSPACE_DIR"

# Check ROS2 environment
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "Error: ROS2 Humble environment not found. Please install ROS2 Humble"
    exit 1
fi

# Check if workspace has been built
if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "Error: Workspace not built. Please run 'colcon build' first"
    exit 1
fi

# Source environments
echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

echo "Sourcing workspace environment..."
source "$WORKSPACE_DIR/install/setup.bash"

# Create log directory
LOG_DIR="$HOME/.ros"
mkdir -p "$LOG_DIR"

# Generate timestamped logfile name
TIMESTAMP=$(date +"%Y-%m-%d-%H-%M-%S")
LOG_FILE="$LOG_DIR/${TIMESTAMP}_localization_log"

echo "Log file: $LOG_FILE"

# Redirect stdout/stderr to logfile and console (tee)
exec 1> >(tee -a "$LOG_FILE")
exec 2> >(tee -a "$LOG_FILE" >&2)

# Launch localization node
echo "Launching localization node..."
ros2 launch localization localization.launch.py

