# GENISOM RoamerX Open

<div align="center">
<img src="https://img.shields.io/badge/ROS2-Humble-blue" alt="ROS2 Humble">
<img src="https://img.shields.io/badge/License-Open%20Source-green" alt="License">
<img src="https://img.shields.io/badge/Platform-Linux-lightgrey" alt="Platform">
<img src="https://img.shields.io/badge/Language-C++-red" alt="Language">
</div>

**Open-source navigation stack for GENISOM-AI robots. Enables intelligent SLAM, path planning and autonomous movement. Free community version (Open). Contributions welcome!**

## 🚀 Features

- **Complete Navigation Stack**: Based on ROS2 Nav2 with custom enhancements
- **Multi-Platform Support**: Gazebo/UE simulation, NX/3588 hardware platforms
- **Advanced Path Planning**: NavFn planner with MPPI controller
- **Flexible Communication**: Support for UDP and LCM protocols
- **Behavior Trees**: Sophisticated autonomous navigation logic
- **Real-time Obstacle Avoidance**: Advanced costmap-based collision detection
- **Transform Management**: Comprehensive coordinate frame handling
- **Open Source**: Community-driven development with full source access

## 📋 Table of Contents

- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Architecture Overview](#architecture-overview)
- [Usage Examples](#usage-examples)
- [Configuration](#configuration)
- [Contributing](#contributing)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## 🔧 System Requirements

### Operating System
- **Ubuntu 22.04 LTS** (Jammy)
- **ROS2 Humble Hawksbill**

### Hardware Requirements
- **CPU**: x86_64 or ARM64 (for hardware deployment)
- **RAM**: Minimum 4GB (8GB recommended)
- **Storage**: At least 10GB free space

### Dependencies
- Gazebo (for simulation)
- OpenCV
- PCL (Point Cloud Library)
- OMPL (Open Motion Planning Library)
- Behavior Tree CPP

## 📦 Installation

### 1. Install ROS2 Humble

```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop
```

### 2. Clone and Setup the Repository

```bash
# Clone the repository
git clone https://github.com/zsibot/genisom_roamerx_open.git
cd genisom_roamerx_open

# Install all dependencies
chmod +x script/dep/install_all.sh
./script/dep/install_all.sh

```

### 3. Build the Project

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build all packages
./build.sh all
```

### 4. Source the Workspace

```bash
# Source the built workspace
source install/setup.bash
```

## 🚀 Quick Start
### 🔹 Simulation with ZsiBot-MATRiX
   Welcome to explore our another open-source simulation project, MATRiX:
👉 https://github.com/zsibot/matrix

### Layout Requirement for the Launcher Button

If you want to start RoamerX Open directly from the MATRiX launcher button, keep the two repositories at the same directory level:

```text
<workspace>/
  matrix/
  genisom_roamerx_open/
```

For example:

```text
/home/user/work/workspace/
  matrix/
  genisom_roamerx_open/
```

The launcher-side RoamerX integration resolves `genisom_roamerx_open` from this sibling layout by default. If it is placed elsewhere, export `GENISOM_ROAMERX_OPEN_WORKSPACE=/path/to/genisom_roamerx_open` before starting the launcher.

The current `genisom_roamerx_open` workspace may not include the optional `pub_tf` package. The launcher skips it when missing; `robot_forward` publishes `odom -> base_link` TF from MATRiX `/odom/mujoco_odom` after the MATRiX simulation is running.

1. **Launch MATRiX simulation first (provides odom for UE navigation):**
```bash
# Terminal 0 (inside Docker container)
cd matrix/
./bin/sim_launcher
```

*(Note: In the launcher, select the appropriate robot and map. `sim_launcher` automatically sets required environment variables like `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and `ROS_DOMAIN_ID=89`.)*

################# 注意⚠️：需要在matrix/src/robot_mc/build/export/config/sdk_config.yaml文件的最上面添加以下代码 #################
target_ip: "127.0.0.1"
target_port: 43988
#############################################################################################################################
```

If you are running on host desktop with GUI, you may use:
```bash
cd matrix/
./open_sim_launcher
```

2. **Launch navigation in two more terminals (Docker/container recommended):**
```bash
# Terminal 1
# 注意⚠️：在终端中先export
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89
export SDK_CLIENT_IP=127.0.0.1
cd /workspace/genisom_roamerx_open
bash script/bash/start_navigation.sh nav

# Terminal 2
# 注意⚠️：在终端中先export
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89
export SDK_CLIENT_IP=127.0.0.1
cd /workspace/genisom_roamerx_open
bash script/bash/start_navigation.sh rviz
```

```bash
# Terminal 3
################# 注意⚠️：需要先进入root终端 #########################################################
sudo -i
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89
export SDK_CLIENT_IP=127.0.0.1
source /opt/robot/robot-forward/install/setup.bash
/opt/robot/robot-forward/install/robot_forward/lib/robot_forward/robot_forward
```

You can also print the UE split commands with:
```bash
bash script/bash/start_navigation.sh
```

3. **Send Navigation Goals via RViz2**
   - In RViz2, use the “nav2_rviz_plugins” panel to send navigation commands.
   - The robot will execute navigation tasks such as waypoint patrols with static and dynamic obstacle avoidance.
   - Demo results (Note: Check the clear GIFs in the demo_gif folder.):
     - patrol task with multiple midpoints
       -   ![patrol task with avoidance](demo_gif/ue_mini.gif)

4. **Stop the Navigation Stack**
```bash
bash script/bash/stop_navigation.sh
```


### Simulation with Gazebo

#### 🔹 Single Point Navigation
1. **Launch the Navigation Stack:**
```bash
ros2 launch robot_navigo navigation_bringup.launch.py \
    platform:=GAZEBO \
    mc_controller_type:=RL_TRACK_VELOCITY \
    communication_type:=LCM \
    map:=/path/to/map/map.yaml

ros2 launch pub_tf pub_tf.launch.py tf_type:=gazebo_tf
```

2. **Launch Gazebo to Give Odom Info**
Make sure the Gazebo simulation is running to publish odometry (/odom/gazebo) and other necessary topics for navigation.

3. **Send Navigation Goals via RViz2**
   - Open RViz2 and use the “2D Goal Pose” tool to send navigation targets.
   - The robot will autonomously navigate to the goal position, avoiding both static and dynamic obstacles.
   - Demo results:
     - 🟢 Static Avoidance
        ![single point with static avoidance](demo_gif/single_static.gif)

     - 🔵 Dynamic Avoidance
        ![single point with dynamic avoidance](demo_gif/single_dynamic.gif)

#### Patrol Task Navigation
1. **Launch Navigation Stack:**
```bash
ros2 launch robot_navigo navigation_bringup.launch.py \
    platform:=GAZEBO \
    mc_controller_type:=RL_TRACK_VELOCITY \
    communication_type:=LCM \
    map:=/path/to/map/map.yaml

ros2 launch pub_tf pub_tf.launch.py tf_type:=gazebo_tf

rviz2 -d /path/to/your/rviz2_config.rviz
```

2. **Launch Gazebo to Give Odom Info**

3. **Launch RViz to Send Navigation Goals:**
   - Use "nav2_rviz_plugins" Panel in RViz2 to send navigation commands
   - Demo results:
     - patrol task with multiple midpoints
        ![patrol task with avoidance](demo_gif/patrol.gif)

### Hardware Deployment

For physical robot deployment:

```bash
TODO
```

## 🏗️ Architecture Overview

The GENISOM RoamerX Open stack is organized into three main modules:

### 📡 Interface Module (`src/interface/`)
- **mc_sdk_rosmsgs**: Motion controller SDK message definitions
- **robots_dog_msgs**: Comprehensive robot message types including:
  - Navigation commands and states
  - Sensor data (IMU, odometry, localization)
  - Motor control and telemetry
  - Fault reporting and monitoring

### 🧭 Navigation Module (`src/navigation/src/`)

#### Core Navigation Components
- **navigo_bt_navigator**: Behavior tree-based navigation orchestration
- **navigo_mppi_controller**: Model Predictive Path Integral controller for dynamic path following
- **navigo_costmap_2d**: 2D costmap generation with obstacle detection
- **navigo_path_planner**: Global path planning services
- **navigo_navfn_planner**: NavFn-based path planning algorithm

#### Supporting Services
- **navigo_behaviors**: Navigation behavior implementations (spin, backup, wait)
- **navigo_collision_monitor**: Real-time collision detection and avoidance
- **navigo_map_server**: Map loading and serving capabilities
- **navigo_waypoint_follower**: Multi-waypoint navigation execution
- **navigo_velocity_optimizer**: Velocity command smoothing and optimization

#### Integration Layer
- **robot_navigo**: Robot-specific integration, launch files, and parameter configurations

###  SLAM Module (`src/slam/src/`)

#### Core Mapping Components

- **3D pcd map**: It can generate a 3D pcd point cloud global map
- **2D grid map**: It can generate 2D global occupation raster maps

#### Supporting Services

- **mapping state **: Control the different states of the mapping( start mapping、save map)

### Localization Module (`src/localization/`)
#### Core Localization Components
- **fast_gicp**: Fast Global Registration algorithm for point cloud registration
- **ndt_omp**: Normal Distributions Transform algorithm for point cloud registration
- **localization**: Localization algorithm for robot pose estimation

#### Supporting Services
- **load pcd map**: Load 3D pcd map for localization

## 💡 Usage Examples

### Basic Navigation Commands

```bash
# 1. Start the navigation stack
ros2 launch robot_navigo navigation_bringup.launch.py \
    platform:=GAZEBO \
    mc_controller_type:=RL_TRACK_VELOCITY \
    communication_type:=LCM

# 2. Load a map (in another terminal)
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \
    "{map_url: '/path/to/map.yaml'}"

# 3. Pub tf
 ros2 launch pub_tf pub_tf.launch.py tf_type:=gazebo_tf

# 4. Rviz
 rviz2 -d /path/to/your/rviz2_config.rviz

# 5. Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {w: 1.0}}}}"
```

### Waypoint Navigation

```bash
# Navigate through multiple waypoints
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
    "{poses: [
        {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
        {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}},
        {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}
    ]}"
```

### Map Building with SLAM

```bash
# Run Robot SLAM
ros2 launch robot_slam slam.launch.py

# Start Mapping
ros2 service call /slam_state_service robots_dog_msgs/srv/MapState "{data: 3}"

# Save Map
ros2 service call /slam_state_service robots_dog_msgs/srv/MapState "{data: 5}"

#Note: map data is saved by default in the main directory under./jszr/map
#      For specific operations, please refer to the readme of slam
```

### Run Localization

```bash
# Run Robot Localization
ros2 launch localization localization.launch.py

# Load PCD Map
ros2 service call /load_map_service robots_dog_msgs/srv/LoadMap "{pcd_path: /your_pcd_map_path/map.pcd}"
#Note: Please use the right pcd path and pcd file name.
#eg: ros2 service call /load_map_service robots_dog_msgs/srv/LoadMap "{pcd_path: /home/user_name/datd/map/map.pcd}"
```


## ⚙️ Configuration

### Platform Configuration

The system supports multiple platform configurations:

| Platform | Description | Use Case |
|----------|-------------|----------|
| `GAZEBO` | Gazebo simulation | Development and testing |
| `NX_XG3588` | XG3588 hardware platform | High-performance deployment |
| `XG3588` | Standalone XG3588 | Custom hardware integration |

### Controller Types

| Controller Type | Description | Application |
|----------------|-------------|-------------|
| `RL_TRACK_VELOCITY` | Velocity-based tracking | Simple navigation tasks |
| `RL_TRACK_PATH` | Path-following controller | Precise trajectory following |

### Communication Protocols

| Protocol | Description | Use Case |
|----------|-------------|----------|
| `UDP` | Direct UDP communication | Low-latency, embedded systems |
| `LCM` | Lightweight Communications | Multi-process, distributed systems |

### Parameter Files

Main configuration file: `src/navigation/src/robot_navigo/params/navigo_params.yaml`

Key configuration sections:
- **bt_navigator**: Behavior tree settings
- **controller_server**: MPPI controller parameters
- **local_costmap**: Local obstacle avoidance settings
- **global_costmap**: Global path planning costmap
- **planner_server**: Path planning algorithm settings

## 🔨 Development

### Building Individual Packages

```bash
# Build specific package
colcon build --packages-select <package_name>

# Build with debug symbols
./build.sh all debug
```

### Code Formatting

```bash
# Format all C++ code
./format.sh

# Format specific directory
./format.sh /path/to/directory
```

### Running Tests

```bash
# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select <package_name>
```

## 🐛 Troubleshooting

### Common Issues

**1. Build Errors**

```bash
# Clean and rebuild
rm -rf build install log
./build.sh all
```

**2. Transform Errors**
```bash
# Check transform tree
ros2 run tf2_tools view_frames

# Monitor transform publication
ros2 topic echo /tf
```

**3. Navigation Not Starting**
```bash
# Check required parameters
ros2 param list /bt_navigator
ros2 param get /bt_navigator use_sim_time
```

**4. Map Loading Issues**
```bash
# Verify map file path and format
ros2 topic echo /map --once
```

**5. Backend Residual Process Cleanup (Docker container)**
```bash
# Recommended: one-command cleanup
bash script/bash/cleanup_backend.sh

# Optional: specify container name
bash script/bash/cleanup_backend.sh zsibot_roamerx_matrix007_jbz
```

## 🤝 Contributing

We welcome contributions from the community! Here's how to get involved:

### Development Workflow

1. **Fork the repository**
2. **Create a feature branch**: `git checkout -b feature/amazing-feature`
3. **Make your changes** following our coding standards
4. **Run tests**: `colcon test`
5. **Format code**: `./format.sh`
6. **Commit changes**: `git commit -m 'Add amazing feature'`
7. **Push to branch**: `git push origin feature/amazing-feature`
8. **Submit a Pull Request**

### Coding Standards

- Follow [ROS2 C++ Style Guide](https://docs.ros.org/en/rolling/Contributing/Code-Style-Language-Versions.html)
- Use `clang-format` for consistent formatting
- Write comprehensive comments and documentation
- Include unit tests for new features

### Reporting Issues

Please use [GitHub Issues](https://github.com/zsibot/genisom_roamerx_open/issues) to report bugs or request features.

Include:
- Operating system and ROS2 version
- Hardware platform
- Steps to reproduce the issue
- Expected vs actual behavior
- Relevant log output

## 📄 License

This project is released under the **Open Source License**. See [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **ROS2 Community** for the excellent navigation framework
- **Nav2 Team** for the robust navigation stack
- **Contributors** who help improve this project
- **ZsiBot Community** for feedback and testing

---

<div align="center">
Made with ❤️ by the ZsiBot Community
</div>
