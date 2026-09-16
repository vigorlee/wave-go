# Multi-Robot Simulation Tutorial

The MATRiX platform supports simulating multiple robots simultaneously within the same environment. This feature is essential for researching multi-agent collaboration, swarming behaviors, and competitive scenarios.

This tutorial guides you through configuring the system for multi-robot simulation.

---

## 🔧 Configuration

Multi-robot setup is defined in the configuration file (e.g., `config/config.json`). You can define multiple robot instances by adding corresponding JSON objects. Each robot must have unique port configurations to ensure independent communication channels.

### Example Configuration

Below is an example configuration for two robots (`robot1` and `robot2`):

```json
{
    "robot1": {
        "robot_type": "xgb",
        "weapon": "",
        "position": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "state_port": 25001,
        "cmd_port": 25002,
        "EgoView": true,
        "synchronous_mode": false,
        "synchronous_frequency": 10,
        "sensors": {
            "lidar": {
                "sensor_type": "mid360",
                "enabled": true
            }
        }
    },
    "robot2": {
        "robot_type": "xgb",
        "weapon": "",
        "position": {
            "x": 200.0,
            "y": 0.0,
            "z": 0.0
        },
        "state_port": 25011,
        "cmd_port": 25012,
        "EgoView": false,
        "synchronous_mode": false,
        "synchronous_frequency": 10,
        "sensors": {
             "lidar": {
                "sensor_type": "mid360",
                "enabled": true
            }
        }
    }
}
```

---

## 🔑 Key Parameters Explanation

| Parameter | Description | Important Notes |
| :--- | :--- | :--- |
| **robot_type** | The model type of the robot (e.g., `xgb`). | Ensure the type is supported. |
| **position** | Initial spawn coordinates (x, y, z). | **Crucial**: Set different positions for each robot to avoid collision at spawn. |
| **state_port** | UDP port for broadcasting robot state (Joints, IMU, etc.). | **Must be unique** for each robot (e.g., 25001 vs 25011). |
| **cmd_port** | UDP port for receiving control commands. | **Must be unique** for each robot (e.g., 25002 vs 25012). |
| **EgoView** | Whether the camera follows this robot. | Usually set to `true` for the main robot and `false` for others to avoid camera conflict, or use Free Camera mode. |
| **sensors** | Sensor configuration for the specific robot. | Each robot can have its own independent sensor setup. |

---

## 🎮 Controlling Multiple Robots

Since each robot listens on a different UDP port (`cmd_port`), you need to run separate control instances pointing to these specific ports.

For example:
- **Client A** sends commands to port `25002` (Controls Robot 1)
- **Client B** sends commands to port `25012` (Controls Robot 2)


This architecture allows for:
1. **Decentralized Control**: Independent agents controlling each robot.
2. **Centralized Control**: A single script sending commands to multiple ports.


### 🐍 Running the Python Demo

To run the control script using the HighLevel API, ensure the dependencies are correctly set up:

**Execute the Script**: Run the demo script to start sending commands.

    ```bash
    cp matrix/config/config_multi_robot.json matrix/config/config.json
    cd matrix/multirobot
    python highlevel_demo.py
    ```
**Modify Ports**: In `highlevel_demo.py`, ensure each robot instance is initialized with the correct `local_ip`, `local_port`, `dog_ip`, and `dog_port` corresponding to your configuration.
---

## ⚠️ Tips

- **Performance**: Simulating multiple robots with high-fidelity sensors (like Cameras/Lidar) increases GPU/CPU load. Disable unnecessary sensors for secondary robots if performance drops.
- **Identification**: Use the `EgoView` parameter to check which robot is currently being tracked by the camera, or switch to Free Camera mode (press `V`) to overlook the entire scene.
- **Python API**: Ensure your environment uses Python 3.10. When initializing robot instances, verify that each points to the correct unique IP and port combinations to prevent command conflicts.

---

# Multi-Robot HighLevel API Usage

### Initialization

| Function Name | `initRobot(const std::string & local_ip, const int local_port, const string& dog_ip， const int dog_port)` |
| :--- | :--- |
| **Description** | Establishes communication with the robot dog. The default robot IP is "192.168.234.1". If changing the robot IP, provide the new IP through this interface. |
| **Parameters** | `local_ip`: User host IP; `local_port`: User host port; `dog_ip`: Robot IP; `dog_port`: Robot port. |
| **Returns** | None |
| **Notes** | Outputs failure logs to the terminal if communication establishment fails. |

### Control Functions

| Function Name | `lieDown()` |
| :--- | :--- |
| **Description** | Controls the robot dog to lie down. |
| **Parameters** | None |
| **Returns** | 0: Normal; 0x3012: Motor data lost; 0x3010: Motor disabled; 0x3011: Motor fault; 0x3009: Motor angle out of limit; 0x3007: State machine switch failed; 0x3013: Velocity command too large. |
| **Notes** | Robot lies down, joints locked. |

| Function Name | `standUP()` |
| :--- | :--- |
| **Description** | Controls the robot dog to stand up. |
| **Parameters** | None |
| **Returns** | 0: Normal; Other error codes same as `lieDown()`. |
| **Notes** | Robot stands normally, joints locked. |

| Function Name | `passive()` |
| :--- | :--- |
| **Description** | Controls the robot dog to enter emergency passive mode. |
| **Parameters** | None |
| **Returns** | 0: Normal; Other error codes same as `lieDown()`. |
| **Notes** | Robot enters damping state and lies down. |

| Function Name | `move(float vx, float vy, float yaw_rate)` |
| :--- | :--- |
| **Description** | Controls the robot dog's movement. |
| **Parameters** | `vx`: Forward velocity (-3.0 ~ -0.4 m/s; 0.4 ~ 3.0 m/s)<br>`vy`: Lateral velocity (-2.0 ~ -0.2 m/s; 0.2 ~ 2.0 m/s)<br>`yaw_rate`: Yaw rate (-2.0 ~ 2.0 rad/s). |
| **Returns** | 0: Normal; Other error codes same as `lieDown()`. |
| **Notes** | Must be called from standing state. Controls robot movement at specified velocity. |

| Function Name | `jump()` |
| :--- | :--- |
| **Description** | Controls the robot dog to jump vertically. |
| **Parameters** | None |
| **Returns** | 0: Normal; Other error codes same as `lieDown()`. |
| **Notes** | Vertical jump, must be called from standing state. |

| Function Name | `frontJump()` |
| :--- | :--- |
| **Description** | Controls the robot dog to jump forward. |
| **Parameters** | None |
| **Returns** | 0: Normal; Other error codes same as `lieDown()`. |
| **Notes** | Forward jump, must be called from standing state. |

| Function Name | `backflip()` |
| :--- | :--- |
| **Description** | Controls the robot dog to perform a backflip. |
| **Parameters** | None |
| **Returns** | 0: Normal; Other error codes same as `lieDown()`. |
| **Notes** | Backflip, must be called from standing state. |

| Function Name | `attitudeControl(float yaw_vel, float roll_vel, float pitch_vel, float height_vel)` |
| :--- | :--- |
| **Description** | Controls the robot dog's stationary attitude and height. |
| **Parameters** | `yaw_vel`, `pitch_vel`, `roll_vel`: (-0.5 ~ 0.5 rad/s)<br>`height_vel`: (-0.5 ~ 0.5 m/s). |
| **Returns** | 0: Normal; Other error codes same as `lieDown()`. |
| **Notes** | Must be called from standing state. Controls robot attitude and height at specified velocity. |

### State Retrieval Functions

| Function Name | `getRoll()` |
| :--- | :--- |
| **Description** | Gets current roll angle. |
| **Returns** | Current roll angle (rad). |

| Function Name | `getPitch()` |
| :--- | :--- |
| **Description** | Gets current pitch angle. |
| **Returns** | Current pitch angle (rad). |

| Function Name | `getYaw()` |
| :--- | :--- |
| **Description** | Gets current yaw angle. |
| **Returns** | Current yaw angle (rad) based on power-on origin Z-axis. |

| Function Name | `getBodyAccX()` |
| :--- | :--- |
| **Description** | Gets forward acceleration. |
| **Returns** | Forward acceleration (m/s²). Robot body forward is X-axis positive. |

| Function Name | `getBodyAccY()` |
| :--- | :--- |
| **Description** | Gets lateral acceleration. |
| **Returns** | Lateral acceleration (m/s²). Robot body left is Y-axis positive. |

| Function Name | `getBodyAccZ()` |
| :--- | :--- |
| **Description** | Gets vertical acceleration. |
| **Returns** | Vertical acceleration (m/s²). Robot body vertical up is Z-axis positive. |

| Function Name | `getBodyGyroX()` |
| :--- | :--- |
| **Description** | Gets angular velocity around X-axis. |
| **Returns** | Angular velocity around X-axis (rad/s). Robot body forward is X-axis positive. |

| Function Name | `getBodyGyroY()` |
| :--- | :--- |
| **Description** | Gets angular velocity around Y-axis. |
| **Returns** | Angular velocity around Y-axis (rad/s). Robot body left is Y-axis positive. |

| Function Name | `getBodyGyroZ()` |
| :--- | :--- |
| **Description** | Gets angular velocity around Z-axis. |
| **Returns** | Angular velocity around Z-axis (rad/s). Robot body vertical up is Z-axis positive. |

| Function Name | `getPosWorldX()` |
| :--- | :--- |
| **Description** | Gets X-axis position relative to power-on origin. |
| **Returns** | Position (m). Origin is power-on point; X-axis forward, Y-axis left, Z-axis up. |

| Function Name | `getPosWorldY()` |
| :--- | :--- |
| **Description** | Gets Y-axis position relative to power-on origin. |
| **Returns** | Position (m). Origin is power-on point; X-axis forward, Y-axis left, Z-axis up. |

| Function Name | `getPosWorldZ()` |
| :--- | :--- |
| **Description** | Gets Z-axis position relative to power-on origin. |
| **Returns** | Position (m). Origin is power-on point; X-axis forward, Y-axis left, Z-axis up. |

| Function Name | `getWorldVelX()` |
| :--- | :--- |
| **Description** | Gets X-axis velocity relative to power-on origin. |
| **Returns** | Velocity (m/s). Origin is power-on point; X-axis forward, Y-axis left, Z-axis up. |

| Function Name | `getWorldVelY()` |
| :--- | :--- |
| **Description** | Gets Y-axis velocity relative to power-on origin. |
| **Returns** | Velocity (m/s). Origin is power-on point; X-axis forward, Y-axis left, Z-axis up. |

| Function Name | `getWorldVelZ()` |
| :--- | :--- |
| **Description** | Gets Z-axis velocity relative to power-on origin. |
| **Returns** | Velocity (m/s). Origin is power-on point; X-axis forward, Y-axis left, Z-axis up. |

| Function Name | `getBodyVelX()` |
| :--- | :--- |
| **Description** | Gets velocity in body X-axis direction. |
| **Returns** | Velocity (m/s). Robot body forward is X-axis positive. |

| Function Name | `getBodyVelY()` |
| :--- | :--- |
| **Description** | Gets velocity in body Y-axis direction. |
| **Returns** | Velocity (m/s). Robot body left is Y-axis positive. |

| Function Name | `getBodyVelZ()` |
| :--- | :--- |
| **Description** | Gets velocity in body Z-axis direction. |
| **Returns** | Velocity (m/s). Robot body vertical up is Z-axis positive. |
