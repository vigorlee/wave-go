# Custom Robot Dog Tutorial

This document explains how to integrate your own quadruped MuJoCo robot model into MATRiX, run your controller in MuJoCo mode, and synchronize the robot state from MuJoCo back to Unreal Engine (UE) to complete the full simulation workflow.

## 1. What This Feature Does

MATRiX currently provides a built-in `custom` robot type to support user-defined quadruped robots.

The full workflow can be summarized as follows:

1. Replace the default `custom.xml` with your own MuJoCo robot model.
2. Update the IMU and other sensor definitions according to your robot structure.
3. Keep the two copies of `custom.xml` identical.
4. Select the `custom` robot in `sim_launcher` and enable MuJoCo mode.
5. Use your own MuJoCo API controller to drive the robot.

In simple terms:

- MuJoCo handles robot dynamics and control execution.
- UE handles environment rendering, sensor simulation, and overall visualization.
- MATRiX connects these two parts together.

## 2. Key File Locations

The custom quadruped workflow mainly involves the following files:

- Custom robot model on the UE side:
  - `src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom/custom.xml`
- Custom robot model on the MuJoCo side:
  - `src/robot_mujoco/robots/custom/custom.xml`
- Custom robot scene entry file:
  - `src/robot_mujoco/robots/custom/scene_terrain_custom.xml`
- Runtime robot configuration file:
  - `config/config.json`

Please pay special attention to the following points:

- `scene_terrain_custom.xml` directly `include`s `custom.xml`.
- The `custom` directory already contains an `assets/` folder.
- Your robot mesh file paths must match the filenames referenced in `custom.xml`.

## 3. Default Model Description

The `custom.xml` currently included in the repository is a runnable MuJoCo model template, and it corresponds to the `zsl-1` robot dog by default.

This template already includes:

- The robot rigid-body structure
- Joint and actuator definitions
- IMU-related sites and sensors
- Additional sensor pose definitions such as `livox_imu` and `camera_imu`

If your robot is not `zsl-1`, you can use this file as a reference template and replace the robot body description with your own model.

## 4. Step 1: Replace `custom.xml`

First, edit:

- `src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom/custom.xml`

Replace the default robot model in that file with your own MuJoCo-format robot model.

In most cases, your model should include at least the following parts:

- `<asset>`: mesh assets and file references
- `<worldbody>`: robot rigid-body tree structure
- `<joint>`: joint definitions
- `<actuator>`: motor or actuator definitions
- `<sensor>`: sensor definitions

When replacing the model, check the following carefully:

- Whether link names and joint names are consistent throughout the file
- Whether mesh filenames match the files in `assets/`
- Whether the robot base body can be loaded correctly as the root rigid body
- Whether the model can be parsed by MuJoCo without XML errors

If your robot uses new mesh files, place the corresponding files in both of the following directories:

- `src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom/assets/`
- `src/robot_mujoco/robots/custom/assets/`

Also make sure the filenames match the references in `custom.xml` exactly.

## 5. Step 2: Update the IMU and Other Sensors

After replacing the robot body, you also need to update the sensor-related parts in `custom.xml`.

This is very important because the sensor poses in the default template are configured for the original `zsl-1` robot structure, for example:

- `imu`
- `livox_imu`
- `camera_imu`
- `this_camera`

If your robot dimensions, body structure, or sensor mounting positions are different, these parameters must be updated accordingly.

Typical updates include:

- The position and orientation of each `site`
- The mounting position of the main IMU
- The mounting position of the camera
- The position of LiDAR-related IMUs or auxiliary sites
- The site names referenced in the `<sensor>` section

For example, if you rename a site in `<worldbody>`, then the corresponding entries in `<sensor>` must also be updated, including:

- `framequat`
- `gyro`
- `accelerometer`
- `framepos`
- `framelinvel`

## 6. Step 3: Synchronize the Two Copies of `custom.xml`

After you finish editing the UE-side `custom.xml`, copy the same model to:

- `src/robot_mujoco/robots/custom/custom.xml`

These two files should always stay identical.

The reason is:

- The UE-side path is used for simulator content resource loading.
- The MuJoCo-side path is used when loading the MuJoCo scene under `src/robot_mujoco/robots/custom/`.

In practice, it is recommended to repeat the same synchronization step every time you modify the robot model:

- Copy `src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom/custom.xml`
  to
- `src/robot_mujoco/robots/custom/custom.xml`

If you changed mesh resources, also synchronize the related `assets/` files.

## 7. Step 4: Select the `custom` Robot

The runtime robot type is controlled by `config/config.json`.

For a custom robot dog, it should be set to:

- `"robot_type": "custom"`

In the current repository, `config/config.json` is already configured to use `custom` by default, so if you are following the custom quadruped workflow, this setting is usually already correct.

### Important: Select `CustomWorld` in the Launcher

When you import and launch a URDF-based custom robot from `sim_launcher`, the map selection must be:

- Map: `CustomWorld`

This is not just a recommendation. The current custom URDF runtime uses the fixed scene entry:

- `custom/scene_terrain_custom.xml`

If you select another map such as `YardWorld`, the launcher may still hand off the custom robot to the fixed `custom/scene_terrain_custom.xml` path while other parts of the runtime continue to expect a map-specific scene such as `scene_terrain_yard.xml`. That mismatch can lead to startup failures or inconsistent MuJoCo/UE behavior.

For the current workflow, treat the following combination as required:

- Robot type: `custom`
- Map: `CustomWorld`
- MuJoCo mode: enabled

## 8. Step 5: Enable MuJoCo Mode

The core execution path for a custom robot dog is MuJoCo dynamics simulation, so you need to enable MuJoCo mode in `sim_launcher`.

At the configuration level, this means:

- Set `mujoco_running` to `true` in `config/config.json`

In actual use, you usually do not need to edit this manually, because `run_sim.sh` automatically rewrites it based on the launcher selection:

- If MuJoCo mode is enabled in the launcher, it will automatically be written as `"mujoco_running": true`
- If it is not enabled, it will be reset to `false`

For a custom robot dog, the following combination is recommended at all times:

- Robot type: `custom`
- MuJoCo mode: enabled

Otherwise, your custom MuJoCo robot model will not enter the intended dynamics-control workflow.

## 9. Step 6: Understand How the Custom Robot Scene Is Loaded

The MuJoCo scene entry used by the custom robot dog is:

- `src/robot_mujoco/robots/custom/scene_terrain_custom.xml`

This scene file directly includes:

- `custom.xml`

In other words, the robot body ultimately loaded by MuJoCo is exactly the modified `custom.xml`.

At the same time, `run_sim.sh` writes the following file:

- `src/robot_mujoco/simulate/config.yaml`

The key fields will become:

- `robot: "custom"`
- `robot_scene: "scene_terrain_custom.xml"`

This is why `custom.xml` is the core file in the entire custom quadruped workflow.

## 10. Step 7: Integrate Your Own MuJoCo API Controller

Once your custom robot model is ready, the next step is to use your own controller to drive the robot through the MuJoCo API.

Typically, your controller needs to do the following:

- Read the robot state from MuJoCo
- Compute target joint commands
- Write torques, positions, or other control signals back to MuJoCo
- Implement your own gait, balance, navigation, or policy logic

You can understand MATRiX custom robot support as providing:

- An entry point for custom robot models
- A MuJoCo runtime environment and scene entry
- UE-side display and environment simulation capabilities

How the robot actually moves and how it is controlled are handled by your MuJoCo controller.

## 11. Step 8: Synchronize MuJoCo State Back to UE

When the custom robot dog runs in MuJoCo mode:

- MuJoCo computes robot dynamics and updates the state
- MATRiX synchronizes the robot state back to UE
- UE displays the robot motion inside the full scene

This allows you to get both:

- More realistic robot dynamics simulation on the MuJoCo side
- Rich environment rendering and full-scene simulation on the UE side

This is the core value of the custom robot workflow.

## 12. Recommended Workflow

To reduce problems, it is recommended to follow the steps below:

1. Prepare your own MuJoCo robot XML and mesh assets.
2. Modify `src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom/custom.xml`.
3. Update the IMU, camera, and other sensor definitions.
4. Synchronize the same `custom.xml` and related `assets/` into `src/robot_mujoco/robots/custom/`.
5. Select the `custom` robot in the launcher.
6. Enable MuJoCo mode.
7. Start the simulation.
8. Run your own MuJoCo controller.
9. Check whether the robot motion shown in UE matches the MuJoCo execution result.

## 13. Common Issues

### Issue 1: The Robot Model Cannot Be Loaded

Possible reasons:

- `custom.xml` contains XML syntax errors
- Referenced mesh files are missing from `assets/`
- Mesh filenames do not match the references in the XML
- Body or joint naming is inconsistent

### Issue 2: The Robot Loads, but IMU or Camera Data Is Incorrect

Possible reasons:

- The `site` positions still use the default template parameters
- The `<sensor>` section still references old site names
- The camera pose was not updated for the new robot structure

### Issue 3: MuJoCo-Side and UE-Side Behavior Are Inconsistent

Possible reasons:

- Only one copy of `custom.xml` was modified
- Resource files were synchronized to only one side
- MuJoCo mode was not enabled in the launcher

### Issue 4: The Robot Does Not Move as Expected

Possible reasons:

- Your controller is not sending commands correctly
- The actuator definitions do not match the controller assumptions
- The joint order or naming does not match the control code

## 14. Pre-Launch Checklist

Before starting the simulation, confirm the following:

- Your robot model is valid MuJoCo XML
- The two copies of `custom.xml` are identical
- Both `assets/` directories contain all referenced mesh files
- The IMU and other sensor definitions match the robot structure
- `robot_type` is set to `custom`
- MuJoCo mode is enabled in the launcher
- Your MuJoCo controller is ready

## 15. Summary

Integrating a custom robot dog into MATRiX is not complicated. The key steps are:

- Replace the default `zsl-1` in `custom.xml` with your own robot model
- Update the IMU and other sensors according to their real mounting positions
- Keep the two copies of `custom.xml` and related assets synchronized
- Select `custom` and enable MuJoCo mode in `sim_launcher`
- Drive the robot using your own MuJoCo API controller

Once this pipeline is working end to end, you can run your own quadruped model in MATRiX and use the combined MuJoCo + UE simulation workflow for development and validation.
