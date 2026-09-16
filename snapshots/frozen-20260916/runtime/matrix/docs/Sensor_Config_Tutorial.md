# MATRiX Sensor Configuration Tutorial

This guide explains how to configure robot sensors in the MATRiX simulation platform. It is written based on the actual configuration files currently in the repository and focuses on the following:

- where the sensor configuration files are located;
- which file actually takes effect at runtime;
- how to configure RGB cameras, depth cameras, LiDAR, wide-angle cameras, and panorama cameras;
- what each JSON field means;
- how to modify and validate the configuration efficiently.

---

## 1. Where the configuration files are

MATRiX sensor configuration files are located in the `config/` directory. The most common files are:

- `config/config.json`: the configuration file actually used at runtime
- `config/config_default.json`: default sensor configuration example
- `config/config_wideanglecamera.json`: wide-angle camera configuration example
- `config/config_panorama.json`: panorama RGB + panorama depth configuration example

The most important point is:

> **The file actually read at simulation startup is `config/config.json`.**

Based on the current logic in `run_sim.sh`, the platform synchronizes `config/config.json` into the UE resource directory during startup. That means the final file you should edit or replace is `config/config.json`.

---

## 2. Recommended workflow

It is recommended to configure sensors in the following way:

### Option 1: Start from the default template

```bash
cp config/config_default.json config/config.json
```

### Option 2: Start from the wide-angle camera template

```bash
cp config/config_wideanglecamera.json config/config.json
```

### Option 3: Start from the panorama camera template

```bash
cp config/config_panorama.json config/config.json
```

### Option 4: Edit the current configuration directly

```bash
vim config/config.json
```

After editing, start the MATRiX simulation using your normal workflow.

---

## 3. Overall configuration structure

A standard single-robot sensor configuration file looks like this:

```json
{
	"robot": {
		"robot_type": "xgb",
		"weapon": "",
		"position": {
			"x": 0,
			"y": 0,
			"z": 0
		},
		"state_port": 25001,
		"cmd_port": 25002,
		"EgoView": true,
		"synchronous_mode": false,
		"synchronous_frequency": 10,
		"mujoco_running": false,
		"sensors": {
			"camera": {},
			"depth_sensor": {},
			"lidar": {}
		}
	}
}
```

You can understand it as two parts:

1. `robot`: robot-level settings and runtime parameters
2. `sensors`: the collection of sensors mounted on the robot

---

## 4. Explanation of fields under `robot`

The following fields are under the `robot` object:

- `robot_type`: robot type, such as `xgw`, `xgb`, or `custom`
- `weapon`: weapon configuration; in current examples it is usually an empty string
- `position`: initial robot position
- `state_port`: port for robot state data
- `cmd_port`: port for control commands
- `EgoView`: whether to enable ego view
- `synchronous_mode`: whether to enable synchronous mode
- `synchronous_frequency`: synchronization frequency
- `mujoco_running`: whether MuJoCo integration is enabled
- `sensors`: the sensor list mounted on this robot

If your main goal is only to adjust cameras or LiDAR, the most frequently modified part is `sensors`, and sometimes `robot_type` and `position` if needed.

---

## 5. Basic rules for `sensors`

Each child object under `sensors` represents one sensor instance, for example:

```json
"sensors": {
	"camera": { ... },
	"depth_sensor": { ... },
	"lidar": { ... }
}
```

Based on the current repository examples, common sensor entries include:

- `camera`: standard RGB camera
- `depth_sensor`: depth camera
- `lidar`: LiDAR
- `wargb`: wide-angle RGB camera
- `wadrgb`: wide-angle depth configuration item whose `sensor_type` is `wadepth`
- `panoramargb`: panorama RGB camera
- `panoramadepth`: panorama depth camera

In practice, you can control sensors in the following ways:

- **Add a sensor**: add a new configuration object under `sensors`
- **Remove a sensor**: delete the corresponding sensor object
- **Adjust mount pose**: modify `position` and `rotation`
- **Adjust output parameters**: modify `topic`, `frequency`, `fov`, resolution, and so on

---

## 6. Common sensor fields

Most sensors contain the following fields:

### 6.1 `position`

This defines the mounting position of the sensor relative to the robot body.
All positions are in **meters**.

```json
"position": {
	"x": 0.18,
	"y": 0,
	"z": 0.30
}
```

- `x`: forward/backward offset (positive = forward, unit: m)
- `y`: left/right offset (positive = right, unit: m)
- `z`: up/down offset (positive = up, unit: m)

If you want to mount the sensor higher, you usually increase `z`.

### 6.2 `rotation`

This defines the mounting orientation of the sensor:

```json
"rotation": {
	"roll": 0,
	"pitch": 0,
	"yaw": 0
}
```

- `roll`: roll angle
- `pitch`: pitch angle
- `yaw`: yaw angle

For example:

- If you want the camera to look downward, adjust `pitch`
- If you want the camera to look left or right, modify `yaw`

### 6.3 `topic`

This is the ROS topic name, for example:

- `/front_camera/image/compressed`
- `/front_depth/image`
- `/front_lidar`
- `/wargb/front_left/compressed`
- `/panoramargb/front_camera/compressed`
- `/panoramadepth/front_camera`

It is recommended to keep topic names clear and avoid using the same topic for multiple sensors.

### 6.4 `frequency`

This indicates the sensor publishing frequency, usually understood in Hz.

- A larger value means faster updates
- It also increases GPU/CPU load

### 6.5 `sensor_type`

This defines the sensor type. The actual values appearing in current configuration examples include:

- `rgb`
- `depth`
- `mid360`
- `wargb`
- `wadepth`
- `panoramargb`
- `panoramadepth`

### 6.6 `cloudmode`

This field currently appears in the depth camera configuration:

```json
"cloudmode": false
```

- `cloudmode: false`: publish a depth image topic normally
- `cloudmode: true`: convert the depth result to a point-cloud-format ROS 2 message instead of publishing a depth image

If you enable `cloudmode`, it is recommended to keep the topic name clearly distinguished from a normal depth image topic so downstream consumers do not misinterpret the message type.

---

## 7. RGB camera configuration

The RGB camera example from `config/config_default.json` is as follows:

```json
"camera": {
	"position": {
		"x": 0.18,
		"y": 0,
		"z": 0.30
	},
	"rotation": {
		"roll": 0,
		"pitch": 0,
		"yaw": 0
	},
	"height": 1080,
	"width": 1920,
	"sensor_type": "rgb",
	"topic": "/front_camera/image/compressed",
	"fov": 120,
	"frequency": 10
}
```

### Key fields

- `height` / `width`: image resolution
- `sensor_type: "rgb"`: indicates a color camera
- `topic`: RGB image output topic
- `fov`: field of view
- `frequency`: publishing frequency

### Common modifications

- To improve image quality: increase `width` and `height`
- To widen the view: increase `fov`
- To reduce performance cost: lower the resolution or reduce `frequency`

---

## 8. Depth camera configuration

The depth camera example from `config/config_default.json` is as follows:

```json
"depth_sensor": {
	"position": {
		"x": 0.18,
		"y": 0,
		"z": 0.30
	},
	"rotation": {
		"roll": 0,
		"pitch": 0,
		"yaw": 0
	},
	"height": 480,
	"width": 640,
	"sensor_type": "depth",
	"topic": "/front_depth/image",
	"fov": 120,
	"frequency": 10,
	"cloudmode": false
}
```

### Key fields

- `sensor_type: "depth"`: indicates a depth camera
- `topic`: depth image output topic
- `height` / `width`: depth image resolution
- `fov`: field of view
- `cloudmode`: whether to convert the depth result into a point-cloud-format ROS 2 message

### Recommendations

- If RGB and depth cameras need aligned views, they should usually use similar `position`, `rotation`, and `fov`
- Depth output is commonly used for ranging, obstacle perception, and depth estimation tasks
- If `cloudmode` is enabled, the published message is no longer a depth image, so downstream nodes need to subscribe and process it as point cloud data

---

## 9. LiDAR configuration

The LiDAR example from `config/config_default.json` is:

```json
"lidar": {
	"position": {
		"x": 0.18,
		"y": 0,
		"z": 0.30
	},
	"rotation": {
		"roll": 0,
		"pitch": 0,
		"yaw": 0
	},
	"sensor_type": "mid360",
	"topic": "/front_lidar",
	"draw_points": false,
	"random_scan": false,
	"frequency": 10
}
```

### Key fields

- `sensor_type: "mid360"`: LiDAR type
- `topic`: point cloud or scan data output topic
- `draw_points`: whether to draw points in UE
- `random_scan`: whether to enable random scan mode
- `frequency`: scan frequency

### Recommendations

- If you only care about algorithm input and do not need point rendering in the UI, keep `draw_points` as `false`
- A very high LiDAR frequency also increases performance load, so adjust it according to task requirements

---

## 10. Wide-angle camera configuration

`config/config_wideanglecamera.json` demonstrates how wide-angle cameras are configured:

```json
"sensors": {
	"wargb": {
		"position": {
			"x": 0.27,
			"y": -0.049,
			"z": 0.05
		},
		"rotation": {
			"roll": 0,
			"pitch": 0,
			"yaw": 0
		},
		"height": 540,
		"width": 960,
		"sensor_type": "wargb",
		"topic": "/wargb/front_left/compressed",
		"fov": 80,
		"frequency": 1
	},
	"wadrgb": {
		"position": {
			"x": 0.27,
			"y": -0.049,
			"z": 0.05
		},
		"rotation": {
			"roll": 0,
			"pitch": 0,
			"yaw": 0
		},
		"height": 540,
		"width": 960,
		"sensor_type": "wadepth",
		"topic": "/wadepth/front_left",
		"fov": 80,
		"frequency": 1
	}
}
```

### Notes

- `wargb`: wide-angle RGB camera
- `wadrgb`: wide-angle depth configuration item
- They use the same mounting pose, which is suitable for outputting an aligned pair of wide-angle RGB and depth data

### Differences from the default camera setup

- Lower resolution: `540 x 960`
- Lower frequency: `1 Hz`
- Suitable for special-view sampling or low-frequency perception tasks

---

## 11. Panorama camera configuration

`config/config_panorama.json` demonstrates how to configure panorama RGB and panorama depth sensors:

```json
"sensors": {
	"panoramargb": {
		"position": {
			"x": 0,
			"y": 0,
			"z": 0.30
		},
		"rotation": {
			"roll": 0,
			"pitch": 0,
			"yaw": 0
		},
		"sensor_type": "panoramargb",
		"topic": "/panoramargb/front_camera/compressed",
		"height": 256,
		"frequency": 1
	},
	"panoramadepth": {
		"position": {
			"x": 0,
			"y": 0,
			"z": 0.30
		},
		"rotation": {
			"roll": 0,
			"pitch": 0,
			"yaw": 0
		},
		"sensor_type": "panoramadepth",
		"topic": "/panoramadepth/front_camera",
		"height": 256,
		"frequency": 1
	}
}
```

### Notes

- `panoramargb`: panorama RGB camera configuration item
- `panoramadepth`: panorama depth camera configuration item
- `sensor_type: "panoramargb"`: enables the panorama RGB camera
- `sensor_type: "panoramadepth"`: enables the panorama depth camera
- `height`: controls the output height of the panorama RGB image or panorama depth image
- `width`: does not need to be configured explicitly in the panorama template; by default it is `2 x height`
- RGB and depth use the same `position` and `rotation`, which is suitable for producing an aligned panorama RGB-depth pair

### Differences from standard and wide-angle cameras

- The panorama template currently defines `height`, but does not define `width` or `fov`; the renderer uses the built-in panorama sensor mode, and the default `width` is `2 x height`
- `height` affects the panorama output resolution and performance cost; a higher value usually gives more detail but also increases computation and bandwidth usage
- For example, when `height` is `256`, the default panorama `width` is `512`
- The recommended topic pair is `/panoramargb/front_camera/compressed` and `/panoramadepth/front_camera`
- The default example uses `frequency: 1`, which is safer for initial validation because panorama capture is usually more computationally expensive than a normal forward camera

### Recommended workflow

If you want to switch the robot to panorama output, use:

```bash
cp config/config_panorama.json config/config.json
```

Then restart the simulation and verify that the panorama topics are being published.

---

## 12. How to add a new sensor

If you want to add a new sensor, the simplest way is to copy an existing configuration block and modify the fields.

For example, to add a right-side RGB camera:

```json
"right_camera": {
	"position": {
		"x": 0.18,
		"y": 0.08,
		"z": 0.30
	},
	"rotation": {
		"roll": 0,
		"pitch": 0,
		"yaw": 30
	},
	"height": 720,
	"width": 1280,
	"sensor_type": "rgb",
	"topic": "/right_camera/image/compressed",
	"fov": 100,
	"frequency": 10
}
```

After that, place it inside the `sensors` object.

Recommendations when adding a sensor:

- Do not reuse a topic name that already exists
- Do not start with excessively high resolution or frequency
- First verify that the mounting position and orientation are reasonable

---

## 13. How to remove an unused sensor

If a sensor is not needed, the simplest method is to remove its object directly from `sensors`.

For example, if you do not need the depth camera, you can remove the entire `depth_sensor` block:

```json
"sensors": {
	"camera": { ... },
	"lidar": { ... }
}
```

Benefits of doing this:

- simpler configuration
- fewer ROS topics
- lower UE runtime load
- more stable frame rate in many cases

---

## 14. Common modification scenarios

### Scenario 1: Keep only the RGB camera

Suitable for visual demos, image recognition, object detection, and similar tasks.

How to do it:

- keep `camera`
- remove `depth_sensor`
- remove `lidar`

### Scenario 2: RGB + depth dual-camera setup

Suitable for visual perception, depth estimation, and multimodal fusion.

How to do it:

- keep `camera`
- keep `depth_sensor`
- ensure their `position`, `rotation`, and `fov` are identical or very close

### Scenario 3: LiDAR only

Suitable for point-cloud mapping, localization, and obstacle detection.

How to do it:

- keep `lidar`
- remove visual camera-related items

### Scenario 4: Switch to the wide-angle template

Suitable for tasks that need a special viewpoint or low-frequency sampling.

How to do it:

```bash
cp config/config_wideanglecamera.json config/config.json
```

### Scenario 5: Switch to the panorama template

Suitable for tasks that need 360-degree RGB observation, panorama depth perception, or aligned panoramic RGB-depth data.

How to do it:

```bash
cp config/config_panorama.json config/config.json
```

After startup, check whether `/panoramargb/front_camera/compressed` and `/panoramadepth/front_camera` are available.

---

## 15. Key points to make the configuration take effect

After modifying the sensor configuration, make sure of the following:

1. The final effective file is `config/config.json`
2. The JSON format must be valid; commas and braces must be correct
3. Topic names must not conflict
4. Restart the simulation workflow after the modification
5. If you started from a template, make sure the template content has been copied into `config/config.json`

---

## 16. Frequently asked questions

### 1) Why does changing `config_default.json` not affect the simulation?

Because the runtime actually reads `config/config.json`, not `config/config_default.json`.

The correct approach is:

```bash
cp config/config_default.json config/config.json
```

Then edit `config/config.json`.

### 2) Why is there no data after adding a new sensor?

Check these items first:

- whether the JSON is valid
- whether `sensor_type` is correct
- whether the `topic` conflicts with another one
- whether the sensor is mounted in a reasonable position
- whether the simulation has been restarted

If you are using the panorama template, also check whether:

- `panoramargb` / `panoramadepth` are present under `sensors`
- `sensor_type` is correctly set to `panoramargb` / `panoramadepth`
- `height` is configured with a reasonable value, such as `256`
- the inferred default `width` matches your expected output resolution, for example `512` when `height` is `256`
- `config/config_panorama.json` has actually been copied to `config/config.json`

If you are using the depth camera with `cloudmode: true`, also check whether:

- the downstream subscriber expects a point cloud ROS 2 message instead of a depth image
- the topic name still matches your algorithm pipeline expectations

### 3) Why does the frame rate drop significantly?

Common reasons:

- camera resolution is too high
- too many sensors are enabled
- publishing frequency is too high
- multiple expensive sensors are enabled at the same time

Optimization suggestions:

- remove unused sensors
- reduce `width` / `height`
- reduce `frequency`
- disable unnecessary visualization

---

## 17. A practical minimal configuration example

If you just want to quickly validate a forward RGB + depth + LiDAR setup, you can use a configuration like this:

```json
{
	"robot": {
		"robot_type": "xgb",
		"weapon": "",
		"position": {
			"x": 0,
			"y": 0,
			"z": 0
		},
		"state_port": 25001,
		"cmd_port": 25002,
		"EgoView": true,
		"synchronous_mode": false,
		"synchronous_frequency": 10,
		"mujoco_running": false,
		"sensors": {
			"camera": {
				"position": {
					"x": 0.18,
					"y": 0,
					"z": 0.30
				},
				"rotation": {
					"roll": 0,
					"pitch": 0,
					"yaw": 0
				},
				"height": 1080,
				"width": 1920,
				"sensor_type": "rgb",
				"topic": "/front_camera/image/compressed",
				"fov": 120,
				"frequency": 10
			},
			"depth_sensor": {
				"position": {
					"x": 0.18,
					"y": 0,
					"z": 0.30
				},
				"rotation": {
					"roll": 0,
					"pitch": 0,
					"yaw": 0
				},
				"height": 480,
				"width": 640,
				"sensor_type": "depth",
				"topic": "/front_depth/image",
				"fov": 120,
				"frequency": 10,
				"cloudmode": false
			},
			"lidar": {
				"position": {
					"x": 0.18,
					"y": 0,
					"z": 0.30
				},
				"rotation": {
					"roll": 0,
					"pitch": 0,
					"yaw": 0
				},
				"sensor_type": "mid360",
				"topic": "/front_lidar",
				"draw_points": false,
				"random_scan": false,
				"frequency": 10
			}
		}
	}
}
```

If you want to quickly validate the new panorama support, you can also use this minimal panorama configuration:

In this example, `height` is `256`, so the default panorama `width` is `512`.

```json
{
	"robot": {
		"robot_type": "xgb",
		"weapon": "",
		"position": {
			"x": 0,
			"y": 0,
			"z": 0
		},
		"state_port": 25001,
		"cmd_port": 25002,
		"EgoView": true,
		"synchronous_mode": false,
		"synchronous_frequency": 10,
		"mujoco_running": false,
		"sensors": {
			"panoramargb": {
				"position": {
					"x": 0,
					"y": 0,
					"z": 0.30
				},
				"rotation": {
					"roll": 0,
					"pitch": 0,
					"yaw": 0
				},
				"sensor_type": "panoramargb",
				"topic": "/panoramargb/front_camera/compressed",
				"height": 256,
				"frequency": 1
			},
			"panoramadepth": {
				"position": {
					"x": 0,
					"y": 0,
					"z": 0.30
				},
				"rotation": {
					"roll": 0,
					"pitch": 0,
					"yaw": 0
				},
				"sensor_type": "panoramadepth",
				"topic": "/panoramadepth/front_camera",
				"height": 256,
				"frequency": 1
			}
		}
	}
}
```

---

## 18. Summary

To configure sensors in MATRiX, you mainly need to remember three things:

1. **The file that actually takes effect is `config/config.json`**
2. **All sensors are defined under `robot.sensors`**
3. **You can quickly adjust the sensor layout by modifying pose, topic, frequency, resolution, and field of view**

If this is your first time configuring sensors, it is recommended to start by copying one of `config/config_default.json`, `config/config_wideanglecamera.json`, or `config/config_panorama.json` and then modify the fields step by step. This is the safest and easiest way to troubleshoot problems.
