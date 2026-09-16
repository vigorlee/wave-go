# Custom Scene Tutorial

This tutorial explains how to use MATRiX custom scenes.

In the custom-scene workflow, Unreal Engine (UE) reads `scene/scene.json` at startup, creates the visual scene from the JSON content, and keeps it aligned with the corresponding MuJoCo physical scene. With one JSON file, you can describe:

- static cubes and cylinders;
- dynamic pedestrians;
- pedestrian walking speed;
- waypoint-based walking trajectories;
- mixed scenes that contain both obstacles and moving people.

---

## What Happens When You Load a Custom Scene

When you select **Custom Map** in the launcher, the system uses `scene/scene.json` as the input scene description.

The pipeline is:

1. Read `scene/scene.json`
2. Parse each `Element` entry
3. Generate the corresponding static objects and dynamic pedestrians
4. Synchronize the UE visual scene and MuJoCo physics configuration
5. Start simulation with the generated custom map

This means `scene.json` is the single source of truth for the custom scene.

---

## Quick Start

### Step 1: Prepare a Scene JSON File

Create a new JSON file, or start from one of the examples in the `scene/` directory:

- `scene/scene_example_1.json`
- `scene/scene_example_2.json`
- `scene/scene_example_3.json`

### Step 2: Write Your Scene Content

Define scene elements in JSON.

Currently supported element categories are:

- static cube: `cube1`
- static cube: `cube2`
- static cylinder: `cylinder1`
- dynamic pedestrian: `human1`

### Step 3: Replace the Active Scene File

Copy your scene file to `scene/scene.json`:

```bash
cp your_custom_scene.json scene/scene.json
```

If you want to test an existing example directly:

```bash
cp scene/scene_example_1.json scene/scene.json
```

### Step 4: Launch MATRiX

Start the simulator:

```bash
./open_sim_launcher
```

Then choose **Custom Map** in the launcher.

UE will read `scene/scene.json` and initialize the custom scene from it.

---

## Scene File Rules

Before editing `scene.json`, keep these rules in mind:

- the root object contains multiple entries such as `Element1`, `Element2`, `Element3`;
- each root key must be unique;
- each element must contain the fields required by its type;
- static objects use `type: "static"`;
- pedestrians use `type: "dynamic"`;
- dynamic pedestrians can include `avoid`, `velocity`, and `trajectory`;
- coordinates use `x`, `y`, `z` fields.

---

## Minimal Scene Template

The following is the smallest useful scene structure:

```json
{
  "Element1": {
    "name": "obstacle1",
    "type": "static",
    "model": "cube1",
    "position": {
      "x": 0,
      "y": 0,
      "z": 50
    },
    "rotation": {
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    },
    "scale": {
      "x": 1,
      "y": 1,
      "z": 1
    }
  }
}
```

---

## Common Field Meanings

Most elements share the following fields:

- `name`: element name, recommended to be unique and meaningful
- `type`: `static` or `dynamic`
- `model`: model type such as `cube1`, `cube2`, `cylinder1`, `human1`
- `position`: initial position in 3D space
- `rotation`: element orientation using `pitch`, `yaw`, `roll`
- `scale`: size multiplier of the object or character

Dynamic pedestrians additionally support:

- `avoid`: whether obstacle avoidance is enabled
- `velocity`: walking speed
- `trajectory`: waypoint sequence that defines the walking path

---

## Supported Elements

### 1. `cube1`

Use `cube1` for static box-shaped obstacles or large block structures.

```json
{
  "Element1": {
    "name": "obstacle1",
    "type": "static",
    "model": "cube1",
    "position": {
      "x": 700,
      "y": 700,
      "z": 500
    },
    "rotation": {
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    },
    "scale": {
      "x": 10,
      "y": 10,
      "z": 10
    }
  }
}
```

**Recommended use:**

- walls
- buildings
- large obstacles
- road boundaries

**Parameters:**

- `type`: must be `static`
- `model`: must be `cube1`
- `position`: object center position
- `rotation`: object rotation
- `scale`: object size multiplier

<p align="center">
  <img src="../demo_gif/Element/Cube1.png" alt="Cube1" width="600px"/>
</p>

---

### 2. `cube2`

Use `cube2` for static box-shaped obstacles or large block structures, cube2 has a different texture.

```json
{
  "Element2": {
    "name": "obstacle2",
    "type": "static",
    "model": "cube2",
    "position": {
      "x": 950,
      "y": 100,
      "z": 100
    },
    "rotation": {
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    },
    "scale": {
      "x": 0.5,
      "y": 0.5,
      "z": 2
    }
  }
}
```

**Recommended use:**

- columns
- fences
- narrow barriers
- interior obstacles

**Parameters:**

- `type`: must be `static`
- `model`: must be `cube2`
- `position`: object center position
- `rotation`: object rotation
- `scale`: object size multiplier

<p align="center">
  <img src="../demo_gif/Element/Cube2.png" alt="Cube2" width="200px"/>
</p>

---

### 3. `cylinder1`

Use `cylinder1` for cylindrical static obstacles.

```json
{
  "Element3": {
    "name": "obstacle3",
    "type": "static",
    "model": "cylinder1",
    "position": {
      "x": 450,
      "y": -100,
      "z": 100
    },
    "rotation": {
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    },
    "scale": {
      "x": 0.5,
      "y": 0.5,
      "z": 2
    }
  }
}
```

**Recommended use:**

- pillars
- poles
- round barriers
- landmark objects

**Parameters:**

- `type`: must be `static`
- `model`: must be `cylinder1`
- `position`: object center position
- `rotation`: object rotation
- `scale`: object size multiplier

<p align="center">
  <img src="../demo_gif/Element/Cylinder.png" alt="Cylinder1" width="200px"/>
</p>

---

### 4. `human1`

Use `human1` for dynamic pedestrians that move along a defined trajectory.

```json
{
  "Element1": {
    "name": "pedestrian1",
    "type": "dynamic",
    "model": "human1",
    "avoid": true,
    "position": {
      "x": 1400,
      "y": 1200,
      "z": 90
    },
    "rotation": {
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    },
    "scale": {
      "x": 1,
      "y": 1,
      "z": 1
    },
    "velocity": 0.3,
    "trajectory": {
      "point1": {
        "x": 1400,
        "y": -100,
        "z": 0
      },
      "point2": {
        "x": -1400,
        "y": -100,
        "z": 0
      }
    }
  }
}
```

**What this element supports:**

- initial spawn position
- initial orientation
- walking speed
- optional obstacle avoidance
- multi-point trajectory walking

**Parameters:**

- `type`: must be `dynamic`
- `model`: must be `human1`
- `avoid`: `true` or `false`
- `position`: initial spawn position
- `rotation`: initial character orientation
- `scale`: character size multiplier
- `velocity`: walking speed
- `trajectory`: ordered waypoints such as `point1`, `point2`, `point3`

<p align="center">
  <img src="../demo_gif/Element/Human.png" alt="Human1" width="600px"/>
</p>

---

## How Pedestrian Trajectories Work

The pedestrian path is defined by the `trajectory` object.

Example:

```json
"trajectory": {
  "point1": {
    "x": 1400,
    "y": -100,
    "z": 0
  },
  "point2": {
    "x": -1400,
    "y": -100,
    "z": 0
  }
}
```

This means the pedestrian will:

1. spawn at the initial `position`
2. walk toward `point1`
3. continue toward `point2`

You can extend the path by adding more waypoints such as `point3`, `point4`, and so on.

---

## Example 1: Basic Pedestrian Movement

This example creates three pedestrians moving in parallel.

```json
{
  "Element1": {
    "name": "pedestrian1",
    "type": "dynamic",
    "model": "human1",
    "position": {
      "x": 2000,
      "y": 0,
      "z": 90
    },
    "rotation": {
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    },
    "scale": {
      "x": 1,
      "y": 1,
      "z": 1
    },
    "velocity": 0.5,
    "trajectory": {
      "point1": {
        "x": -1000,
        "y": 0,
        "z": 0
      }
    }
  },
  "Element2": {
    "name": "pedestrian2",
    "type": "dynamic",
    "model": "human1",
    "position": {
      "x": 2000,
      "y": 500,
      "z": 90
    },
    "rotation": {
      "pitch": 0,
      "yaw": 180,
      "roll": 0
    },
    "scale": {
      "x": 1,
      "y": 1,
      "z": 1
    },
    "velocity": 0.5,
    "trajectory": {
      "point1": {
        "x": -1000,
        "y": 500,
        "z": 0
      }
    }
  },
  "Element3": {
    "name": "pedestrian3",
    "type": "dynamic",
    "model": "human1",
    "avoid": true,
    "position": {
      "x": 2000,
      "y": -500,
      "z": 90
    },
    "rotation": {
      "pitch": 0,
      "yaw": 90,
      "roll": 0
    },
    "scale": {
      "x": 1,
      "y": 1,
      "z": 1
    },
    "velocity": 0.5,
    "trajectory": {
      "point1": {
        "x": -1000,
        "y": -500,
        "z": 0
      }
    }
  }
}
```

**Use case:** test parallel pedestrian movement and different initial headings.

<p align="center">
  <img src="../demo_gif/Scene/scene1.gif" alt="scene1" width="600px"/>
</p>

---

## Example 2: Pedestrians with Static Obstacles

This example adds `cube1` and `cube2` obstacles to the scene.

```json
{
  "Element1": {
    "name": "pedestrian1",
    "type": "dynamic",
    "model": "human1",
    "avoid": true,
    "position": {
      "x": 2000,
      "y": 0,
      "z": 90
    },
    "rotation": {
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    },
    "scale": {
      "x": 1,
      "y": 1,
      "z": 1
    },
    "velocity": 0.5,
    "trajectory": {
      "point1": {
        "x": -1000,
        "y": 0,
        "z": 0
      }
    }
  },
  "Element2": {
    "name": "pedestrian2",
    "type": "dynamic",
    "model": "human1",
    "avoid": true,
    "position": {
      "x": 2000,
      "y": 500,
      "z": 90
    },
    "rotation": {
      "pitch": 0,
      "yaw": 180,
      "roll": 0
    },
    "scale": {
      "x": 1,
      "y": 1,
      "z": 1
    },
    "velocity": 0.5,
    "trajectory": {
      "point1": {
        "x": -1000,
        "y": 500,
        "z": 0
      }
    }
  },
  "Element3": {
    "name": "pedestrian3",
    "type": "dynamic",
    "model": "human1",
    "avoid": true,
    "position": {
      "x": 2000,
      "y": -500,
      "z": 90
    },
    "rotation": {
      "pitch": 0,
      "yaw": 90,
      "roll": 0
    },
    "scale": {
      "x": 1,
      "y": 1,
      "z": 1
    },
    "velocity": 0.5,
    "trajectory": {
      "point1": {
        "x": -1000,
        "y": -500,
        "z": 0
      }
    }
  },
  "Element4": {
    "name": "obstacle1",
    "type": "static",
    "model": "cube1",
    "position": {
      "x": 1000,
      "y": 600,
      "z": 50
    },
    "rotation": {
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    },
    "scale": {
      "x": 1,
      "y": 1,
      "z": 1
    }
  },
  "Element5": {
    "name": "obstacle2",
    "type": "static",
    "model": "cube2",
    "position": {
      "x": 1000,
      "y": -600,
      "z": 50
    },
    "rotation": {
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    },
    "scale": {
      "x": 1,
      "y": 1,
      "z": 1
    }
  }
}
```

**Use case:** test pedestrian obstacle avoidance in a structured scene.

<p align="center">
  <img src="../demo_gif/Scene/scene2.gif" alt="scene2" width="600px"/>
</p>

---

## Example 3: Mixed Complex Scene

This example combines:

- multiple pedestrians;
- multiple obstacle types;
- different obstacle sizes;
- multi-segment trajectories.

```json
{
  "Element1": {
    "name": "pedestrian1",
    "type": "dynamic",
    "model": "human1",
    "avoid": true,
    "position": {
      "x": 1400,
      "y": 1200,
      "z": 90
    },
    "rotation": {
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    },
    "scale": {
      "x": 1,
      "y": 1,
      "z": 1
    },
    "velocity": 0.3,
    "trajectory": {
      "point1": {
        "x": 1400,
        "y": -100,
        "z": 0
      },
      "point2": {
        "x": -1400,
        "y": -100,
        "z": 0
      }
    }
  },
  "Element2": {
    "name": "pedestrian2",
    "type": "dynamic",
    "model": "human1",
    "avoid": true,
    "position": {
      "x": 1300,
      "y": 1200,
      "z": 90
    },
    "rotation": {
      "pitch": 0,
      "yaw": 180,
      "roll": 0
    },
    "scale": {
      "x": 1,
      "y": 1,
      "z": 1
    },
    "velocity": 0.25,
    "trajectory": {
      "point1": {
        "x": 1300,
        "y": 100,
        "z": 0
      },
      "point2": {
        "x": -1300,
        "y": 100,
        "z": 0
      }
    }
  },
  "Element3": {
    "name": "obstacle1",
    "type": "static",
    "model": "cube1",
    "position": {
      "x": 700,
      "y": 700,
      "z": 500
    },
    "rotation": {
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    },
    "scale": {
      "x": 10,
      "y": 10,
      "z": 10
    }
  },
  "Element4": {
    "name": "obstacle2",
    "type": "static",
    "model": "cube2",
    "position": {
      "x": 950,
      "y": 100,
      "z": 100
    },
    "rotation": {
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    },
    "scale": {
      "x": 0.5,
      "y": 0.5,
      "z": 2
    }
  },
  "Element5": {
    "name": "obstacle3",
    "type": "static",
    "model": "cylinder1",
    "position": {
      "x": 450,
      "y": -100,
      "z": 100
    },
    "rotation": {
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    },
    "scale": {
      "x": 0.5,
      "y": 0.5,
      "z": 2
    }
  },
  "Element6": {
    "name": "obstacle4",
    "type": "static",
    "model": "cube2",
    "position": {
      "x": 1400,
      "y": 900,
      "z": 100
    },
    "rotation": {
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    },
    "scale": {
      "x": 0.5,
      "y": 0.5,
      "z": 2
    }
  }
}
```

**Use case:** test interaction between multiple pedestrians and multiple obstacle types in a richer custom scene.

<p align="center">
  <img src="../demo_gif/Scene/scene3.gif" alt="scene3" width="600px"/>
</p>

---

## Recommended Editing Workflow

For efficient iteration, use this workflow:

1. start from an existing example file;
2. modify only one or two elements at a time;
3. copy the file to `scene/scene.json`;
4. launch MATRiX and choose **Custom Map**;
5. verify object position, scale, and pedestrian trajectory;
6. repeat until the scene behaves as expected.

---

## Common Mistakes

If the custom scene does not load as expected, check the following first:

- the file name is exactly `scene.json`
- the file is placed in the `scene/` directory
- `Element1`, `Element2`, ... keys are unique
- `type` and `model` match supported values
- dynamic pedestrians include a valid `trajectory`
- JSON syntax is valid, including commas and braces
- you selected **Custom Map** in the launcher

---

## Typical Use Cases

This feature is useful for:

- pedestrian avoidance algorithm validation
- multi-agent path planning evaluation
- urban crowd or traffic-style scene construction
- robot navigation testing under custom obstacle layouts
- perception, detection, and trajectory prediction experiments

---

## Summary

The custom scene system lets you describe a complete simulation scene with a single JSON file.

In practice, the workflow is simple:

1. edit `scene/scene.json`
2. launch MATRiX with `./open_sim_launcher`
3. choose **Custom Map**
4. let UE initialize the scene from your JSON definition

With this mechanism, you can rapidly build repeatable scenes containing static obstacles and dynamic pedestrians, while keeping the UE visual scene and MuJoCo physical scene synchronized.
