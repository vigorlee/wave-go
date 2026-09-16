# 自定义场景仿真平台

本仿真平台推出了**自定义场景功能**，用户只需通过简单的 JSON 文件即可定义场景结构和行为逻辑。系统会自动生成对应的**静态场景、动态行人及物理仿真配置**，实现高效、灵活的测试验证。

---

## 🌍 功能概览

通过在 JSON 文件中定义场景元素，平台自动生成对应的 **Mujoco XML 文件**，并在虚幻引擎 (UE) 中构建同步的 3D 仿真场景，实现**视觉与物理仿真的统一**。

### 核心特性

- **静态场景构建**
  使用可变尺寸的立方体和圆柱体元素构建建筑物、障碍物和道路等结构。

- **动态行人系统**
  - 支持多行人并行仿真
  - 可配置行走速度
  - 自定义多段线轨迹定义
  - 自动避障与路径调整

- **地形与路径多样性**
  用户可以自由定义地形类型、路径布局、行人数量及分布，快速模拟各种现实世界场景。

---

## 🧱 支持的元素与参数

### Cube1 (立方体1)

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

**参数说明:**
- `name`: 元素的唯一标识符
- `type`: 元素行为类型 (`static` 表示固定障碍物)
- `model`: 几何形状 (`cube1`)
- `position`: 世界空间中的 3D 坐标 (x, y, z，单位: cm)
- `rotation`: 欧拉角 (pitch, yaw, roll，单位: 度)
- `scale`: 各轴向的缩放倍数

<p align="center">
  <img src="../demo_gif/Element/Cube1.png" alt="Cube1" width="600px"/>
</p>

---

### Cube2 (立方体2)

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

**参数说明:**
- `name`: 元素的唯一标识符
- `type`: 元素行为类型 (`static` 表示固定障碍物)
- `model`: 几何形状 (`cube2`)
- `position`: 世界空间中的 3D 坐标 (x, y, z，单位: cm)
- `rotation`: 欧拉角 (pitch, yaw, roll，单位: 度)
- `scale`: 各轴向的缩放倍数

<p align="center">
  <img src="../demo_gif/Element/Cube2.png" alt="Cube2" width="200px"/>
</p>

---

### Cylinder1 (圆柱体1)

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

**参数说明:**
- `name`: 元素的唯一标识符
- `type`: 元素行为类型 (`static` 表示固定障碍物)
- `model`: 几何形状 (`cylinder1`)
- `position`: 世界空间中的 3D 坐标 (x, y, z，单位: cm)
- `rotation`: 欧拉角 (pitch, yaw, roll，单位: 度)
- `scale`: 各轴向的缩放倍数

<p align="center">
  <img src="../demo_gif/Element/Cylinder.png" alt="Cylinder1" width="200px"/>
</p>

---

### Human1 (动态行人)

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

**参数说明:**
- `name`: 行人的唯一标识符
- `type`: 元素行为类型 (`dynamic` 表示移动实体)
- `model`: 角色模型 (`human1`)
- `avoid`: 启用避障行为 (`true`/`false`)
- `position`: 初始生成坐标 (x, y, z，单位: cm)
- `rotation`: 初始朝向 (pitch, yaw, roll，单位: 度)
- `scale`: 各轴向的缩放倍数
- `velocity`: 行走速度 (米/秒)
- `trajectory`: 多段线路径定义，包含一系列航点 (point1, point2, 等)

<p align="center">
  <img src="../demo_gif/Element/Human.png" alt="Human1" width="600px"/>
</p>

---

## 📋 场景定义示例

### 示例 1: 基础行人移动

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

<p align="center">
  <img src="../demo_gif/Scene/scene1.gif" alt="scene1" width="600px"/>
</p>

---

### 示例 2: 行人与静态障碍物

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

<p align="center">
  <img src="../demo_gif/Scene/scene2.gif" alt="scene2" width="600px"/>
</p>

---

### 示例 3: 多障碍物复杂场景

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

<p align="center">
  <img src="../demo_gif/Scene/scene3.gif" alt="scene3" width="600px"/>
</p>

---

## ⚙️ 自动转换与同步

平台自动执行以下操作：
- 解析 JSON 文件并生成 Mujoco XML 配置
- 同步 UE 场景与 Mujoco 物理引擎
- 确保视觉模型与物理碰撞体对齐，实现精确仿真

---

## 🚶‍♂️ 应用场景

- **行人避障算法验证**
  在受控环境中测试智能体避障行为

- **多智能体路径规划**
  评估多个自主智能体的协作策略

- **城市交通与人群仿真**
  模拟城市场景中真实的行人流动模式

- **机器人导航测试**
  在各种地形条件下验证导航算法

- **行人检测与预测**
  评估检测和轨迹预测模型的性能

---

## 🚀 核心优势

- **快速建模**
  使用简单的 JSON 定义复杂场景，无需手动构建

- **灵活扩展**
  支持多种几何图元和地形类型，易于扩展

- **高效验证**
  快速切换场景配置，测试算法的鲁棒性和泛化能力

- **物理一致性**
  同步的 UE 视觉场景和 Mujoco 物理引擎确保仿真精度

---

## 📄 总结

自定义场景功能使用户能够以极低的工作量构建多样化的仿真环境。通过自动化的 JSON → XML → UE 转换流程，实现了从结构定义到物理仿真的无缝集成，为算法开发和测试提供了一个高效、可扩展的实验平台。
