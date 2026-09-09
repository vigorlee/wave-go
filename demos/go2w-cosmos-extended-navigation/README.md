# Go2-W × Cosmos3-Edge：实体坡连续长程导航

这是 WAVE-Go 仓库中的第二条 demo 线：**Cosmos3-Edge 只在任务开始时选择一次白名单路线；Nav2/RoamerX 负责全局/局部规划和动态避障；DreamWaQ 负责 Go2-W 底层运动**。它与仓库根目录的 mapless charging 闭环不同，不能把这条链路描述成“世界模型直接输出 Go2-W 速度”。

## 已验证版本

发布证据来自 2026-09-03 的 MuJoCo/UE HouseWorld 运行：

| 指标 | 结果 |
| --- | ---: |
| 路线 | `farthest_end_via_stairs_ramps_cylinders_pedestrians` |
| 阶段 | 18/18 |
| 导航动作 | `NavigateThroughPoses=1`，中间重发 `=0` |
| 连续路径里程 | 60.15 m |
| 运行时长 | 311.9 s |
| 实体坡高度 | 0.413 → 0.851 → 0.397 m |
| 坡面高差 | +0.438 m / −0.454 m |
| 中央地形圆柱 | 4 个，全部通过安全距离验收 |
| 动态行人 | 3 个，最近中心距 0.974 m |
| 画面 | 2560×1440，H.264，15 fps，347.1 s |

证据索引在 [`evidence/README.md`](evidence/README.md)，结果摘要在 [`evidence/result.json`](evidence/result.json)。完整 MP4 和坡道片段作为 GitHub Release 资产发布，不把大视频塞进 Git 历史。

## 目录

```text
config/       路线白名单、策略参数、RViz 配置
scene/        MuJoCo/UE 配对场景与 MATRiX 动态演员 JSON
scripts/      Cosmos bridge、一次性连续 mission、点云、验证和启动器
tests/        协议、mission、地形策略回归测试
evidence/     精简机器可读结果与关键帧
```

## 运行前提

仓库不捆绑以下机器相关依赖：Ubuntu/ROS 2 Humble、MATRiX/HouseWorld、RoamerX install、Go2-W DreamWaQ bridge、CUDA/NVIDIA 驱动、Cosmos3-Edge checkpoint。请在已准备好的工作站上设置：

```bash
export WAVE_GO_RUNTIME_ROOT=/home/unitree/matrix_go2w_lcm_demo
export COSMOS_VLN_ROOT=/home/unitree/matrix_g1_lcm_demo
```

`WAVE_GO_RUNTIME_ROOT` 必须包含 `genisom_roamerx_open/install/setup.bash` 和已有的 Go2-W 运行脚本；`COSMOS_VLN_ROOT` 必须包含 `packages/cosmos-framework/.venv` 与 `Cosmos3-Edge`。

## 命令

先做不启动仿真的静态检查：

```bash
cd demos/go2w-cosmos-extended-navigation
./validate.sh
```

只启动运行栈（便于手动观察）：

```bash
./scripts/start_extended_stack.sh
./scripts/start_cosmos_vln_visualization.sh
```

运行一次完整任务并按需录制桌面：

```bash
OUT="$PWD/artifacts/extended_navigation_$(date +%Y%m%d_%H%M%S)"
COSMOS_VLN_ARTIFACT_DIR="$OUT" ./scripts/record_demo.sh
```

停止：

```bash
./scripts/stop_extended_stack.sh
```

脚本以普通桌面用户执行，不要使用 `sudo`。录制器默认使用 `$DISPLAY` 的 4096×2304 桌面并输出 H.264；无图形会话时仍会运行任务并保留 `result.json`，但不会生成 MP4。

## 场景和控制边界

- 楼梯和坡道中央放置 `Cylinder37/38/39/41` 四个真实障碍；另有四个楼梯后交错圆柱。
- 坡面是 +10°、0.42 m 坡顶、−10° 的真实 MuJoCo 碰撞几何；UE skin 与物理坡同角度，只偏移 5 mm 用于显示。
- 三个动态行人由 MATRiX JSON 轨迹进入世界模型点云；UE 中另有紫色 render-only 滑轨代理，避免重复碰撞源。
- Cosmos 只能发布 `route_id`、未来观测和风险；mission supervisor 独占 Nav2 action。
- 一次任务只发送一个 `NavigateThroughPoses`；经过通过点时更新地形模式，不取消或重发动作。
- `GO2W_RL_STAIR_OBSTACLE_AVOIDANCE=1` 只对本扩展场景开启；原稳定 YardWorld 不受影响。

## 限制

这是固定仿真场景中的工程回归证据，不是真机安全保证，也不是未知场景泛化证明。`result.json` 的高度来自 `/odom/mujoco_odom`，没有足端力、接触力或关节力矩通道；不要将它写成真实机器人接触验证。
