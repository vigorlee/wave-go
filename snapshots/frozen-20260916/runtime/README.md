**新版 v1：** [实时三维语义地图、点云聚类与物体标签](README_SEMANTIC_V1_CN.md)

> 完整穿障到远角落：运行 `scripts/start_go2w_all_obstacles_corner.sh`，见 [全障碍任务说明](README_ALL_OBSTACLES_CORNER_CN.md)。

> 新增开发入口：**Cosmos 远端探索 + 实时 3D 建图**，从空白地图启动，见 [本地运行说明](README_ONLINE_EXPLORATION_CN.md)。下面原数据链与运行章节描述已有基线。

> 当前保存版本：**Wavegov1**。原 YardWorld 楼梯、斜坡与固定障碍；MID360 10 Hz、关闭 UE 绿色扫描点、四窗口与真实模型感知。
> 启动、测试与回退见 [Wavegov1 本地说明](README_WAVEGOV1_CN.md)。历史模型与实验记录见 [算法说明](README_ANYTHING_CN.md)。

# MATRiX Go2-W LCM Demo

**本地复现入口：** [三类场景运行与验收](README_EXTENDED_DEMO_CN.md) · [避障点云、语义信息与图片采集](README_POINTCLOUD_SEMANTICS_CN.md)

本项目按 `MATRiX公开课.pdf` 的实操链路复现 MATRiX + RoamerX 导航 demo，机器人替换为 Unitree Go2-W，速度桥使用 LCM。

## 数据链

```text
MATRiX /odom/mujoco_odom -> robot_forward -> map -> odom -> base_link
MATRiX /livox/lidar      -> local_costmap obstacle_layer
NavFn /plan -> MPPI /cmd_vel_nav -> smoother /cmd_vel
/cmd_vel -> vel_cmd_lcm_pub -> LCM vel_cmd_lcm_data -> DreamWaQ -> eCAL mujoco_cmd
```

固定运行参数：

- ROS 2 Humble，`RMW_IMPLEMENTATION=rmw_zenoh_cpp`
- `ROS_DOMAIN_ID=89`
- LCM URL：`udpm://239.255.76.67:7667?ttl=255`
- LCM channel：`vel_cmd_lcm_data`
- MATRiX robot：`go2w`，scene：YardWorld（ID 3）
- 默认控制器：Go2-W DreamWaQ RL（`GO2W_CONTROLLER=rl`）
- 导航初始航向：`MATRIX_ROBOT_INITIAL_YAW_DEG=0`（地图与 MuJoCo 的 `+X`）

## 运行

首次构建：

```bash
cd /home/unitree/matrix_go2w_lcm_demo
./scripts/build_demo.sh
```

启动并检查：

```bash
./scripts/start_demo.sh
./scripts/check_demo.sh
```

在 RViz 使用 `2D Goal Pose`，或按课件示例下发 `(2, 1, 0.1)`：

```bash
./scripts/send_goal.sh 2.0 1.0 0.1
```

停止：

```bash
./scripts/stop_demo.sh
```

## RL 楼梯策略

项目包含公开 Go2-W DreamWaQ 预训练权重的 eCAL/LCM 控制桥。先用独立 MuJoCo
楼梯基准验证策略，不启动 UE 和导航：

```bash
./scripts/build_demo.sh
./scripts/start_stair_policy.sh --test
./scripts/check_stair_policy.sh
```

`--test` 会在策略稳定接管后发送 `0.8 m/s` 前进指令，并用横向位置/航向闭环保持在
楼梯中央，到达平台后自动把前进速度归零。控制器默认使用训练时的
5 帧历史时序（历史帧不含速度指令），这是楼梯策略正常触发抬腿动作的必要条件。
正常 LCM 模式不带该参数：

```bash
./scripts/start_stair_policy.sh
```

控制链为 `LCM vel_cmd_lcm_data -> DreamWaQ (CPU, 50 Hz) -> eCAL mujoco_cmd`，
电机指令以 500 Hz 保持发送。停止基准：

```bash
./scripts/stop_stair_policy.sh
```

预训练权重来自 `third_party/DreamWaQ_Go2W`；上游仓库未声明顶层许可证，重新分发前需单独确认授权。

日志位于 `logs/`。排错顺序与课件一致：先看进程，再看话题接口，最后看话题数据与 TF。

需要回退到旧的平地 PD/轮速控制器时：

```bash
GO2W_CONTROLLER=pd ./scripts/start_demo.sh
```

## Cosmos3-Edge VLN

### 楼梯/双坡道中央圆柱长程 Demo

扩展场景包含上楼梯、下楼梯、上坡、下坡中央圆柱、平地交错圆柱和动态行人。
完整运行、四窗口录屏、停止命令与验收字段见
[`README_EXTENDED_DEMO_CN.md`](README_EXTENDED_DEMO_CN.md)。直接录制：

```bash
cd /home/unitree/Go2w+cosmos3Nav/matrix_go2w_lcm_demo
OUT="$PWD/artifacts/cosmos_center_obstacles_$(date +%Y%m%d_%H%M%S)"
COSMOS_VLN_ARTIFACT_DIR="$OUT" \
  ./scripts/record_cosmos_extended_desktop_demo.sh
```

2026-09-10 修复了 UE 坡道方向、坡顶接缝、静态障碍显示及动态代理同步。
当前复测结果、录像和完整命令见
[`README_EXTENDED_DEMO_CN.md`](README_EXTENDED_DEMO_CN.md)。动态障碍使用可见的移动高圆柱代理。
历史2026-09-03产物保留用于追溯，不作为这次视觉修复的验收依据。

Cosmos3-Edge 作为高级指令层接入 ROS 2。它读取英文任务和最新前视图像，只能从
`config/cosmos_vln_routes.json` 的白名单中选择路线并预测路线级未来风险。当前模型和
推理环境位于：

```text
/home/unitree/matrix_g1_lcm_demo/Cosmos3-Edge
/home/unitree/matrix_g1_lcm_demo/packages/cosmos-framework/.venv
```

控制链边界固定为：

```text
English high-level task + camera
  -> Cosmos: allowlisted route_id + future-risk prediction
  -> mission supervisor: route_id -> trusted staged goals
  -> Nav2: global/local planning and obstacle avoidance
  -> DreamWaQ: locomotion, including stair ascent and descent
```

Cosmos 输出：

- `/cosmos_vln/route_command`：`action + route_id + confidence` 结构化高级指令
- `/cosmos_vln/future_prediction`：路线完成后的预期观测、风险和任务进度
- `/cosmos_vln/status`：推理状态和耗时

提示词只向模型暴露路线的 `route_id`、描述和英文任务示例，不暴露阶段、坐标或路径。
Cosmos 桥没有 Nav2 ActionClient，也不能输出或发布速度、转向、步态、局部位移、坐标、
航点或轨迹。未来预测只用于可视化和风险说明，不参与运动控制。任务监督器独占 Nav2
`/navigate_to_pose`，并将通过校验的 `route_id` 展开为路线目录拥有的阶段目标。

送入 Reasoner 的图像默认等比例缩放到最长边 640 px，以限制视觉注意力显存；可通过
`COSMOS_VLN_MAX_IMAGE_EDGE_PX` 调整，允许范围为 256 到 1024。

### 一条命令运行完整楼梯任务

以下命令会停止当前 demo、把机器狗重置到楼梯中央入口、以离屏和禁用独立 RHI 线程的模式
启动 UE 传感器渲染，并打开只读 RViz 与标注相机窗口。完成健康检查后脚本会下发英文高级任务，等待并结构化
解析模型返回；只有 `navigate_to_route / far_end_via_stairs` 才继续，错误路线、保持命令、
无法解析或超时都会立即取消任务。监督器也会在发送任何 Nav2 目标前执行同一 required-route
校验；启动检查还会拒绝重复的监督器进程：

```bash
cd /home/unitree/matrix_go2w_lcm_demo
./scripts/run_cosmos_full_stair_demo.sh
```

必须以 `unitree` 桌面用户执行，不能使用 `sudo`。默认 GUI 使用当前 `DISPLAY`；若终端没有
继承桌面环境，脚本会尝试 `DISPLAY=:0` 和当前用户的 GDM Xauthority。脚本在确认正确路线
后返回，三阶段任务和可视化继续运行；完整任务历史成功耗时约 125 秒。

脚本下发的任务是：

```text
Complete the full stair course: climb to the upper landing, descend the far staircase, then continue to the farthest safe endpoint. Do not stop at the upper landing.
```

期望 Cosmos 返回：

```text
action: navigate_to_route
route_id: far_end_via_stairs
```

监督器随后只从本地白名单解析并执行三个阶段：

| 阶段 | 模式 | 目录目标 | 完成条件 |
| --- | --- | --- | --- |
| `climb_to_upper_landing` | `up` | `(0.2, 4.8)` | 上层平台稳定验证通过 |
| `descend_far_staircase` | `down` | `(0.2, 10.2)` | 下层平台稳定验证通过 |
| `navigate_to_far_end` | `avoid` | `(0.4, 14.4)` | Nav2 目标成功 |

这些坐标只存在于受信任的路线目录，不会发送给 Cosmos。最后一个点是当前地图中已经完整
验证的最远安全终点，不代表理论地图边界。

### 分步运行

需要分别控制启动和任务下发时，从干净终端执行：

```bash
cd /home/unitree/matrix_go2w_lcm_demo

export MATRIX_ROBOT_INITIAL_X=0.2
export MATRIX_ROBOT_INITIAL_Y=0.0
export MATRIX_ROBOT_INITIAL_YAW_DEG=90
export MATRIX_UE_OFFSCREEN=1
export MATRIX_UE_NO_RHI_THREAD=1
export ROAMERX_RVIZ_READ_ONLY=1
export COSMOS_VLN_MISSION_TIMEOUT_SEC=900
export COSMOS_VLN_MISSION_REQUIRED_ROUTE_ID=far_end_via_stairs
export COSMOS_VLN_INFERENCE_TIMEOUT_SEC=900
export MATRIX_CAMERA_FREQUENCY=1

./scripts/start_go2w_cosmos_vln.sh

export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
COSMOS_VLN_VISUALIZATION_RVIZ=1 \
COSMOS_VLN_VISUALIZATION_IMAGE_WINDOW=1 \
  ./scripts/start_cosmos_vln_visualization.sh

./scripts/check_demo.sh
./scripts/check_cosmos_vln.sh
```

再下发英文高级任务：

```bash
cd /home/unitree/matrix_go2w_lcm_demo
set +u
source /opt/ros/humble/setup.bash
source /home/unitree/matrix_go2w_lcm_demo/genisom_roamerx_open/install/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=89

ros2 topic pub --once /cosmos_vln/mission std_msgs/msg/String \
  "{data: 'Complete the full stair course: climb to the upper landing, descend the far staircase, then continue to the farthest safe endpoint. Do not stop at the upper landing.'}"
```

### 状态与世界模型可视化

可在两个独立终端查看 Cosmos 选择和三阶段状态：

```bash
cd /home/unitree/matrix_go2w_lcm_demo
set +u
source /opt/ros/humble/setup.bash
source genisom_roamerx_open/install/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=89

ros2 topic echo --full-length /cosmos_vln/route_command
```

```bash
cd /home/unitree/matrix_go2w_lcm_demo
set +u
source /opt/ros/humble/setup.bash
source genisom_roamerx_open/install/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=89

ros2 topic echo --full-length /cosmos_vln/mission_status
```

RViz 订阅 `/cosmos_vln/markers` 显示监督器解析后的全局目标和预测文本；标注后的相机
画面发布在 `/cosmos_vln/annotated_image`，同时保存为
`.run/cosmos_vln/latest_visualization.jpg`。颜色按置信度区分：低置信度红色、中等
置信度黄色、高置信度绿色。RViz 固定为只读，避免人工目标和任务监督器争用。

不打开 GUI 时仍可直接查看最新 JPEG：

```bash
xdg-open /home/unitree/matrix_go2w_lcm_demo/.run/cosmos_vln/latest_visualization.jpg
```

停止整套系统：

```bash
cd /home/unitree/matrix_go2w_lcm_demo
./scripts/stop_go2w_systemd.sh
```

### 安全边界

协议会拒绝未知 route、额外字段、非法 profile/mode 配对以及任何速度、局部位移、坐标、
航点或轨迹字段。以下情况均按 fail-closed 终止或保持停止，并在结束后恢复 `avoid`：

- 完整任务入口收到的路线不是 `far_end_via_stairs`
- Cosmos 置信度低于 `0.35`
- 相机帧或里程计陈旧
- 起点位置、横向偏差、朝向或姿态不安全
- 阶段越界、长时间无进展或任务超时
- 上层/下层平台稳定验证失败
- Nav2 拒绝目标或导航失败

当前结论只适用于 YardWorld 固定仿真地图、指定楼梯入口和已经验证的白名单路线，不能外推为
未知环境或实机安全保证。新增目标必须先单独完成规划、导航和安全验证，再加入路线目录。

### 验证与成功证据

协议和提示词隔离测试：

```bash
cd /home/unitree/matrix_go2w_lcm_demo
python3 -m unittest tests/test_cosmos_vln_protocol.py -v
```

`check_demo.sh` 和 `check_cosmos_vln.sh` 只检查进程、话题和接口健康，不等同于端到端任务
通过。完整验收应确认：Cosmos 精确选择 `far_end_via_stairs`，阶段按 `up -> down -> avoid`
执行，上下平台均验证成功，最终 Nav2 状态为 `SUCCEEDED`，最终模式恢复 `avoid`，且没有安全
事件。

已完成的一次完整仿真验收：

- 结果：`SUCCEEDED`
- 总耗时：`124.65 s`
- Cosmos：`navigate_to_route / far_end_via_stairs`，置信度 `0.90`
- 最终稳定位置：约 `(x=0.29, y=14.05)`
- 最终模式：`avoid`
- 安全事件：`0`

![Cosmos full stair-to-far-end result](artifacts/cosmos_full_stair_far_20260723_144853/final_visualization.jpg)

完整日志、原始 Cosmos 作业、命令、三阶段状态、最终位姿与图片说明位于
[`artifacts/cosmos_full_stair_far_20260723_144853/`](artifacts/cosmos_full_stair_far_20260723_144853/README.md)。

## Go2-W 适配说明

MATRiX 0.1.2 发布了 Go2-W 模型，但其 `robot_mujoco` 二进制只在 XGW/ZGWS 分支处理每条腿的第 4 个轮关节。项目因此在物理侧使用 XGW 控制布局，并把 Go2-W 的 16 个 actuator 调整为每腿 `hip/thigh/calf/wheel` 交错顺序；UE5 侧仍加载官方 `go2w` 外观与传感器。桥接器持续发布完整的 20 组 `RobotCmd` 字段，并在 LCM 命令超时 300ms 时自动停轮。
