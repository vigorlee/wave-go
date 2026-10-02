# 穿越全部障碍，到东侧最远角落

本入口执行完整必经地形任务：上楼 → 下楼 → 圆柱障碍区 → 上坡 → 下坡 → 穿过北侧通道 → 绕北侧转角 → 横穿长廊 → 进入东侧走廊 → 到达记录中的最远角落。终点为原场景 `map` 坐标 `(18.0, 22.8)`，需要走过 `y≈32.8` 的转角和东侧长廊；不是在起点前方 30 米处结束。

```bash
cd /home/unitree/matrix_go2w_lcm_demo
bash scripts/start_go2w_all_obstacles_corner.sh
```

启动会重置仿真。Cosmos3-Edge 读取现场相机图像并确认完整任务，随后通过一个 `NavigateThroughPoses` 连续任务执行 19 个阶段。前 11 阶段完整保留已验证的 Wavegov1 楼梯、圆柱与坡道顺序，后续保留历史远端路线的 6 个阶段，并在北侧长廊增加 2 个避让占用区域的途经点。未增加或移动原有地形。

## 实际分工

- **Cosmos**：真实本地模型推理，选择完整地形任务；输入图像、请求及模型原文保存到 `cosmos_jobs/`。
- **导航与运动**：使用已验证的场景路线、原有导航地图、MuJoCo 位姿及地形控制策略。该模式不是纯无图探索，也不是 Cosmos 从零生成整条路线。
- **实时 3D 建图**：独立的 FAST-LIO 系列 `robot_slam` 仅接收真实 MID360 原始测量的过滤结果和 IMU，未接入导航用的人工障碍融合点云。地图保持实时累积。
- **四窗口**：MuJoCo、UE、RViz 实测 3D 点云、Depth Anything / DINO / SAM 图像。

导航使用原场景 `map/base_link`；建图使用独立 `lio_map/lio_base_link`。两套 TF 不覆盖对方。RViz 固定坐标系为 `lio_map`，点云按高度着色，不代表全局语义类别。建图节点不会发出探索目标，不能抢占完整穿障任务。

## 数据与验收

默认输出目录 `artifacts/all_obstacles_corner_时间/`，可用 `COSMOS_VLN_ARTIFACT_DIR` 指定。

| 文件 | 内容 |
|---|---|
| `course/events.jsonl` | Cosmos 选择、阶段切换、到达与失败原因 |
| `course/result.json` | 19 阶段进度、连续动作数量、坡道物理高度验证 |
| `course/odom.csv` | 实际 MuJoCo 位置、姿态与阶段采样 |
| `cosmos_jobs/` | 模型相机输入、请求、推理日志与原始输出 |
| `mapping/map.pcd` | 持续保存的实测 3D 点云 |
| `mapping/summary.json` | **仅建图节点**的点数与扫描统计，不代表导航完成状态 |

导航完成以 `course/result.json` 中 `passed=true`、`completed_stages=19`、坡道物理验证通过及终点位置为依据。判断同时保留实际轨迹，不能仅凭发送目标或点云覆盖声称通过障碍。

停止全部任务：

```bash
systemctl --user stop go2w-course.service go2w-navigation.service go2w-mapping-sidecar.service go2w-anything-models.service go2w-anything-view.service
bash scripts/stop_demo.sh
```

纯在线探索版本仍可参考 [在线建图说明](README_ONLINE_EXPLORATION_CN.md)，原地形基线见 [Wavegov1](README_WAVEGOV1_CN.md)。

## 2026-09-13 早期失败记录（后续已完成全程）

- `artifacts/all_obstacles_corner_take1/`：原 17 阶段完成 13 个，全部楼梯、圆柱和坡道通过，北侧长廊占用区域导致停滞。
- 加入北侧长廊 2 个避让点后，`artifacts/all_obstacles_corner_take2/` 完成 19 阶段中的 15 个，仅发送 1 次连续动作。已经越过原停滞位置，实际走到 `(16.787, 32.857)`，坡顶升高 0.446 米。
- 第二轮在进入东侧走廊前，UE 报告 `UGameEngine::Tick.ViewportClosed` 并以状态 0 退出，随后仿真联动停止。第三轮重试也出现同样的窗口关闭事件，未完成。
- **不能将以上记录标记为全程通过。** 东侧最终角落仍需保持 UE 窗口开启后完整运行验证。
- 80 项本地测试通过；运行证据和失败原因均保留。此前发布的 Wavegov1 基线未覆盖。

楼梯平台上的四窗口实测截图：

![楼梯平台、实时点云与深度语义](artifacts/all_obstacles_corner_take2/stairs_live_mapping.png)

后续 `artifacts/semantic_v1_take1/course/result.json` 已实测通过 19/19 阶段，一次连续动作，到达 `(18.056, 23.037)`；语义最终版本与复跑数据见 [新版 v1](README_SEMANTIC_V1_CN.md)。
