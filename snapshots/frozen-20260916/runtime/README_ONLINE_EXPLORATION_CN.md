# Cosmos 远端探索与实时 3D 建图（开发版）

从空白地图启动，MID360 与 IMU 实时估计运动并积累 3D 点云。Cosmos3-Edge 读取当前相机图像和雷达观测产生的可达候选点（包含已经扫描完整的通道尽头），沿机器人初始前进方向向更远处推进；Nav2 执行局部规划和避障。二维地图保留已测得的低矮障碍；从其上方穿过的射线不会清掉它，实际地面端点重复观测才允许清除。这里的“最远”按初始前进方向上的已观测可达进度判断，不能在未知环境中保证全局最远点。

## 启动与显示

```bash
cd /home/unitree/matrix_go2w_lcm_demo
bash scripts/start_go2w_cosmos_mapping.sh
```

保留原 YardWorld 的楼梯、坡道与障碍物，显示 MuJoCo、UE、RViz 3D 建图、Depth Anything / DINO / SAM 四窗口。此次导航不读取预建地图或固定路线目录。启动会重置仿真并重新建图。

只运行传统边界探索、不调用 Cosmos：

```bash
GO2W_COSMOS_FARTHEST=0 bash scripts/start_go2w_exploration.sh
```

停止：

```bash
systemctl --user stop go2w-navigation.service go2w-anything-models.service go2w-anything-view.service
bash scripts/stop_demo.sh
```

## 实际算法与数据链

```text
UE /livox/lidar_raw → 保留真实测量字段的距离过滤 → /livox/lidar
MuJoCo /imu + /livox/lidar → RoamerX robot_slam（FAST-LIO 系列）
    → LIO 估计位姿 → map 到 base_link 的 TF、导航里程计
    → 配准点云 → 0.10 m 体素累积地图 → RViz
    → 测量射线投影 → 从未知开始的二维占据栅格 /map
实时图像 + 当前连通自由空间边界 → 本地 Cosmos3-Edge → 候选 ID
    → 当前几何复核 → Nav2 → LCM → DreamWaQ → 仿真机器人
```

建图及导航节点没有订阅真值里程计，不启动 `robot_forward`、预建 `map_server` 或人工障碍点云发布器。Super-LIO 尚未接通，本版不能称为 Super-LIO 建图。当前没有回环优化。

Cosmos 每次使用真实本地模型推理，保存输入图像、请求、模型原文、最终选择和日志。模型只能选择测量候选 ID；输出异常、低置信度、暂停判断、传感器过期或候选失效时，不擅自改成固定路线。当前按“停车推理、再执行”的方式运行，推理耗时以各次记录为准，不能视为高频控制器。

RViz 的蓝到金色是**点高度着色**，不是语义分类；左下窗口的 Depth Anything、DINO 和 SAM 是单独的真实相机推理显示，尚未将其融合成全局 3D 语义地图。

## 话题与输出

| 内容 | ROS 2 话题 |
|---|---|
| 实时积累 3D 点云 | `/wavego/mapping/points` |
| 实测局部扫描 | `/livox/lidar` |
| 在线二维占据地图 | `/map` |
| LIO 实际行走轨迹 | `/wavego/mapping/trajectory` |
| 当前探索边界 | `/wavego/exploration/frontiers` |
| 导航里程计 | `/wavego/navigation/odom` |
| 状态、点数与进展 | `/wavego/exploration/status` |

输出默认保存在 `artifacts/exploration_时间/`，也可通过 `GO2W_EXPLORATION_OUTPUT` 指定目录：

- `map.pcd`：实际测量得到的 xyz 3D 点云，可在 CloudCompare 等工具打开。
- `map.npz`：相同点云、二维占据数据、分辨率与地图原点；未知栅格为 -1。
- `summary.json`：点数、扫描数、目标数、达到目标数、LIO 路程与距起点距离。
- `events.jsonl`：选择、到达、超时、Cosmos 决策与失败记录。
- `cosmos_*/`：模型输入、推理日志和原始输出。

“到达边界”指机器人进入目标 0.60 m 范围后取消该局部动作并继续探索，不等同于每次 Nav2 返回成功。单次目标 75 秒超时会取消并重新选点。累计行走距离来自 LIO，包含估计误差。

## 当前边界

二维导航投影使用初始地面高度与保守障碍高度阈值；此版本验证的是可达平地区域探索，**不能据此宣称具备无图自动上下楼梯能力**。楼梯仍可进入实时 3D 点云，但地形可通行性分析和楼梯运动模式切换还需单独接入。地图当前范围 60 × 60 m，3D 体素上限 50 万，适合当前场景实验。

已保存的 `Wavegov1` GitHub 分支及 `Wavegov1-baseline` 标签不改动。本地恢复固定场景基线请按 [Wavegov1 文档](README_WAVEGOV1_CN.md)，并关闭本开发版的运行进程。

## 本机 SLAM 修改与重建

`robot_slam` 增加 `publish.archive_raw_map` 开关，默认保持原行为；本模式关闭未经降采样的无限点云归档，改由有上限的体素地图保存。更换机器或恢复源码后，需要重建修改过的包：

```bash
source /opt/ros/humble/setup.bash
source genisom_roamerx_open/install/setup.bash
cd genisom_roamerx_open
PATH=/usr/bin:/bin:$PATH CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-select robot_slam --executor sequential --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

本机已完成此构建。运行还依赖现有 MATRiX、RoamerX 安装及 `/home/unitree/matrix_g1_lcm_demo/Cosmos3-Edge` 模型权重；开发快照不包含这些大型资源。

## 检查

```bash
source /opt/ros/humble/setup.bash
source genisom_roamerx_open/install/setup.bash
.venv-anything/bin/python -m unittest discover -s tests -q
```

开发运行证据见 `artifacts/Wavegov2_cosmos_farthest_take6/`；检查实际 `events.jsonl` 和 `summary.json`，不要把目标已发出当作已经到达。

## 最终实测记录

`artifacts/Wavegov2_cosmos_farthest_take6/` 从空白地图启动，4 个 Cosmos 导航目标全部进入到达范围，LIO 前向进度约 29.3 m；本轮没有恢复地图或重启导航节点。最终状态为 `farthest_observed_reached`，位置接近当前 60 × 60 m 导航栅格前方边界，不能据此宣称已找到地图范围以外的全局最远点。推理过程中会停车，当前尚非全程无停顿控制。

78 项自动测试通过，`ros_graph_audit.json` 记录实际节点与订阅，`live_stream_audit.json` 记录真实话题更新频率，`verification.json` 固定此次核验结果，`live_result.png` 为四窗口实拍截图。`implementation_snapshot.tar.gz` 和 `implementation_sha256.json` 保存本地源码快照；大型模型和运行依赖不在快照内。
