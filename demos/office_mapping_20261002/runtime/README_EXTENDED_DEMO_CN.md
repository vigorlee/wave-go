# Go2W + Cosmos3-Edge 三类障碍场景

本版本复现历史任务 `01a06026-64cc-74d1-b18d-53f5bd25fc53` 的三类场景：楼梯中央圆柱、实体双坡道中央圆柱、平地交错圆柱与动态行人代理。Cosmos3-Edge 在开始时选择批准路线，Nav2/RoamerX 负责规划与 MPPI 避障，DreamWaQ 控制机器狗运动。18个阶段由一个 `NavigateThroughPoses` 动作连续执行。

## 2026-09-10 修复

- **坡道方向与承重面**：MuJoCo 的上坡为 +10°，下坡为 −10°。UE 对倾斜方体的 roll 显示方向相反，因此只对两块可视坡面做符号补偿。坡顶连接块的半长度增至 0.33 m，覆盖原来的厘米级接缝。碰撞射线扫描确认坡面内部没有落到地面的空隙。
- **障碍物显示**：楼梯和坡道的四个中央圆柱使用与碰撞体相同尺寸、相同位置的可视副本；补齐四个平地交错圆柱、坡顶与坡底新增圆柱的显示。原 UE 地图中三个交通桶保留在原坐标，补上半径 0.20 m、高 0.80 m 的保守碰撞包络；下坡出口目标移到右侧 `(0.92, 18.25)`，让开交通桶。
- **动态障碍显示与同步**：UE 的动态更新依赖 mesh 数据 ID 及资源名中的 `OBS`。普通 capsule、cylinder 即使日志显示 `Visible: 1`，也不会随关节更新。现使用带命名网格的拟合圆柱，半径约 0.22 m、高约 1.74 m；画面中是移动的高圆柱代理，并非写实人体。第三个代理改为沿 y 方向横穿东西走廊。三个代理由 MuJoCo 滑动关节驱动，导航点云直接读取同一份实时 qpos，修正了原来 JSON 时钟、关节参考值和画面不同步的问题。
- **楼梯停滞恢复**：在首段楼梯、圆柱前的无障碍区，允许轻微偏离中线时触发受限脱困，前进指令上限0.80 m/s、横向指令上限0.08 m/s，避免对齐降速与恢复条件互相锁死。
- **横穿等待**：基于同一份关节状态预测横穿障碍的运动窗口，机器狗在通道前最多1.6 m处等待，确认有通过空间后继续。等待请求在底层控制的地形辅助之后生效，避免被前进辅助覆盖；文件过期时保持等待。按0.18 m/s的保守通行速度预测；三处代理峰值速度分别为0.28、0.24、0.22 m/s，与场景配置一致。Nav2进度等待时限为40秒。此项为仿真状态驱动的辅助，并非Cosmos逐帧控制。
- **录屏与验收**：四个窗口置顶排成 2×2，避免被编辑器遮挡；忽略关闭辅助窗口瞬间的 `wmctrl BadWindow`。动态距离使用高频仿真状态，单独验收，不能由“导航到达终点”推断避障成功。

物理 XML 与 UE XML 现在有意不同：UE 只反转 `box_OBS15/16` 的 roll。编辑物理场景后必须重新生成 UE 版本，不能直接复制覆盖：

```bash
cd /home/unitree/Go2w+cosmos3Nav/matrix_go2w_lcm_demo
python3 scripts/sync_go2w_extended_ue_scene.py
./scripts/validate_go2w_extended_world.py
```

## 本次复测

完整复测已通过（18/18阶段，一次 NavigateThroughPoses、中途零重发）。行程约60.88 m；坡道机身高度从0.410 m升至0.854 m，再降至0.404 m，实际上升0.445 m、下降0.450 m。四处地形圆柱验收均通过；三处动态代理最近中心距离分别为1.621、1.713、1.688 m，均大于0.65 m。完整录像426.6秒，已全片解码检查，并逐段核对坡道方向、实际上下坡与障碍物可见性。

- [三类场景UE实录精选](artifacts/three_scenarios_verified_20260910_take3/three_scenarios_ue_highlights.mp4)
- [完整四窗口录像](artifacts/three_scenarios_verified_20260910_take3/matrix_go2w_cosmos_extended_navigation_15fps.mp4)
- [结果与验收指标](artifacts/three_scenarios_verified_20260910_take3/result.json)
- [三维里程计](artifacts/three_scenarios_verified_20260910_take3/odom_physics.csv)
- [动态显示专项录像](artifacts/scene_visual_repair_20260910/dynamic_fitted_visible.mp4)
- [坡道承重与上下坡专项录像](artifacts/scene_visual_repair_20260910/corrected_ramp_probe.mp4)
- [坡顶接缝碰撞扫描](artifacts/scene_visual_repair_20260910/collision_ray_audit_final.json)
- [测试日志](artifacts/scene_visual_repair_20260910/tests.log)

2026-09-03 的记录保留用于追溯路线和控制配置，不能作为修复后的视觉验收。9月10日早期完整运行虽18/18到达终点，但分别存在动态代理未显示或动态间距不足，已在对应目录的 `VISUAL_QA.json` 标记为不接受；它们不是本次最终交付。

## 点云与语义信息

需要同时观察楼梯点云、语义类别和同期相机画面，使用 [点云与语义采集说明](README_POINTCLOUD_SEMANTICS_CN.md) 中的 `scripts/record_go2w_semantic_demo.sh`。该入口沿用本页场景与导航控制，增加只读诊断与快照采集。

## 一键复现

以当前桌面用户运行：

```bash
cd /home/unitree/Go2w+cosmos3Nav/matrix_go2w_lcm_demo
OUT="$PWD/artifacts/three_scenarios_$(date +%Y%m%d_%H%M%S)"
MATRIX_UE_NO_RHI_THREAD=1 COSMOS_VLN_ARTIFACT_DIR="$OUT" \
  ./scripts/record_cosmos_extended_desktop_demo.sh
```

该入口启动 MuJoCo、UE、RoamerX、Cosmos 和点云融合器，录制 H.264 15 fps 四窗口视频。当前 4096×2304 桌面缩放输出为 2560×1440。仅运行导航：

```bash
MATRIX_UE_NO_RHI_THREAD=1 ./scripts/run_cosmos_extended_navigation_demo.sh
```

停止：

```bash
./scripts/stop_demo.sh
```

## 验收口径

`result.json` 中的 `result == "succeeded"` 表示路线到达，`acceptance_passed == true` 才表示各项数值验收全部通过。三类场景验收还须检查：18个阶段完成；`continuous_navigation.passed` 为 true（一次动作，中途零重发）；`physical_ramp_metrics.passed` 为 true（实际高度上升与下降）；四个 `terrain_avoidance_metrics` 均通过；`dynamic_avoidance_metrics.passed` 为 true（动态代理与机器人中心距离至少0.65 m）。再检查录像中坡面方向、实体上下坡、静态障碍及动态代理可见，且完整视频可解码。中心距离是近似包络指标，不代表逐足端接触力测量。

`/livox/lidar` 是实际传感器点云加场景几何模型的融合结果，动态位置来自仿真状态。这是可重复的仿真演示，不能据此宣称未知环境纯视觉感知或真机验证。动态代理不参与刚体碰撞，由导航点云提供避障约束。

## 关键文件

- 物理场景：`matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_yard_extended_nav.xml`
- UE生成场景：`matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/scene_terrain_yard_extended_nav.xml`
- 路线：`config/cosmos_vln_routes.json`
- 同步生成与静态校验：`scripts/sync_go2w_extended_ue_scene.py`、`scripts/validate_go2w_extended_world.py`
- 横穿等待：`scripts/go2w_dynamic_yield.py`、`controllers/go2w_rl_bridge/src/main.cpp`
- 实时状态副本：`scripts/go2w_state_tee.c`、`scripts/go2w_mujoco_state_relay.py`
- 点云融合：`scripts/go2w_world_obstacle_cloud.py`

扩展入口显式启用 `GO2W_MJ_STATE_RELAY=1`。状态副本只复制 MuJoCo 发往本机 UE 9999 端口的报文到本机25011端口；原报文原样发送，不改写控制数据。观测端状态缺失或过期时停止输出融合点云并报告异常。其他场景未显式启用时沿用原入口行为。
