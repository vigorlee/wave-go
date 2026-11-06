# OfficeWorld 楼梯入口局部避障修复（2026-10-12）

原始运行 `office_auto_20261012_201413` 在第 8/15 阶段反复产生
`pedestrian_yield` / `pedestrian_clear`。入口转向存在实际平移，而旧控制仅停车等候；
极短的放行窗口不足以完成转向。原运行最终因 odometry 变旧退出，不是正常到达。

本次只修改路线跟踪与平地局部恢复，不修改 LIO、点云颜色、楼梯几何或普通前进的
行人避让距离。当前距离仍为候选半径 + 0.95 m，预测距离为半径 + 0.85 m，
解除等待的滞回距离仍为 0.10 m。

## 修改

- 西侧转向目标从 `(-5.90, 3.35)` 移到 `(-6.20, 4.15)`，留出实际转向平移的空间。
- 向北接近时达到远侧安全转向区域即开始对准楼梯，避免围绕很小的坐标目标打圈。
- 仅楼梯入口阶段使用已有的 `flat` 控制参数。旧 `avoid` 参数把后退截为
  -0.15 m/s、转向截为 0.6 rad/s；入口平地转向指令最高 0.9 rad/s，
  最后楼梯前的精确对准保持原限制。
- OfficeWorld 默认 `GO2W_RL_IDLE_STAND=0`，使用策略零速度平衡。原静态
  PD 站立在停步后出现约 -0.44 rad 俯仰和后移，反复打断入口校准。
  其他场景的默认值不变；Office 可显式设置 `GO2W_RL_IDLE_STAND=1` 回到原方式。
- 入口在 y ≤ 2.015 m 才开始站稳确认，为实际停步位移预留余量。
  显式 `--start-stage 7` 续测仅能在实测位置、航向、姿态均已对准时接入，
  仍需新的站稳观测，不凭坐标推断任务完成。
- 等待超过 3 秒时可以尝试短距离后退。每次以新鲜、同时间的实测 LiDAR、
  物理场景碰撞几何及行人预测检查短轨迹；行人间距必须增长，不能向行人穿越。
  未知或过期输入、楼梯/坡面、后方障碍均禁止恢复。
- 后退指令最多 -0.30 m/s，实际总位移不超过 0.30 m，尝试不超过 6 秒。
  起步阶段反向位移超过 7.5 cm 立即停；起步 1 秒后反向位移超过 3.5 cm 停。
  实际动作与规划不符时停车，不将指令当作执行结果。
- RViz 状态显示 `REVERSE ESCAPE - checked clearance`，运动记录包含 `escaping`。
  解锁仅允许已检查的恢复动作，原前进路线仍处于等待状态。

后退可用区域扩展到入口北侧实际平地（y ≤ 4.2 m）。这不是删墙或改变地图：
沿 `(-5.72,3.61)` 到 `(-6.10,4.00)` 的独立物理几何检查显示地面高度为 0，
最近静态障碍距离至少 1.109 m；轨迹仍逐次接受实时点云与几何碰撞检查。

## 测试范围

86 项代码、几何与安全回归检查通过，包含后方未知障碍、接近行人、过期数据、
反方向实测动作、恢复预算、预览地面支撑和真实测量才能切换楼梯策略。
这些检查不等于完整实景导航通过。

保留所有失败与续测数据。`203223` 为从问题位置启动的诊断，不是完整 15 阶段。
`204529` 在第一次完整试跑中仍遇到转向问题；其 `entry_recovery_retry_*`
为同场景续测，不与前 7 阶段拼接宣称一次全程成功。

`205716` 与其 `entry_long_approach_*`、`aligned_entry_continuation_*` 均是调试证据，
不作为一次完整成功。

**最终版完整实测通过：** `artifacts/office_auto_20261012_210928`，从起点连续
完成 15/15，耗时 376.301 秒。通过行人区域与两段原生楼梯，二楼门前目标距离
0.03667 m、航向误差 4.842°。完成事件顺序严格为 0–14，无任务中位姿重置或
将续测拼接成全程结果。该轮等待 12 次，入口已正常恢复前进。新增后退恢复在
本轮完整成功中没有触发，因此该结果主要验证新的转向区域、控制参数与站稳策略，
不将它写成所有恢复动作均已实景验收。

依据：本轮 `course/result.json`、`course/events.jsonl` 与
`local_escape_navigation_validation.json`。

四窗录像 `recordings/office_four_windows_20261012_211006_070.mp4` 为
1920×1080、12 fps、5086 帧、423.834 秒，已完成全片解码检查。
四个窗口保留，21:17:10 已在当前门前交接 WASD。

导航成功不表示房号核验或 SLAM 精度通过：本轮门牌身份仍未确认，
本次未验证原生 LIO 地图精度，也未将几何候选称为神经网络确认的人类标签。
原始代码备份、检查结果与来源记录在
`artifacts/local_escape_fix_20261012_202317`。

## 运行

```bash
cd /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916
unset OFFICE_PREPARE_GATE OFFICE_START_STAGE INLINE_START_X INLINE_START_Y INLINE_START_YAW
OFFICE_LIO_EXECUTABLE="$PWD/.run/go2w/native_lio_build/mapping" \
OFFICE_RECORD=1 bash scripts/start_office_global_auto.sh SmacAStar
```

截图：`/usr/bin/python3 scripts/capture_office_four_windows.py`。
接管当前机器狗：`bash scripts/open_office_manual_here.sh`；这会停止自动导航。
