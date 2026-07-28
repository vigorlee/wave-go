# 节点一：Go2-W 纯世界模型盲探索

记录时间：2026-07-28 19:01 CST
状态：**暂停，Attempt 19 达到安全 blocked 终态；换锁修复待现场复测**
工作分支：`fix/adaptive-hazard-seed-budget`

## 目标

从 HouseWorld 右上角启动 Go2-W，在 `online_slam`、`nav2=disabled` 条件下，仅由 Cosmos3 Generator 产生运动；发现 ArUco ID 560 候选后交给 Reasoner，以三帧 RGB-D 完成近距离确认，停车、下蹲并切换充电模态。

## 已完成

- Attempt 14 至 Attempt 19 均已归档；Attempt 19 达到明确 blocked 终态后，HouseWorld 已安全停止。
- Attempt 14 审计：120 个 Generator 请求，63 ready、56 veto、1 rejected；未发现 ID 560。
- Generator 批量接口离线实测：输出为 `2 x 16 x 9`，`batch=2` 稳态约 198 ms，seed 4/5 串行约 269 ms；同 seed 的批内 chunk 完全相同，混合 seed 请求返回 HTTP 400。因此 batch 不能提供独立多 seed 候选，未接入闭环。
- 自适应 seed 预算已在 Attempt 16/17 现场运行：危险区先评估两个有效候选，仅在没有安全共识时扩展到全部 8 个 seed；最近成功 seed 优先。
- Attempt 16：57 次推理启动，18 ready、32 veto、6 rejected；最终请求在取消时丢弃，终态 `canceled`。
- Attempt 17：86 次推理启动，16 ready、63 veto、6 rejected；最后量测约 `front=0.998 m`、`left=1.052 m`、`right=0.464 m`，命令为 `(vx,wz)=(0,0)`，终态 `canceled`。没有 marker、Reasoner 或 target-confirmed 事件。
- Adapter 接入 Go2-W 后，Generator 原始 action chunk 为 `16 x 9D`，自适应预测时域为 `H in {4,8,16}`，底盘输出为 `H x 2`，单帧是 `[linear_x, angular_z]`。下蹲和模态切换不属于 Adapter 输出，由安全状态机在终止门控通过后触发。
- 当前 `wave-go` 代码已加入危险区纯 yaw 候选、hazard phase 滞回保持、推理前后 phase 变化时丢弃旧候选、进入 hazard 时立即停车并丢弃非 hazard prefix，以及最终候选入口的共识安全检查。
- 公开仓库全套测试为 `126/126`；Generator action 专项测试为 `wave-go 61/61`、完整工程 `59/59`，两份主搜索脚本通过 `py_compile`。
- Attempt 18：59 次推理启动、20 ready、38 veto、10 次 prefix 提前停止；13 个 yaw-only 候选通过 ready，request 3/10/11/30/52 完成四帧非零 yaw-only 执行。首次墙前请求记录 `initial_valid_candidate_budget=8`，方向锁由 Generator 共识建立；执行命令严格满足 `linear_x=0`，实测里程计 yaw 从 `-90 deg` 改变到约 `-74.9 deg`，且前距没有因墙前转向继续前移跌入停止线。
- Attempt 18 最后量测约 `front=2.940 m`、`left=1.125 m`、`right=1.432 m`，命令为 `(vx,wz)=(0,0)`。`cosmos-nav-collector-20260728-182100.service` 在 request 59 推理中发送 cancel 并停止同名 HouseWorld 服务，因此终态 `canceled` 是外部中断，不是搜索算法的成功、blocked 或超时终态。
- 启动器已补上 runtime systemd mask 的自动解除，并让 Cosmos Generator 以 `UV_OFFLINE=1` 使用已验证的本地 VAE/CLI 缓存，避免 PyPI TLS 抖动导致冷启动失败。
- Attempt 19：24 次推理启动、14 ready、7 veto、47 个执行帧；request 11 由 seed 5 建立 `+1` 方向锁。终态为 `blocked/front_hazard_direction_unavailable`，连续 veto 计数为 8；最后量测 `front=3.023 m`、`left=1.193 m`、`right=1.300 m`，底盘双零速，没有 marker 或 Reasoner 事件。截图和日志均确认状态一致。
- Attempt 19 的 request 24 已生成可执行反向共识：seed 6 为完整四帧 `(vx,wz)=(0,-0.477)`，但被旧 `+1` 锁拒绝。代码现改为仅在“锁定侧低于 `1.20 m`、另一侧仍安全、连续 8 次无同向共识”时，在同一次全 seed 评估内释放旧锁并由该安全 Generator 共识重新建锁；不会短暂退出危险区，也不会让 LiDAR 指定新方向。
- Attempt 20 启动请求被新增的数据采集互斥锁立即拒绝，没有启动进程或占用新端口，证明并发保护生效。当前 `cosmos-nav-collector-20260728-182100.service` 正在运行自己的 MuJoCo/UE/Zenoh 栈。

## 关键结论

1. 性能瓶颈不是单次 Generator 推理，而是危险区无安全候选时串行扩展全部 seed；自适应预算能缩短常见成功轮次，但在持续 veto 时仍会回到全预算。
2. 批量接口只有同 seed 的重复样本，不能替代独立 seed 共识。
3. Attempt 18 已现场验证纯 yaw/hazard-phase 修复：开放区 prefix 在进入墙前阶段时立即停止，墙前执行保持 `vx=0`，Go2-W 实际 yaw 能响应。
4. 当前主要运行阻塞是并发数据采集服务与盲探索共用 `go2w-house-navigation.service`、ROS domain、端口和 `.run/go2w_house`。在采集服务退出前不能再启动盲探索，否则会互相 cancel/stop 并覆盖运行证据。
5. 墙前侧向净空在 `1.20 m` 门槛附近波动时，8-seed 会频繁扩展并产生 veto；这不是单次 Generator 延迟问题，是安全候选接受率导致的有效吞吐下降。
6. Attempt 19 证明固定方向锁在锁定侧失去净空后会错误拒绝已存在的安全反向 Generator 共识；原子换锁修复保留 Generator-only 选向原则，同时避免这一确定性 blocked。

## 未完成项

- 等 `cosmos-nav-collector-20260728-182100.service` 完成或经用户确认停止后，从固定出生点重跑 Attempt 20，现场核验 `search_arc_direction_released -> search_arc_direction_locked` 在同一 request 内出现并执行反向 yaw-only。
- 完整工程启动器包含数据采集互斥锁，公开仓库不复制该工程专用实现；共享的搜索主脚本、配置和 Adapter 合同保持一致。
- 从安全重置位继续搜索并发现 ID 560；验证候选处的 Reasoner 调用。
- 完成三帧 RGB-D 近距确认、停车、下蹲和模态切换。
- 统一 RViz、UE 和 Cosmos 标注相机的 ROS domain/时间源并形成最终证据。
- 完成最终中文报告、README、代码同步、提交与推送。`raw/` 大日志和用户已有 `figures/` 不纳入提交。

## 归档位置

- `world-model-blind-attempt-14/`
- `world-model-blind-attempt-15/`
- `world-model-blind-attempt-16/`
- `world-model-blind-attempt-17/`
- `world-model-blind-attempt-18/`
- `world-model-blind-attempt-19/`

Attempt 18/19 原始日志、配置、runtime 快照和最后一帧可视化已归档。恢复节点一时，必须先确认数据采集服务 inactive，再执行任何仿真操作。
