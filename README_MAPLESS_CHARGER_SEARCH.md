# HouseWorld 无图充电桩世界模型测试

这份说明对应当前实际运行的 HouseWorld 无图搜索链。目标是让 Go2-W 从通道侧出生点出发，只用实时 RGB、RGB-D、LiDAR 和 NWM-Cosmos3Edge 搜索带二维码的机器人充电桩；搜索过程中不使用已知地图、地图目标、`/navigate_to_pose`、航点或充电桩坐标。Cosmos3 Generator 是搜索、接近和重捕获阶段唯一允许产生非零运动的 nominal action source，LiDAR/RGB-D/姿态层只能限幅或清零，不能补写传统 planner 动作。

当前 checkpoint 没有 Go2-W action domain 或 Go2-W 微调权重。本链使用基础 Cosmos3-Edge 的 `av` 9D 相机相对位姿输出，经 `experimental_av_relative_pose` 零样本跨 embodiment 适配器转换为 Go2-W `Twist`。因此它是 Generator 闭环仿真试验，不应表述为已经训练完成的 Go2-W 专用策略。

## 当前配置

- 场景：HouseWorld，scene ID `6`
- ROS domain：`90`
- 默认地图源：`online_slam`（空地图启动，`/scan -> slam_toolbox -> /map`）
- 出生点：`(x=1.1, y=-7.2, yaw=90 deg)`，位于右侧通道一侧
- 出生点模型净空：约 `1.08 m`
- 相机：`/image_raw/compressed`
- 深度：`/image_raw/compressed/depth`，`32FC1`，配置尺寸 `320x240`，单位为米
- LiDAR：`/livox/lidar`
- 速度命令：`/cmd_vel_nav`
- 视觉标记：OpenCV ArUco `DICT_4X4_1000`，ID `560`
- NWM-Cosmos3Edge 目标类型：`robot_charging_dock`
- Generator 服务：`http://127.0.0.1:8098`，`av / 9D / 16 steps / 10 Hz / 256 px`
- Generator 执行窗：搜索按净空动态执行 `16 / 12 / 8` 步，接近最多执行 `8` 步，每步重新检查视觉、RGB-D、LiDAR 和姿态；命令 TTL `0.20 s`
- Generator 候选：搜索请求在 `[0,2,3,5]` 中每轮旋转一个 seed，接近/重捕获比较前两个 seed
- Generator 速度上限：搜索 `0.90 m/s`、远距接近 `0.50 m/s`、偏航 `0.50 rad/s`
- Reasoner 输出预算：`1024` tokens；提示要求先输出完整六字段 JSON，缺字段、截断或附加字段仍严格拒绝

出生点由 [start_go2w_house_navigation.py](scripts/start_go2w_house_navigation.py) 设置。无图搜索本身的参数在 [go2w_house_mapless_search.json](config/go2w_house_mapless_search.json)；默认 `online_slam` 环境不会注入固定路线目录。

## 脚本职责

| 文件 | 职责 |
| --- | --- |
| `scripts/start_go2w_house_navigation.py` | 启动/停止 systemd HouseWorld 服务、MuJoCo/UE、RL bridge、LiDAR、NWM-Cosmos3Edge、RViz 和无图搜索节点 |
| `scripts/cosmos3_go2w_generator_server.py` | 常驻加载一次 Cosmos3-Edge，同时提供 `/predict_batch` Generator 和共享 `/reason` 语义确认接口 |
| `scripts/go2w_house_pointcloud_to_laserscan.py` | 将实时 `/livox/lidar` 投影为 `/scan`，供在线 SLAM 使用 |
| `config/go2w_house_slam_toolbox.yaml` | 在线 async mapping、`map -> odom` 和地图更新参数 |
| `scripts/go2w_house_mapless_charger_search.py` | Generator 请求/动作适配、ArUco 检测、NWM-Cosmos3Edge 语义复核、veto-only shield、RGB-D 停车/充电门控 |
| `config/go2w_house_mapless_search.json` | Generator 合同、无图传感器、速度上限、墙距、二维码和姿态保护阈值 |
| `controllers/go2w_rl_bridge/src/main.cpp` | DreamWaQ 底层运动；零速度持续策略平衡、`charge` 下蹲和带姿态反馈的 `recover` |
| `scripts/cosmos_vln_visualizer.py` | 生成实时相机和状态可视化 |
| `tests/test_go2w_house_mapless_search.py` | 无图搜索、二维码、近距门控、姿态保护和 recover 合同测试 |
| `tests/test_go2w_house_generator_action.py` | Generator 9D 动作、坐标适配、TTL、漂移拒绝和 safety shield 合同测试 |

旧的固定观察点实现 [go2w_house_charger_search.py](scripts/go2w_house_charger_search.py) 和 [go2w_house_charger_search.json](config/go2w_house_charger_search.json) 保留作历史参考，但当前服务不会加载它们。

## 启动

```bash
cd /home/unitree/matrix_go2w_lcm_demo
/usr/bin/python3 scripts/start_go2w_house_navigation.py start \
  --onscreen --with-cosmos --rviz --start-timeout 240
```

检查服务和出生点：

```bash
/usr/bin/python3 scripts/start_go2w_house_navigation.py status
```

`runtime.json` 应显示 `"map_source": "online_slam"`，进程列表应包含
`cosmos_generator`、`pointcloud_to_laserscan`、`slam_toolbox` 和 `vel_cmd_lcm`，不应包含 `navigation`、
`cosmos_bridge` 或 `cosmos_mission`。在线 `/map` 只用于观察和记录，无图搜索节点仍不
订阅地图；运行环境中也不存在 `COSMOS_VLN_ROUTES_FILE`。

确认 `/odom/mujoco_odom` 的位置约为 `x=1.1, y=-7.2`，高度应保持在 `0.35 m` 以上，且机器人速度为零后再发布任务。

## 发布世界模型测试

先加载 ROS 2 和 Zenoh 环境：

```bash
set +u
source /opt/ros/humble/setup.bash
source genisom_roamerx_open/install/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=90
export ROS2CLI_NO_DAEMON=1
```

发布无图搜索任务：

```bash
ros2 topic pub --once /cosmos_vln/charger_search std_msgs/msg/String \
  "{data: 'Search maplessly for the QR-marked robot charging dock in the unknown environment. Build the map online only for visualization, keep clear of walls, use NWM-Cosmos3Edge to verify ID 560, approach until RGB-D confirms the dock is within 0.40 meters, stop completely at the dock, and only then crouch to charge.'}"
```

搜索状态：

```bash
ros2 topic echo --full-length /cosmos_vln/charger_search_status
```

取消搜索/接近并立即停轮。若任务已经进入 `charge`，取消消息不会自动把姿态改回 `stand`：

```bash
ros2 topic pub --once /cosmos_vln/cancel std_msgs/msg/Empty '{}'
```

停止整个仿真链：

```bash
/usr/bin/python3 scripts/start_go2w_house_navigation.py stop
```

## 状态流和严格门控

正常状态流是：

```text
ready
  -> started
  -> searching
  -> generator_inference_started
  -> generator_action_ready
  -> generator_action_executing
  -> marker_candidate
  -> cosmos_settle_started
  -> cosmos_settle_ready
  -> cosmos_verification_started
  -> cosmos_verification
  -> target_confirmed
  -> approaching / approaching_tracked / approaching_hold / reacquiring
  -> close_marker_confirmation
  -> arrived_stopped
  -> charging
  -> succeeded
```

共享 Edge Reasoner 只能确认“这是带 ID 560 的机器人充电桩”，不能直接触发下蹲。进入 `charging` 前必须同时满足：

- 从 NWM-Cosmos3Edge 返回后重新采样的稳定接近起点累计路径长度至少 `1.20 m`，推理期间漂移不计入；
- 每个最终确认帧都重新检测到 ID `560`；
- 连续 `3` 组新的 RGB 与 Depth 帧满足最终门控，接收时间差不超过 `0.60 s`；
- 二维码高度占图像至少 `0.10`，水平误差不超过 `0.06`；
- 二维码内部收缩多边形的实时深度为 `0.20-0.40 m`；
- 实时 LiDAR 前方最近障碍净空至少 `0.42 m`；
- 轮速清零并保持停车约 `3.0 s`，停车期间精确二维码和深度仍需持续确认；
- 里程计机身高度不低于 `0.35 m`，roll/pitch 绝对值不超过 `0.30 rad`。

远距离二维码检测同时尝试原图和地面区域的 `2x/4x` 多尺度图像，并针对小轮廓和 JPEG 压缩调整 ArUco 参数。近距离码位于暗色充电底座时，还会尝试 `40/50/60` 的保守全局阈值；结果仍必须精确匹配 `DICT_4X4_1000` 的 ID `560`。首次精确解码 ID `560` 且 NWM-Cosmos3Edge 确认充电桩身份后，CSRT 视觉跟踪器可跨过远距离小码的模糊帧继续提供对准观测，状态为 `approaching_tracked`。近距离黑色码边与底座粘连、强透视导致 OpenCV 拒绝候选时，搜索器只在这个已确认 tracker 区域内提取候选四边形并透视拉正，使用每格中心中值重新读取 6x6 ArUco 单元；只有存在单一阈值可让 20 个黑边格和 16 个编码格以零比特误差匹配 ID `560`，且黑白间隔至少 18 灰度级时，才返回 `exact_id=true, verification=target_cell_exact`。这里不允许汉明纠错，错误 ID 和单比特损坏都会拒绝。跟踪观测本身仍固定为 `exact_id=false`，不能提供最终深度门控、累计最终确认帧或触发下蹲；最终停车的每个新帧仍需通过 OpenCV 字典解码或上述零误差目标格解码。

RGB 与 Depth 使用相同外参和 `90 deg` 水平视场，但分辨率/纵横比不同。搜索节点按相机射线模型把 RGB 中的四个 ArUco 角点投影到 `320x240` Depth，向中心收缩 `20%` 后过滤无效值并提取最近主深度簇。当前 Depth 发布器会把 `width/height/step` 填成 `0`，节点因此严格校验 `32FC1` 和 `320*240*4` 字节 payload 后再按配置尺寸解析；格式、长度、有效像素数、有效率或同步时间任一不合格都会 fail closed，不会回退到二维码尺寸、CSRT 或 LiDAR 后墙距离。

CSRT 跟踪结果最多在最近一次精确 ID `560` 后使用 `4.0 s`，不会无限续命。接近 chunk 内每一步都会先尝试精确解码，再使用这个短时 tracker；chunk 内恢复的精确 ID、RGB-D 深度和时间戳会原子回传外层观测状态。搜索、接近、短时 hold 和重捕获都把实时图像及传感器上下文送进 Generator；旧的 `select_exploration_heading -> reactive_velocity` 和阻尼视觉伺服不在运行链中。`approach_hold` 和 `reacquire` 的 safety shield 强制平移为零；重新识别目标后才允许 Generator 的正向分量通过。最多等待 `20 s`，二维码仍未恢复会停止并报告 `blocked`。RGB、里程计、LiDAR 或 Depth 过期、推理期间位姿漂移超过 `0.05 m / 0.05 rad`、输出维数错误或出现 NaN/Inf 都会停轮。连续三次 Generator 失败后任务以 `cosmos3_generator_unavailable` 阻塞，绝不回退传统 planner。

搜索阶段第一次检测到 ID `560` 后，候选状态最多保持 `6.0 s`，覆盖远距离小二维码的短时漏检，期间不会恢复 Generator 搜索动作。候选停车只衰减当前 Generator 命令；若前方进入避障距离立即清零。当前 LiDAR 没有后向净空扇区，因此 shield 禁止倒车，也不会自行生成转向。调用共享 Edge Reasoner 前，里程计线速度必须不超过 `0.03 m/s`、角速度不超过 `0.06 rad/s`，且站立姿态连续稳定 `1.0 s`；`6.0 s` 内无法稳定会报告 `cosmos_precheck_not_stable`。

接近 Generator 命令的远距上限为 `0.50 m/s`：当前帧精确 ID/RGB-D 深度大于 `1.50 m` 时允许使用该上限，`0.80-1.50 m` 限为 `0.25 m/s`，`0.40-0.80 m` 限为 `0.12 m/s`，进入 `0.40 m` 即禁止前进。只有 tracker 而没有当前帧精确深度时也限为 `0.12 m/s`；水平误差超过最终容差两倍时最多 `0.25 m/s`，超过 `0.32` 时禁止前进。远场闭环标定确认：图像左侧目标需要正 `/cmd_vel_nav.angular.z`，即目标偏差与 ROS 偏航命令反号；LCM 和 DreamWaQ bridge 保持标准 ROS 偏航符号，不做额外反转。接近阶段最长 `240 s`。推理开始时旧 chunk 会被清空并发布零速；响应到达后重新采样传感器，搜索根据净空执行最多 `16` 个 `0.1 s` 预测步，接近最多执行 `8` 步。每一步都重新采样安全观测，ROS 发布 TTL 最多 `0.20 s`；任何 veto 都可缩短该 prefix。最终到达仍只由实时 RGB-D、精确 ID、里程计和安全门控决定。

HouseWorld 启动器设置 `GO2W_RL_IDLE_STAND=0`，所以普通零速度命令仍由 DreamWaQ 使用重力和陀螺反馈保持平衡，不会从动态策略硬切到固定关节站姿。异常姿态触发的 `recover` 同样使用零速度策略反馈；只有经过近距停车门控后写入 `charge` 才会执行下蹲关节轨迹。

搜索 prompt 要求保持通道中心和约 `1.20 m` 墙距。前方不足 `0.68 m` 时搜索平移被 veto；接近阶段的紧急门槛为 `0.42 m`。转向靠近过近侧墙会被 veto，shield 不产生弧线、倒车或替代探索方向。

## 验收日志

2026-07-25 的完整闭环验收从 `started` 到 `succeeded` 用时 `297.89 s`，其中无图搜索到
`target_confirmed` 为 `235.25 s`，从确认到 `arrived_stopped` 为 `58.77 s`。常驻 Generator
除首次编译/预热 `3.88 s` 外，120 次热请求平均 `0.195 s`；搜索实际执行了
`8/12/16` 步动态窗口，接近执行 `8` 步窗口，执行命令峰值 `0.90 m/s`。最终累计接近
`2.807 m`，连续三帧以 `target_cell_exact` 零误码确认 ID `560`，RGB-D 深度
`0.357 m`，停车复核约 `3 s` 后进入 `charging` 并报告 `succeeded`。

一次完整成功测试应能在 `charger_search.log` 中看到：

```text
state=generator_inference_started ... action_source=cosmos3_generator
state=generator_action_ready ... nominal_linear_x ... nominal_angular_z ... shield
state=generator_action_executing ... request_id ... chunk_step ... prefix_steps ... command_ttl_sec=0.20
state=marker_candidate ... marker_id=560
state=cosmos_settle_ready ... linear_speed<=0.03 ... yaw_rate<=0.06
state=cosmos_verification ... target_kind=robot_charging_dock ... confidence>=0.70
state=target_confirmed
state=approaching / approaching_tracked / approaching_hold / reacquiring ... approach_travel_m ... marker_depth_m ... front_m
state=close_marker_confirmation ... approach_travel_m>=1.20 ... marker_depth_m<=0.40 ... front_m>=0.42
state=arrived_stopped
state=charging
state=succeeded
```

以下任一情况都不是成功：

- 只有 `target_confirmed`，没有 `arrived_stopped`；
- 远处二维码检测后直接出现 `charging`；
- `approach_travel_m` 小于 `1.20 m`；
- `marker_depth_m` 缺失、不在 `0.20-0.40 m` 或来自重复/不同步 Depth 帧；
- `front_m` 小于 `0.42 m`；
- 机器人仍在移动或姿态已下沉；
- 只有旧图像中的二维码，没有新的连续确认帧。

`approach_travel_m` 是累计路径长度，`marker_depth_m` 是二维码内部区域的 RGB-D 实测距离，`front_m` 只表示 LiDAR 碰撞净空。HouseWorld 的白色充电桩没有进入 MuJoCo 碰撞点云，因此不能再把后墙 LiDAR 距离当作桩距。最终验收仍必须查看停车阶段的新相机帧，确认 ID `560` 清晰可见、画面尺寸达到门槛，并且机器人与二维码充电桩的实际相对位置正确。

关键文件：

- `.run/go2w_house/cosmos/latest_visualization.jpg`：相机、二维码框和当前状态
- `logs/go2w_house/charger_search.log`：无图搜索状态和门控数值
- `logs/go2w_house/bridge.log`：底层姿态、速度命令和 `recover` 状态
- `.run/go2w_house/cosmos/charger_search/`：NWM-Cosmos3Edge 输入、输出和推理记录
- `.run/go2w_house/posture`：当前 `stand`、`charge` 或 `recover`

## 测试命令

纯 Python 合同测试：

```bash
cd /home/unitree/matrix_go2w_lcm_demo
/usr/bin/python3 -m unittest \
  tests.test_go2w_house_generator_action \
  tests.test_go2w_house_online_slam \
  tests.test_go2w_house_mapless_search \
  tests.test_go2w_house_runtime \
  tests.test_go2w_house_navigation_strategy
```

完整测试需要先加载 ROS 环境：

```bash
set +u
source /opt/ros/humble/setup.bash
source genisom_roamerx_open/install/setup.bash
set -u
/usr/bin/python3 -m unittest discover -s tests -p 'test_*.py'
```

## 运行边界

HouseWorld 仿真资源本身是固定测试场景，但默认运行时不向搜索器或 SLAM 提供先验地图：
`/map` 从空白开始由实时 LiDAR 建立，运动控制也不消费该地图。因此这是“运行时未知
环境”的无图搜索与在线建图测试，不等同于真实机器人安全证明。不要同时运行旧的固定
路线充电节点，也不要用 `--known-map-nav`、人工目标或预生成地图替代本测试的无图搜索。

## 代码备份

本次切换 Generator 前的代码备份：

```text
.backups/pre_cosmos_generator_20260725T022841.tar.gz
SHA-256 e701fdbdc5fd64528a33b916c21cb99519a0c968e89be6f7a58115059a8bb268
```
