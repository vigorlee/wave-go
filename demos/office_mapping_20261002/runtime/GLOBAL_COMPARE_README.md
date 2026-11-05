# OfficeWorld：两种开源全局规划与动态行人

**当前入口：自动穿越楼梯、斜坡和动态行人，上二楼绕过桌椅，到玻璃门 `Sm_Door02` 前停稳，再打开 WASD。**

**2026-10-02 门牌关联更新：** 辅助 RGB 为实际 3840×2160 / 1 Hz，配对 640×360 / 1 Hz 米制深度，OCR 默认提交间隔 2 秒。当前木门房号 **01.05** 已通过跨源帧识读、同源门掩膜与三维关联，RViz 显示 `ROOM 01.05`、`TARGET 01.05 / DOOR MATCHED | LIVE`、停靠点及关联线。门牌与门面位置来自深度和 LIO；既有任务目标的显示使用明确标注的源时刻仿真参考/LIO 局部配准，不改写点云。查看[运行说明与真实验证](docs/DOOR_RECOGNITION_20261002.md)。

场景延续已验证的 Go2-W 大楼梯、斜坡、圆柱和原生二楼。新增两种真正调用的 Nav2 全局规划器、两名 MATRiX 原生行人，并在四分屏中同时显示运动、实时三维建图、视觉模型和 UE 场景。

## 两种路线

| RViz 颜色 | 上游插件 | 算法 | 当前 15 阶段缓存长度 |
|---|---|---|---|
| 蓝色 | `nav2_navfn_planner/NavfnPlanner` | Dijkstra，`use_astar: false` | 48.775 m |
| 紫色 | `nav2_smac_planner/SmacPlanner2D` | 代价感知 A*，带路径平滑 | 48.409 m |

安装版本均为 Nav2 Humble `1.1.20`。历史 12 阶段 `ComputePathToPose` 动作结果保存在 `artifacts/global_compare_20261001/plans/routes.json`。当前 15 阶段缓存位于 `config/global_compare/door_routes.json`，保留每段求解时间和未经修改的二维坐标；上表是读取该缓存后的静态路线长度，不是机器人实测行驶距离。自动入口检查场景、任务与规划配置的散列；首次或内容变化时重新调用两种插件求解，并把当前结果保存到本轮 `plans/routes.json`。路径跟随器直接读取保存的 Nav2 求解坐标，保留既有 RL 爬楼与坡道控制。坡道前瞻距离为 1.0 m，楼梯 0.65 m，平地 0.45 m；增大坡道前瞻用于避免入口处反复请求原地转向。动态障碍物处理由局部 LiDAR 聚类、预测与停车等待完成；本演示没有把它描述为 Nav2 在线动态全局重规划。

这是**根据现有场景几何构建分层二维可通行区域，再由 Nav2 求解每一层路线**的演示。原路线 12 个地形转换检查点；2026-10-02 门前版本增加 3 个二楼检查点，共 15 个阶段。它们约束楼梯和楼层切换；二维插件本身不负责三维攀爬，也不是无图探索。RViz 中的路径高度来自地形表面，用于展示；建图点云来自实际 LiDAR 和 IMU 的 LIO 估计。

**2026-10-02 完整实测：** `office_auto_20261002_184713` 的 Smac A* 从起点连续完成 **15/15**，耗时 **261.40 秒**，最后正对玻璃门停车并交接 WASD。15 个不同阶段的完成事件齐全，停车位置距目标约 **0.194 m**，航向误差约 **4.22°**。本轮事实记录见[导航校验](artifacts/office_auto_20261002_184713/readonly_navigation_validation_20261002_185300.json)。文末的两算法历史成功记录属于较短的 12 阶段路线；本轮 Dijkstra 的全程调试完成 14/15，门前修正后另做了单阶段续测，不当成一次完整 15 阶段成功。

本轮可查看[四窗口连续实录](artifacts/office_auto_20261002_184713/recordings/office_four_windows_20261002_184751_020_recovered.mp4)、[门前四窗口 PNG](artifacts/office_auto_20261002_184713/screenshots/office_four_windows_20261002_190237_461.png)和[实际在线门检测证据](artifacts/office_auto_20261002_184713/door_evidence/full15_success_online_models_20261002_185217.png)。视频为 1920×1080、12 fps、3157 个真实帧，263.084 秒，已全片解码检查。原录制在停止时缺失 MP4 索引，因此从原文件完整 H.264 数据恢复画面，以已知 12 fps 重建媒体时间轴；不插入画面，排除末尾一个不完整数据包，原文件保留。恢复后的媒体时间不能代替传感器或任务日志时间，详见[恢复记录](artifacts/office_auto_20261002_184713/recordings/recovery/recovery_report.json)。新的停止流程已用短片实录和重复停止信号验证正常封装。

手动交接后的[只读验证](artifacts/office_auto_20261002_184713/manual_handoff_validation.json)记录 601 个新里程计样本与 60 个全零运动指令；未发送额外测试运动。用户可在当前门前直接点击 WASD 面板接管。

参考上游源码：[NavFn](https://github.com/ros-navigation/navigation2/blob/humble/nav2_navfn_planner/src/navfn_planner.cpp)、[Smac 2D](https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/src/smac_planner_2d.cpp)。

## 两名动态行人

配置：`matrix/scene/scene_go2w_global_compare.json`。保留 60 个既有静态物体，增加两个 `type: dynamic`、`model: human1`、`avoid: false` 的人物。人物分别在斜坡后的平地与原生楼梯入口附近往返行走，轨迹端点与机器狗通道留有间隔。`avoid: false` 使避让由机器人侧承担。

参数参考 `/home/unitree/下载/MATRiX公开课.pdf` 物理第 17 页（页面印刷编号 16/42）和 `matrix/docs/Custom_Scene_Tutorial_CN.md`。UE 位置单位为厘米，场景转换为 `UE=(100*x-760, -100*y+2420, 100*z)`。两人的 JSON `velocity` 为 0.5、0.4，轨迹端点使用实际站立角色中心高度 `104 cm`，避免归一化运动方向含较大的向下分量。轨迹起点分别为物理坐标 `(-4.1,-3.0)`、`(-4.4,4.2)`。

当前打包 Blueprint 使用归一化运动输入，默认 `MaxWalkSpeed=500 cm/s`。启动脚本将其设为 `100 cm/s`，使上述输入对应正常步行速度；航点到达阈值为 `15 cm`，避免在端点附近抖动。原生角色完成轨迹会销毁，因此显式配置足够长的往返轨迹，不依赖未公开的循环字段。启动参数直接执行 UE 场景加载和角色设置，不依赖键盘焦点。启动检查读取真实 UE 日志中的人体网格与初始位置，确认加载两人，保存到 `pedestrian_validation.json`；该启动检查不证明运动，后续移动需由实时 LiDAR 跟踪和运行画面验证。

机器人侧使用当前 MID360 点云进行静态表面扣除和动态区域聚类，支持只扫到腿部的情况、短时遮挡保持、速度预测与停车等待。RViz 的红色标记是**几何障碍物候选簇**；不能把它们当成已经测过准确率的神经网络语义标签。左下视觉面板继续展示 Depth Anything V2、Grounding DINO 和 SAM2 的实际推理结果。

视觉模型增加 `door` 提示词与 **class 8** 门标签，由 Grounding DINO 检测和 SAM2 掩膜产生，模型置信度与原始标签保存在视觉 `frames.jsonl`。画面使用可读英文类别，中文类别保留在元数据；门标签不是直接把 UE 网格名字写成识别结果。语义点的米制坐标来自对齐的 UE 深度传感器，Depth Anything V2 只提供相对深度显示。

门牌文字另由 `scripts/go2w_sign_ocr.py` 的本地 **Tesseract English LSTM** 读取，独立低频异步线程处理真实 RGB 图像。面板在原图上叠加文字框、识读结果和置信度；没有可读文字时显示对应状态，过期或画面改变的结果会隐藏。它没有把 Depth Anything 当作识字模型，也没有按场景名称补写门牌；当前是否实际识读成功，应查看本轮视觉记录中的 `ocr` 字段。

底层新增可选 `GO2W_RL_DYNAMIC_YIELD_FILE` 心跳：行人等待优先于下坡退出补偿；演示在坡后平地执行等待，避免坡面急停。心跳过期时保持等待；手动控制有独立控制权。未设置该变量的旧入口保留原来的控制行为。

## RViz 建图调整

- MID360 10 Hz，原始回波送入现有 LiDAR/IMU 估计器。
- 当前使用 **3 cm** 显示体素，至少两次独立扫描观测后融合实测坐标，每 2 秒发布全范围累积点云。当前扫描以淡白色另外叠加，显示新回波；融合不插点、不复制回波。
- **5 Hz** 更新自适应膨胀：橙色半透明区域表示当前局部障碍膨胀，青色圆表示机器人当前半径。自适应半径来自 `flycar` 思路的速度、制动距离与通道宽度分析。该半径用于地图可视化分析；执行停车的行人保护使用下述独立阈值。
- 动态候选以稳定跟踪编号、速度、距离、1.4 秒预测箭头及粉色保护圆显示；遇到碰撞风险时保护圆变红并显示 `YIELD`。保护圆半径为观测簇半径 + 0.95 m，与跟随器当前距离保护一致；预测判定另有 0.85 m 裕量。标签为几何候选，不能当成已验证的人体语义分类。
- 2026-10-02 新版把全高度点云设为默认显示，另提供可选建筑剖切、白色几何及当前扫描图层；旧录制版本曾默认做剖切。
- 点云颜色按实测高度连续映射：地面橙/黄、中部绿/青、上层蓝/紫，黑底侧视效果见[RViz 截图](artifacts/lio_map_review_20261002/rviz_spectral_side.png)。[三视图](artifacts/lio_map_review_20261002/lio_map_three_views.png)同时给出完整俯视、建筑剖切和走廊高度剖面。彩色球和连线只表示同次运行中抽样的 LIO 位姿，不表示语义类别或回环约束。
- 2026-10-02 加入[彩色语义与几何并排图](artifacts/lio_map_review_20261002/lio_semantic_overlay_20261002.png)：同次运行的 Grounding DINO / SAM2 深度投影标签，按 5 cm 最近 LiDAR 距离筛选，只保留可对齐的楼梯、坡道、圆柱、墙面、地面候选点。相同图层也已在当前 RViz 窗口中启用。静态语义图不包含移动行人，候选标签未经准确率评测。轨迹节点由 21 个缩到 5 个。
- 完整 OfficeWorld 演示的 RViz 默认打开语义地图、当前语义点和对象标签；行人只在实时扫描中显示，不写入静态地图。
- 初始静止时用仿真里程计和 LIO 位姿做一次刚体坐标对齐，让全局路线叠加到 LIO 地图。该对齐不用于生成或修正 LiDAR 地图点。

3 cm 是显示体素尺寸，并不等于经过验证的绝对测量精度。历史动态回波仍可能出现在累积点云中；当前版本没有声称完成回环优化或高精地图精度评测。旧 PCD 与原始 float32 调试捕获已于 2026-10-02 清理，新调试默认不再写入它们；在线建图和语义节点的 PCD 均仅在显式传 `--export-pcd` 时导出，平时保留压缩 NPZ。

本轮约 222 万个显示点，LiDAR 接收约 10 Hz，膨胀约 5 Hz，预览丢批次数为 0。初始刚体对齐后的 LIO 位姿与仿真里程计位置差 P95 约 **0.939 m**，这是定位一致性检查，不是点云表面误差；目前结果不足以证明厘米级地图精度。

## 导航前瞻与采样显示

RViz 中白色球是跟随器实际使用的前瞻目标；青色线和末端球表示当前下发速度的 2.4 秒平面运动预览，其余六条蓝色线展示左右角速度采样。预测与动态候选的保护范围重叠时线条变红，停车等待时采样长度归零。这些是导航解释图层，**不参与选择控制速度、不代表新接入 MPPI 优化器，也不预测爬楼 RL 的三维姿态**。楼梯仍沿已经验证的地形控制逻辑行走。

## 在这台机器运行

2026-10-02 自动导航与完整点云入口（导航时四窗，成功到门后交接手动）：

```bash
bash '/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/scripts/start_office_global_auto.sh'
```

默认使用本轮已完成 15/15 连续实测的 Smac A*；在命令末尾加 `NavFnDijkstra` 可切换到另一种已求解路线。每次重新初始化场景并建立 `artifacts/office_auto_日期_时间/`，连续行走完整楼梯、坡道、行人通道及二楼桌椅通道，最终目标为玻璃门 `Sm_Door02` 前的 `(13.35,-2.35,5.90)`。最后从 `(12.2,-2.35)` 沿 +X 方向前进约 1.15 m；真实 Nav2 采样路径仍有侧向偏移，NavFn 最大约 42 mm、Smac 约 25 mm。运行过程与结果位于本轮 `course/`；任务不瞬移机器狗。

玻璃门资产的实读 pivot 为 `(14.4944,-1.78776,5.50)`，UE yaw 为 +90°，网格 `ExtendedBounds.Origin.X` 为 56.499660 cm。结合 pivot、旋转并假设实例缩放为 1，推算门中心约为 `(14.4944,-2.352757,5.50)`。实例缩放没有独立核定；本轮已用[成功后的真实在线 RGB 与语义画面](artifacts/office_auto_20261002_184713/door_evidence/full15_success_online_models_20261002_185217.png)核对停靠在带窄竖玻璃的木门前。

门前以 0.30 m/s 接近，进入目标 8 cm 内才请求站立，给步态切换导致的机身后退留出余量。最终要求站姿距离不超过 0.38 m、航向误差不超过 0.12 rad，至少观察 2 秒站姿切换，且连续 1 秒位移不超过 3 cm；姿态、里程计和任务超时保护继续生效。最终事件记录的是停车后的实测机身位置，不能把规划坐标当作已到达的精确坐标。

RViz 主图使用 `/wavego/live_dense/points` 全部经过重复观测过滤的累积实测体素，不按楼层或场景范围裁剪；剖切图层默认关闭，地图随扫描逐步增长。四个窗口在任务完成后继续保留。MuJoCo 只读镜像把墙面、楼梯和斜坡显示为红色；上二楼后切换显示组 5 并让相机跟随机器人，避免楼层或远端走廊把机器狗遮住。镜像直接复制仿真 UDP 中的真实 `qpos/qvel`，只改变显示颜色、图层和相机。

成功到门、航向收敛并保持稳定后自动截图、结束可选录制，再启动右侧紧凑 WASD 面板；当前位置和点云保留。点击面板后使用 W/A/S/D 或方向键；松键或失焦停车。空格/Esc 锁定，点击“接管手动”解锁。**R 会重新初始化仿真并回到起点，不是保持当前位置的站立恢复。**

需要截图时执行下面一行，每次保存一张带时间戳的 3840×2160 PNG 到当前运行的 `screenshots/`，四个窗口按顺序取帧拼接，并非严格同步传感器快照：

```bash
/usr/bin/python3 '/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/scripts/capture_office_four_windows.py'
```

在当前位置重新调出手动面板（不重启仿真）：

```bash
bash '/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/scripts/open_office_manual_here.sh'
```

录制完整四窗口演示时使用：

```bash
OFFICE_RECORD=1 bash '/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/scripts/start_office_global_auto.sh'
```

只采集四个指定窗口，1920×1080、12 fps，独立窗口视频流拼接到同一输出时间轴，并非严格同步传感器录像。视频保存到本轮 `recordings/office_four_windows_时间戳.mp4`，同时记录窗口 ID、采集命令和元数据；成功到门后自动完成 MP4 封装。普通启动不录制视频，原始 float32 和 PCD 录制/导出均默认关闭。

手动驾驶同一套 OfficeWorld 场景时，运行 `bash scripts/start_office_global_manual.sh`。它启动四个窗口和置顶的「Go2-W 手动驾驶」面板，并把控制权交给面板；不启动自动路线。先让面板保持焦点，再按住 **W/S 前进/后退、A/D 左转/右转**，方向键等效；松键或窗口失焦会停止。**1** 是平地模式，**2** 是爬楼/越障模式；**空格/Esc** 停止并锁定，点击「接管手动 / 解除停止锁定」后才能再走；**R** 会重启当前 OfficeWorld 场景、回到起点并恢复站立，创建带日期的新运行目录。按钮也可执行相同操作。

2026-10-02 在 `artifacts/office_manual_20261002_152921/` 实测按住 W 0.75 秒：面板发布 15 个正向速度样本，最高 0.45 m/s，机器狗里程计移动 0.038 m，松键后速度归零。该面板只连接仿真域 89，不向真机下发控制。

```bash
cd '/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916'

# 首次安装本次底层等待修正时编译；原始源码和二进制已有备份。
/usr/bin/python3 scripts/build_inline_bridge.py

# 使用另一种上游路线，按需开启录制。
OFFICE_RECORD=1 bash scripts/start_office_global_auto.sh SmacAStar
```

自动启动和校验使用 UE 的 `-ExecCmds` 与日志，无需切换窗口焦点或输入法。ROS 使用独立的本机 7759 端口，避免共享路由器重启影响本演示。LiDAR 的快速过滤转发运行在独立进程，避免地图计算或压缩保存堵住 LIO 输入；传感器与速度命令设为高优先级，稠密可视化消息设为后台优先级。优先级按 [rmw_zenoh 配置机制](https://github.com/ros2/rmw_zenoh/blob/rolling/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5) 设置。地形模式参数异步切换，里程计和动态点云回调继续处理，真实数据断流仍会停止导航。

重新调用上游规划器求解：

```bash
bash scripts/global_compare/start_planners.sh
source /opt/ros/humble/setup.bash
source genisom_roamerx_open/install/setup.bash
source scripts/office_ros_environment.sh
export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp
/usr/bin/python3 scripts/global_compare/maps.py
PLAN_REVIEW_DIR="$PWD/artifacts/plans_$(date +%Y%m%d_%H%M%S)"
/usr/bin/python3 scripts/global_compare/plan_routes.py \
  --output "$PLAN_REVIEW_DIR"
```

系统 Nav2 规划服务使用隔离环境，避免项目 RoamerX 库覆盖系统 `libnav2_planner`。

## 输出和回退

当前自动入口默认不录制视频；设置 `OFFICE_RECORD=1` 时输出本轮 `recordings/office_four_windows_时间戳.mp4`。每轮保留 `course/events.jsonl`、`course/result.json`、`course/odom.csv`、`course/path_tracking.json`、`capture/`、行人校验、坐标对齐及带时间戳的截图。以 `course/result.json` 的最终状态及视频人工复核作为结果依据，不能仅以进程成功启动为通过。

原始楼梯和斜坡几何未改动。修改前的控制器源码、二进制、启动文件及相关配置位于 `artifacts/global_compare_20261001/source_backup/`。旧版已通过的视频 `artifacts/office_large_full_08_final/officeworld_inline_second_floor.mp4` 保留。本轮不执行 GitHub 推送。

## 历史结果：2026-10-01 的 12 阶段复测

[完整四分屏对比视频（5 分 39.4 秒）](artifacts/global_compare_20261001/demo_final/officeworld_two_global_planners_demo.mp4) · [原 RViz 建图截图](artifacts/global_compare_20261001/demo_final/rviz_dense_map.png) · [2026-10-02 三种实测点云视图](artifacts/lio_map_review_20261002/lio_map_three_views.png)

视频依次完整保留 Dijkstra 和 Smac A* 两轮连续行走，A* 从 02:54.7 开始。两轮使用相同的场景、行人配置、跟随器、桥接程序和显示配置，关键文件的 SHA-256 已比对。每段原始视频均完成全文件解码检查，合并时直接复制视频流。

| 算法 | 完成阶段 | 导航耗时 | 等待 / 恢复次数 | 终点机身高度 | 历史 PCD 点数（文件已清理） |
|---|---|---|---|---|---|
| NavFnDijkstra | 12 / 12 | 160.2 s | 3 / 3 | 5.922 m | 3,324,505 |
| SmacAStar | 12 / 12 | 156.6 s | 3 / 3 | 5.927 m | 3,316,238 |

四分屏分别是 MuJoCo 实时状态、RViz 三维点云与路线、真实视觉模型推理、UE 场景与原生行人。MuJoCo 镜像显示机器人与静态地形；动态人体显示及激光命中验证在 UE 侧完成。本次没有进行人体与机器狗碰撞动力学的定量评测。

两次导航的最近几何障碍簇中心距离分别为 0.892 m、0.888 m；它们不是精确的人体表面间距。初始刚体对齐后，LIO 位置与仿真参考位姿的差值 P95 分别为 0.253 m、0.478 m，包含时间延迟影响。当前结果存在累计误差，未完成回环优化或厘米级测绘精度验证。

这是两次完成路线的演示记录，不作为算法性能或统计成功率对照。调试中出现过坡面急停失稳、反复等待和坡道入口无进展；已按要求清理可确认的失败迭代，完整清单见[存储与建图复核报告](artifacts/lio_map_review_20261002/STORAGE_AND_LIO_REVIEW_20261002.md)。最终通过的是以下两个 `final2` 目录。

- `/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/global_navfn_final2_20261001`：独立 MP4、压缩点云快照、行人验证、控制事件与配置快照。
- `/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/global_smac_final2_20261001`：独立 MP4、压缩点云快照、行人验证、控制事件与配置快照。
- `/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/global_compare_20261001/demo_final`：合并 MP4、六张实际视频关键帧、建图截图、坐标对齐与 `demo_manifest.json`。此前导出的 PCD 已按要求删除；原始 ROS bag 保留。

![最终 RViz 建图](artifacts/global_compare_20261001/demo_final/rviz_dense_map.png)
