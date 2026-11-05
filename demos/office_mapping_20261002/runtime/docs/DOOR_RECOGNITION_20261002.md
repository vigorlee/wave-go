# 门牌 OCR 与三维门关联记录 · 2026-10-02

记录状态：**已核对物理导航、在线门牌/门关联源帧、四窗截图及首段短片全片严格解码**。更新于 2026-10-02T22:06:24.304667+08:00。本记录区分物理到达、文字识读、三维门关联和当前可见性；其中任一通过都不能代替其余项目。

最终运行目录：`/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_215236`。Smac A* 从起点连续完成 15/15，耗时 272.754 秒；自动交接记录分别确认物理导航成功和房号 **01.05** 与目标门关联通过，随后在门前打开 WASD。完整视频、同源传感器证据及源码快照见下方最终验收章节。

此前 `office_auto_20261002_210318` 也记录 15/15，但交接时房号关联尚未核验，后续独立识别才通过；该历史事实保留。以下早期源帧和 45 秒监测属于该旧运行，未把它们当作最终运行的证据，也未把到达预设坐标写成纯视觉识别成功。

## 数据来源和频率

| 数据 | 实际配置 | 话题/处理 | 用途 |
| --- | --- | --- | --- |
| 辅助门牌 RGB | 3840×2160，1 Hz，JPEG，水平 FOV 90° | `/go2w/sign/rgb/compressed` | 从真实源图片自动找牌面，再读文字 |
| 辅助米制深度 | 640×360，1 Hz，float32 轴向米制深度，水平 FOV 90° | `/go2w/sign/depth/image_raw` | 提取文字多边形内的有效深度，拟合所见门面 |
| 门牌识读 | 本地 Tesseract English LSTM，异步单工作线程，默认最短提交间隔 2 s | `scripts/go2w_sign_ocr.py` | 两个预处理尺度原文一致，再进行跨源帧确认 |
| 门分割 | 已加载的 Grounding DINO + SAM2，class 8 | `scripts/go2w_door_target_live.py` | 对与 OCR 相同的归档 RGB 生成门掩膜 |
| 感知位姿 | 源时刻 LIO 里程计 | `/wavego/navigation/odom`，`lio_map` | 把米制深度投到 LIO 地图 |
| 仿真参考位姿 | 源时刻仿真机体里程计 | `/odom/mujoco_odom` | 仅把既有任务目标叠加到 LIO 坐标，记录配准来源 |

两路辅助传感器采用同位置与旋转：UE 配置位置 `(29,0,1)` cm，pitch 45°。1 Hz 是传感器采样配置；**不是 OCR 推理速率**。默认 2 s 提交间隔限制识读为至多约 0.5 Hz，实际频率还受推理耗时与同步数据可用性影响。Depth Anything V2 的相对深度用于视觉展示，未用来读取文字或给门牌恢复米制坐标。

源 RGB、深度和 LIO 位姿按时间戳配对：OCR 结果与归档 RGB 差不超过 25 ms，RGB/深度不超过 50 ms，深度/LIO 位姿不超过 80 ms，源观测年龄不超过 4 s。不能用当前图像代替异步识读对应的历史图片。完整像素框、来源时间戳、深度有效像素数量、米制离散程度和投影状态保存在 JSON 中。

## 确认条件与地图含义

文字识读直接输出原文，不按网格名、预设房号或机器人已到达的位置补写结果。房号先要求不同预处理尺度完全一致，再要求至少三张具有不同源时间戳的图像读出相同文字并满足框位置关联。单张候选不能作为房号确认，重复提交同一时间戳也不累加确认次数。

三维锚点在相同文字的独立源观测之间检查空间一致性。门牌身份确认与门的几何关联分别记录：门需真实模型掩膜中的米制深度点支持，门牌需在门平面附近且关联唯一；还要检查同一物理牌面的相互冲突房号。箭头或多行导向牌保留导向语义，不作为附近房门的名称。检测框、门面拟合和关联阈值以本轮最终配置和验证文件为准，不能用一个条件通过代替整个关联成功。

`sign_position` 是 OCR 多边形内有效实测深度的三维位置；`door_position` 是当前所见门面样本的中心，局部遮挡可能改变该中心；`goal_position` 是既有机器狗停靠目标在输出坐标系中的位置。门牌高度、门面中心与机器人停靠高度含义不同，不能互相替代。

目标叠加使用源时刻 LIO 与仿真参考机体位姿的局部刚体配准：`T_lio_map←map = T_lio_map←body × inverse(T_map←body)`。JSON 标明 `ground_truth_pose_used_for_goal_overlay=true`，同时标明 `ground_truth_pose_used_for_perception_anchor=false`、`lidar_geometry_modified=false`。前者是本演示把已有地图任务目标与漂移的 LIO 显示坐标对应的仿真辅助条件，**不是纯视觉驱动导航**；门牌/门面感知几何来自实测米制深度及 LIO 位姿。该辅助配准不改写 LiDAR 地图点，也不能用于证明测绘绝对精度。

当前可见与持久化锚点明确区分。新的图像失去文字、结果过期或没有新门检测时，已保存锚点可以保留，但应显示保存状态，不继续宣称当前实时确认。3 cm 点云体素仅为显示/融合分辨率，空间一致性和重复识读亦不是经过外部基准验证的厘米级绝对定位精度。

## 本轮证据状态

- 物理连续导航：`/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_210318/course/result.json` 已记录 15/15 完成。
- 真实 4K 原始图：`/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_210318/ocr_4k_validation_20261002/actual_4k_door_00.jpg`、`actual_4k_door_01.jpg`、`actual_4k_door_02.jpg`，来源见 `capture_door.json`；保存的是原 ROS JPEG 字节。
- 离线三图对照：`static_door_multiframe_results.json` 中第一、三张分别读出 `01.05`，分数 0.84、0.79，第二张无接受候选。此离线对照不能写成“三帧均一致通过”，也不是在线确认日志。
- 在线成功的主同步证据：`door_recognition/verified_capture_20261002_212041/door_target_status.json` 及同目录原始 RGB/米制深度，源时间戳 `1790947243.872604`（2026-10-02 21:20:43.872604 +08:00）。`goal_status.state=confirmed_visible`、`confirmed=true`，房号原文 `01.05`，门牌锚点 1、门 1；房号 OCR 跨源帧计数为 5，三维锚点观测窗口计数为 8，身份与门关联均已确认，无身份冲突。主证据位于带日期的专用快照目录。
- 首次确认的历史事实：`observations.jsonl` 保存源时间戳 `1790946995.876418`（21:16:35.876418）的 `confirmed_visible` 记录及此前独立观测。旧 helper 重启后曾覆盖 `confirmed_source.*` 同名便利文件，**当前这些同名图片/JSON 不能被引用为首次 21:16 的不可变源帧**；读取其中内容必须尊重其当前源时间戳。以下首次观测表仅由原始 JSONL 记录支持，不声称现有 alias 图像来自该历史时刻。
- 文字限制：成功源帧读取房号 `01.05`；完整房名实际 OCR 原文为 `Rest Roon`，不是 `Rest Room`。图片可见的房名不等于模型准确读出的原文，文档和报告不将字母误读修正成成功。

### 首次在线确认的独立观测

以下三个源时间戳均来自 `door_recognition/observations.jsonl` 中与同一门 1 关联的实际 `01.05` 观测，不是重复计数一张图片。OCR 帧确认与三维锚点确认是两个独立阶段，三维观测并非要求每个 1 Hz 传感器样本都参与处理。

| 原始源时间戳 | 本地采样时间 | OCR 房号/分数 | 门编号 | 门牌位置（`lio_map`，m） |
| --- | --- | --- | --- | --- |
| 1790946981.838954 | 21:16:21.838954 | `01.05` / 0.78 | 1 | (-6.419577, 2.264076, 6.818185) |
| 1790946992.876732 | 21:16:32.876732 | `01.05` / 0.89 | 1 | (-6.418830, 2.263633, 6.818666) |
| 1790946995.876418 | 21:16:35.876418 | `01.05` / 0.89 | 1 | (-6.419359, 2.264094, 6.818873) |

### 21:20:43 同步快照的质量指标

| 指标 | 实测值 | 含义/限制 |
| --- | --- | --- |
| 源 RGB/深度 | 3840×2160 / 640×360 | 实际辅助传感器分辨率 |
| DINO 门分数 | 0.559296 | 检测模型分数，不是测量精度 |
| SAM2 IoU 预测 | 0.909668 | 分割模型自评分，不是与人工真值实测 IoU |
| 门平面内点 | 1960 / 2400，81.67% | 所见门掩膜中 RANSAC 保留的真实米制深度样本 |
| 平面残差 P90 | 0.003600 m | 渲染深度样本相对所拟合门平面的残差，**不是 LIO/SLAM 绝对精度** |
| 房号有效/保留深度 | 45 / 45 像素 | OCR 多边形中参与实测投影的深度像素 |
| 房号轴向深度 | 2.137727 m | 深度图的相机轴向距离，不是机器人到牌面的欧氏距离 |
| 目标到门面的前向/侧向量 | 1.060063 / 0.008102 m | 源时刻仿真辅助配准后用于目标位置关联，不是外部测绘误差 |

主同步证据的原始 JPEG 为 `/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_210318/door_recognition/verified_capture_20261002_212041/source_rgb.jpg`，解码 PNG 为 `source_rgb.png`，同源米制深度为 `source_metric_depth.npz`；状态与配准见 `door_target_status.json`，来源、时间戳和 SHA256 见 `evidence.json`。这组快照的源时刻为 21:20:43，与 21:16 的首次历史确认分开记录。

### 历史结果保留

- 历史 1920 图及备用 RapidOCR 输出中的 `01.08`、`01.06` 等存在字符误读，不能作为房号真值、不能更正 4K 原文，也未接入主识读服务。
- 早期 `door_identity_handoff.json` 的 `room_plate_goal_verified=false` 是当时检查结果，保留历史事实；后续成功不能回写成当时已成功。

### 持续在线观察窗口

[45 s 实际在线监测](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_210318/ocr_4k_validation_20261002/live_45sec_actual_rate.json)记录 45.084 s 内 18 个实际处理的不同 OCR 源时间戳（1790947237.85427 至 1790947282.873271），全部原文为 `01.05`，18/18 的目标关联状态均为 `confirmed_visible`，该窗口内错误房号和空房号输出均为 0。已逐行核对源时间戳唯一、读取原文和目标状态；处理耗时中位数为 848.95 ms。分母是**此窗口内被 OCR 实际处理的不同源帧**，不是所有 1 Hz 传感器采样；这是同一门前短时观察结果，不是跨场景识别准确率、导航成功率或房名识读精度。仍保留实际 `Rest Roon` 字母误读的事实。


## 可查看的真实演示与同步证据

- [门前四窗口 PNG](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_210318/door_recognition/screenshots/office_four_windows_20261002_211821_662.png)：已目视核对二楼红色 MuJoCo 布局、RViz 全谱点云、辅助门牌原文 `01.05` 和实际 UE 门前机器人。截图中的 `Rest Roon` 等文字原样保留，未修成准确房名。
- [门前四窗口短片](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_210318/door_recognition/recordings/office_four_windows_20261002_211826_016.mp4)：容器及视频流元数据已用 `ffprobe` 核对为 1920×1080、12 fps、480 帧、40.000 s，录制 manifest 状态为 `completed`。视频为已到门后的实时显示短片，**不是另一轮完整 15 阶段导航录像**。四个指定 X11 窗口分别采集并编排到同一媒体时间轴，不是硬件同步传感器录制；根代理已用 `ffmpeg -xerror` 完成一次全片严格解码，退出码 0。
- [同步真实传感器快照](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_210318/door_recognition/verified_capture_20261002_212041/evidence.json)：源时间戳 1790947243.872604，保存原始 4K ROS JPEG 字节及 SHA256，配套米制深度，`status_matches_source_rgb=true`、`goal_confirmed_in_status=true`。RGB/深度时间差 5.96 μs、深度/LIO 位姿时间差约 0.668 ms；这是同源数据匹配检查，不是绝对定位精度。

传感器快照中的 `model_panel.png` 来自主相机可视化，其 `model_panel_same_source_verified=false`，不能作为辅助 4K 门相机的推理同步证明。辅助感知匹配证据应读取同目录 `door_target_status.json` 与原始 RGB/米制深度；旧 `confirmed_source.*` alias 曾被服务重启覆盖，不作为首次历史源帧证据。四窗截图、40 s 短片、首次确认 JSONL 记录与同步快照来自不同时间点，各自时间记录保留，不将媒体帧时间冒充源传感器时间。

### 归档修复与新版显示

最终 helper 将新的成功源帧写入 `door_recognition/confirmed/source_时间戳/` 专用目录，`latest_confirmed_evidence.json` 只指向最新目录。便利 alias 仅在不存在时创建；此前曾发生覆盖的历史已明确保留，**不将这些 alias 称为本运行最早一次确认的不可变原图**。源目录和 JSON 中实际时间戳为准。

[重启后追加归档校验](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_210318/door_recognition/FINAL_VERIFICATION_20261002.json)记录新源 `1790947810.883307` 对应 `confirmed/source_1790947810883307/`，四个既有便利文件 SHA256 全部保持不变；该次实际状态仍为 `confirmed_visible`，房号 `01.05`。验证未发送运动指令，门锚点边界测试 20 项、检测过滤测试 4 项通过。新增归档可查看同源原图、OCR 叠加、米制深度和 `confirmed_source.json`；这次验证证明后续追加行为和既有文件保护，不回写历史文件的源时间。

新版四窗显示已将门牌视图保留原比例、主标注简化为房号，并在 RViz 中显示绿色的任务目标登记点、实测门牌关联连线与 `TARGET` 标签；目标的坐标生成来源仍为上文的仿真辅助配准，没有因显示调整改写点位置。

- [新版四窗口 PNG](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_210318/door_recognition/final_ui/office_four_windows_20261002_212745_324.png)。
- [新版门前 40 s 短片](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_210318/door_recognition/final_ui/office_four_windows_20261002_212749_294.mp4)：已由根代理目视核对并用 `ffmpeg -xerror` 全片严格解码通过，退出码 0。它仍是旧运行 `office_auto_20261002_210318` 的门前显示短片，不能称为之后从起点新跑一轮的全程录像。

## 最后联调中的失败与回撤

两次带日期的从起点联调均未完整到门，保留原始 `course/result.json`，不以较早的成功运行替代其结果。阶段索引以下按日志原样使用，从 0 开始。

| 运行 | 实际终态时间（+08:00） | 完成阶段 | 停止位置/原因 | 原始记录 |
| --- | --- | --- | --- | --- |
| `office_auto_20261002_213142` | 21:34:15.863185 | 8/15 | 索引 8 `native_lower_landing`，横向偏离后触发 `attitude or height safety`；机器人姿态超出安全范围 | [result.json](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_213142/course/result.json) |
| `office_auto_20261002_214332` | 21:47:06.572296 | 7/15 | 索引 7 `native_stair_approach`，新增平地入口校验中的纠偏未收敛，触发 `no progress` | [result.json](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_214332/course/result.json) |

第一轮只读日志显示楼梯助力的横向偏离扩大；失败前模型耗时与成功运行接近、LiDAR/LIO 均约 10 Hz，现有证据不支持明显模型阻塞直接导致跌倒。没有完整逐周期策略调度记录，因此不作“已排除所有时序问题”的结论。第二轮新增入口校验也没有通过实测；这项临时路径跟随门控已回撤，恢复先前路径跟随控制内容，保留门牌 OCR、米制几何关联、追加式证据归档与显示改进。

本次门牌功能的真实验证与导航稳定性分开报告。上述失败表明不能将这次门牌迭代写成“导航已变得稳定”或“每次都能完整通过”。之后若一次完整复跑成功，记为该次验收结果，仍保留这两次失败；这些调试运行没有组成预先设计的统计对照，也不据此计算泛化成功率。

## 回撤后最终单次全程验收：`office_auto_20261002_215236`

本段仅引用从起点重新启动、加载最终门牌代码的这一轮，未复用较早运行的成功 JSON、同源传感器快照或 18/18 监测统计。一次完成的演示不能改写前两轮失败，也不用于声明整体导航稳定性获得提升。

| 项目 | 本轮实际结果 | 证据 |
| --- | --- | --- |
| 起点/完整阶段 | `start_stage_index=0`，15/15，`passed=true`、`succeeded` | [course/result.json](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_215236/course/result.json) |
| 导航计时 | 21:53:17.618823 至 21:57:50.372844，**272.754 s** | 首次 `stage_starting` 至最终 `succeeded` 的日志时间差，见 [events.jsonl](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_215236/course/events.jsonl) |
| 最后停车参考位姿 | `(13.155210, -2.323423, 5.871621)` m，yaw `-0.069011` rad | 来自本轮最终导航事件中的实际仿真参考位姿，不是写入目标值 |
| 相对既有停车目标 | 水平距离 0.196595 m，三维距离 0.198632 m，航向误差 3.954° | 与 `(13.35,-2.35,5.90)`、yaw 0 的任务目标比较；不代表点云测绘误差 |
| 交接时门牌核验 | `physical_navigation_passed=true`，`room_plate_goal_verified=true`；源时刻 **21:58:02.732852** 的实际原文 `01.05`，锚点 1、门 4，`confirmed_visible` | [door_identity_handoff.json](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_215236/door_identity_handoff.json) |
| WASD 交接 | 21:58:09 +08:00 在门前打开手动控制 | [manual_handoff.txt](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_215236/manual_handoff.txt) |

交接后独立采集的新同步快照在 [verified_capture_20261002_215810](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_215236/door_recognition/verified_capture_20261002_215810/evidence.json)，其实际源时间戳为 **1790949493.734449（21:58:13.734449 +08:00）**，与交接检查的源时刻分开记录。原始 4K JPEG、640×360 米制深度和 OCR/门关联状态源时间戳完全匹配；RGB/深度差 **6.914 μs**、深度/LIO 位姿差 **0.600 ms**、深度/仿真参考位姿差 **0.355 ms**。采集器未发送运动指令。完整几何/文字状态见同目录 [door_target_status.json](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_215236/door_recognition/verified_capture_20261002_215810/door_target_status.json)，本轮原图为 [source_rgb.jpg](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_215236/door_recognition/verified_capture_20261002_215810/source_rgb.jpg)，深度为 `source_metric_depth.npz`。主相机 `model_panel.png` 仍不是辅助门相机的同源推理证明。

该新快照实际房号 OCR 分数 0.77、跨源确认计数 9；三维锚点观测窗口计数 7，锚点 1 关联门 4，身份与门关联均为 true，无身份冲突。门检测分数 **0.722331**，SAM2 IoU 预测 **0.940938**；门平面内点 **2089/2400（87.04%）**、残差 P90 **0.004159 m**。文字多边形内深度有效/保留像素 **123/116**，轴向深度 **2.073281 m**。这些是本轮源图与渲染米制深度的识读、关联及拟合质量，仍不等于经外部基准验证的地图绝对精度。目标叠加继续使用源时刻 LIO/仿真参考配准，未根据 OCR 自动生成或选择新的导航目标。

本轮 [完整四窗 MP4](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_215236/recordings/office_four_windows_20261002_215314_863.mp4) 为 1920×1080、12 fps、3525 帧、293.750 s，容器与流元数据已用 `ffprobe` 核对，录制在正常请求停止后完成封装。媒体包含导航前后显示时间，因此不以 293.750 s 替代导航日志的 272.754 s；四窗采样亦非硬件同步记录。根代理已用 `ffmpeg -xerror` 做全片严格解码，**退出码 0**。录制 manifest 为 `stopped_by_request`、`container_valid=true`；FFmpeg 在正常 SIGINT 停止时返回 255，不能将其单独解释为媒体损坏，封装与严格解码均已通过。实际文件为 46134538 字节。[门前四窗 PNG](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_215236/screenshots/office_four_windows_20261002_215805_298.png) 属于这一轮的成功后截图，旧运行的新版 UI 截图/40 s 短片另列在上文。

手动交接后站姿变化的短时状态曾出现 OCR `confirming_text` 和目标登记状态 `confirmed_persisted`，已保存身份与当前实时可见性仍分别处理。本轮结论限定为全程物理成功、交接时实际识读/关联成功，以及之后保存的同源快照；**不宣称本轮每帧持续 100% 识读通过**。前文 45 s、18/18 的监测属于旧运行 `210318`，不计入本轮统计。

本轮 [源码/配置快照清单](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_215236/source_snapshot_20261002/manifest.json)保存 15 个文本文件、184471 字节，排除模型权重、PCD 与视频。实际 `scripts/global_compare/follow_route.py` 及 `office_auto_20261002_184713` 成功时的源快照 SHA256 均为 `578a9ad1924be63f4facd759e42cd5a04da615ad6b795d336dca85897efbda50`，已只读重新计算核对；临时导航入口门控已撤回。本轮验证的是恢复该跟随器后加载最终门牌功能的一次完整任务和真实目标关联，不把撤回的临时改动写成导航稳定性改善。 本轮综合验收清单见 [FINAL_DOOR_DEMO_VERIFICATION_20261002.json](/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261002_215236/FINAL_DOOR_DEMO_VERIFICATION_20261002.json)，独立记录起点 0、15/15、交接房号核验、同步源捕获、全片解码、控制器散列，以及四窗与 WASD 面板实际存在的检查。

## 运行入口、状态文件与 WASD

重新开始完整演示：

```bash
bash /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/scripts/start_office_global_auto.sh
```

默认 Smac A*；参数 `NavFnDijkstra` 可选另一种已求解路线。每次入口初始化场景并建立 `artifacts/office_auto_日期_时间/`。需要四窗口录制时增加环境变量 `OFFICE_RECORD=1`；录制媒体时间轴与任务/传感器时间戳分开记录。四窗口依次为红色 MuJoCo 实际状态视图、RViz 全点云/轨迹/动态膨胀、RGB+Depth Anything+Grounded SAM2 可视化、OfficeWorld UE 实际场景。

当前运行目录由 `/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/.run/go2w/current_output_dir` 指示。关键文件如下：

| 文件 | 含义 |
| --- | --- |
| `course/result.json` | 全程阶段计数、机器人真实参考位姿与终态 |
| `door_recognition/status.json` | 最新 OCR、锚点、门面质量与目标关联状态 |
| `door_recognition/observations.jsonl` | 带不同源时间戳的投影及关联记录 |
| `anything/frames.jsonl` | 主视觉推理与模型输出记录 |
| `door_recognition/confirmed_source.json` | 旧便利 alias；曾在服务重启后的新成功中覆盖，以 JSON 当前源时间戳为准，不当作首次历史主证据 |
| `door_recognition/confirmed_source_rgb.png`、`confirmed_ocr.png` | 与便利 alias 配套的图片；源时刻应核对 JSON，不称首次历史帧 |
| `door_recognition/confirmed_metric_depth.npz` | 与便利 alias 配套的深度/标签，不代替带日期的同步快照 |
| `door_recognition/verified_capture_20261002_212041/` | 本记录主同步证据：原始 RGB、米制深度、同源状态及来源清单 |
| `door_recognition/confirmed/source_时间戳/` | 最终版本成功源帧的追加式归档目录；源 JSON/图/深度对应同一实际时刻 |
| `door_recognition/latest_confirmed_evidence.json` | 最新追加目录指针；可更新，历史证据读取被指向的具体目录 |
| `door_identity_handoff.json` | 交接时物理到达与房号核验分别是否通过 |
| `manual_handoff.txt` | 实际手动面板交接时间 |

完成物理导航后，交接脚本最多等待 40 s 检查房号关联；即使识读未通过，仍保留物理成功事实并打开手动面板，文件会明确核验未通过，不能以 WASD 已打开作为 OCR 成功证据。

已经运行到门前、只需重开手动面板时：

```bash
bash /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/scripts/open_office_manual_here.sh
```

此入口保留当前位置、当前地图和实验视图，不重新开始路线。点击“接管手动 / 解除停止锁定”后按住 W/↑ 前进、S/↓ 后退、A/← 左转、D/→ 右转；A/D 是转向而非侧移。松键或失焦停车，空格/Esc 停止并锁定，再点击接管解除。1 选择普通行走，2 选择爬楼/越障模式，速度滑块 0.15–0.80 m/s。**R 会重启仿真回起点，不是原位站立恢复。**需要截图可单独运行 `scripts/capture_office_four_windows.py`，不会发送运动指令。

## README 可合并段落

以下段落由带日期的已保存同步快照支持；截图已目视核验、首段 40 s 视频已全片严格解码通过，不能用媒体状态代替源传感器证据：

> 门牌源图升级为 3840×2160、1 Hz 辅助 RGB，配对 640×360、1 Hz 米制轴向深度。Tesseract English LSTM 在低频异步线程中读取真实文字，要求不同预处理尺度及至少三张不同源时间戳图像的原文一致；默认 OCR 提交间隔 2 s。Grounding DINO/SAM2 的同源门掩膜支持门面深度拟合，文字的三维位置由米制深度与源时刻 LIO 位姿恢复，并检查唯一门关联、导向牌及冲突身份。本轮 `office_auto_20261002_210318` 的 Smac A* 完成 15/15；`observations.jsonl` 保留 21:16 的首次历史确认，主同步证据 `door_recognition/verified_capture_20261002_212041/` 记录源时刻 21:20:43.872604 的 `01.05` 房号、门 1、八次独立三维观测的窗口计数及 `confirmed_visible=true`，原始 RGB、同源米制深度与状态均已保存。旧 `confirmed_source.*` 便利文件曾被重启覆盖，不作为首次源帧证据。实际房名识读为 `Rest Roon`，未声称精确读取 `Rest Room`。
>
> RViz 中的任务目标叠加采用源时刻仿真参考位姿与 LIO 的局部刚体配准，明确记录仿真辅助来源；它不修改 LiDAR 几何，也不用于生成门牌识别原文或其感知三维坐标。该演示沿既有规划路线导航并验证所见门牌，不是纯视觉房号导航，亦未完成外部测绘绝对精度评估。当前可见和保存锚点分别显示；识读/关联失败时保留失败状态。
