# 源码、数据与实现范围

## 快照来源

- 运行工程：`/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916`；复制时间、每个原路径、字节数、原 SHA-256 和发布 SHA-256 见 [清单](../source_manifest.json)。源工程未被本次发布操作改写。
- MATRiX 源码基线：`918fae36216e4abb70fbbe275c687c0127fe4b02`，保留 [BSD 3-Clause License](../runtime/matrix/LICENSE)。需要另外取得 UE OfficeWorld、机器人模型等运行资产。
- RoamerX 源码基线：`51d1e9d22e9bf73380e92a9c4792983d0b355ad0`，保留 [BSD 3-Clause License](../runtime/genisom_roamerx_open/LICENSE)。`robot_slam` 完整包源码随快照保存，内部第三方代码保留各自版权头。
- 自适应膨胀来自本机 `semantic_map_preview/adaptive_inflation/adaptive_core.py`；算法受到 flycar 距离场处理的启发，是本地分析组件。[源说明](adaptive_inflation_source.md)保留实验参数和范围；其中本机输出链接不属于发布包。
- 9 月中文标签组件、LAZ 导入、去顶、Z≤5 m 几何特征、局部楼梯拼接源码保存在 `demos/semantic_hierarchy/`。实测 LAZ/微信原始资料没有混入 OfficeWorld 仿真数据包；运行那些入口需准备对应本机输入。

运行源码只做了一处发布路径适配：`dense_capture.py` 导入同目录捆绑的 `adaptive_core.py`。包装层另增只读核验、可选备份恢复、地图查看命令与文档，没有更换控制器或重算原地图。

## 原始记录与版本关系

| 记录 | 本次保留 | 与其他记录的关系 |
|---|---|---|
| 2026-10-01 NavFn / Smac 成功运行 | 六张视频关键帧、验证报告、路线、配置哈希；Smac 压缩点云/轨迹/语义 | 原始每轮坐标对齐 JSON 独立；不以截图证明新代码重跑通过 |
| 2026-10-02 地图复核 | 实际 RViz 截图与离线三视图；当前显示源码 | 使用 10 月 1 日保存数据，属于回放/后处理 |
| 2026-09-16 bag 的原生地图回放 | 567,242 点 ikd-Tree NPZ、图像与出口检查 | 独立旧记录，不能直接与 10 月 1 日地图叠加 |
| 2026-09-19 校正图 / 9 月 23 日中文截图 | 旧查看器源码、截图、状态校验 | 使用显式场景层高先验，不是新一次纯 LIO 精度结果 |

完整 MP4 与原始 rosbag 保留在本机，未放入本分支；视频源绝对路径及关键帧时间均在 [截图说明](../evidence/README.md)。旧 PCD 已在本次任务前清理，历史 `demo_manifest.json` 中的 `pcd_exports` 只保留当时统计，不能解释为仍可下载的文件。

## 算法与可视化边界

- 当前实时工程的 `robot_slam` 是 LiDAR/IMU + ikd-Tree 前端。独立官方 FAST-LIO 的代码保留在 V2；两者不能混称。没有运行完整 Super-LIO、Hydra、LiDAR S-Graphs 或 Elevator-LIO。
- 相机深度投影使用仿真提供的公制深度。Depth Anything 的图是相对深度模型输出，不能当成已标定的米制深度。
- RGB-D 模型类别、点云几何候选、抽样位姿图、9 月 Spark-DSG 物体关系图是不同表示。最新 `trajectory_graph` 中的边仅连接相邻抽样位姿，没有语义关系或回环求解。
- 动态避让使用实测 LiDAR 几何簇。原生行人可见，不代表已评测人体识别精度、人体网格间距或碰撞动力学。
- 自适应膨胀输出为分析/显示图层，尚未验证闭环驱动本页的两种全局规划器。
- 本轮静态路线来自场景几何和地形检查点，不是无图探索。Nav2 2D 求解和 RL 攀爬分别承担各自功能。
- 源消息逐点时间偏移为零；运动去畸变、回环与绝对精度仍待验证。动态回波历史和定位漂移可见于累积图，不能通过配色消除。

## 外部运行依赖

Ubuntu 22.04、ROS 2 Humble；Nav2 NavFn/Smac 1.1.20 为保存记录时版本；RoamerX 的 `rmw_zenoh_cpp`；MATRiX UE/MuJoCo 资源；DreamWaQ Go2-W g2wDWAQ 模型；eCAL 5.13.3、Protobuf 3.12.4、LCM 1.5.1、zsibot_common 0.5.9、LibTorch/CUDA；视觉模型依赖及检查点见 `runtime/models/anything/manifest.json`。这些运行依赖不因提供源码覆盖包而自动安装。

单独看随附点云只需 ROS 2/Python 可视化依赖，无需上述控制、UE 和模型环境。旧中文窗口则使用 Noetic 的 Qt5/Ogre 环境，具体入口见其 README。
