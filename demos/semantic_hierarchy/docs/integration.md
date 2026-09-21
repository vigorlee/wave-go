# 上游核对与集成边界（2026-09-19）

| 上游 | 查看版本 | 本次使用方式 |
|---|---|---|
| MIT-SPARK/Hydra | `2e58a35baea629eee8838409771876acff015daf` | 阅读接口和数据要求；完整系统未构建、未运行 |
| snt-arg/lidar_situational_graphs | `35dd3561730af299a0cc2730e3e3ce44a0699c06` | 阅读多层结构接口；未运行其优化器 |
| xiaofan4122/Elevator-LIO | `1d79af77f3d9747ea57ef52a9b01d326a8ec561a` | 阅读电梯支持与 ROS 版本；本例无电梯数据 |
| MIT-SPARK/Spark-DSG | PyPI `spark-dsg==1.1.3` | 真实安装及调用，序列化往返检查通过 |

源码下载放在 `.runtime/`，不作为已经集成成功的证据。Hydra 当前官方主线说明面向 ROS 2 Jazzy／Ubuntu 24.04；本机是 Humble／Noetic，因此本次选择明确可运行的 Spark-DSG 数据层进行展示，没有将它冒充完整 Hydra。

本例自定义 Spark-DSG 层：2 OBJECTS、3 REGIONS、4 FLOORS、5 BUILDINGS。这不是原版 Hydra 的层号语义；导入 Hydra 应另写 schema 转换，不能把 FLOORS 当成 ROOMS。候选节点设置 `is_predicted=true`，原始来源存入 metadata；关系包含 `contains`、`contains_region`、`near_region`、`bounded_by_candidate`、`stairs_candidate`。

下一步完整集成所需输入和验证：

1. **LIO 模式切换**：先有包含电梯的时间同步 LiDAR/IMU 记录；分别跑 FAST-LIO2 与 Elevator-LIO，评估升降平台和非惯性段，保证切换后的 `map→odom→base_link` 连续性。本目录没有根据高度变化直接切换里程计。
2. **S-Graphs**：需要把同坐标系里程计、注册点云、关键帧／楼层观测送入其真实优化流程，并检验回环后的点云及语义坐标一起更新。当前 RANSAC 不能替代该优化。
3. **完整 Hydra**：输入连续同步的 RGB、米制深度、类别图、相机内参／外参和位姿。当前 rosbag 只有点云／IMU／里程计，没有完整 RGB-D 序列，不能仅凭现有 NPZ 声称跑通其重建前端。
4. **Nav2 多楼层**：各楼层定义独立导航坐标变换，验证投影栅格、机器人高度、障碍膨胀和定位。楼梯／电梯边需要专用行为执行器及通行状态；普通二维 Nav2 不会因一条图边自动上楼。

参考：[Elevator-LIO](https://github.com/xiaofan4122/Elevator-LIO)、[LiDAR S-Graphs](https://github.com/snt-arg/lidar_situational_graphs)、[Hydra](https://github.com/MIT-SPARK/Hydra)、[Spark-DSG](https://github.com/MIT-SPARK/Spark-DSG)。
