# 同一地图的 Graph、语义 Box 与纯点云

> 这是上一版结果说明。2026-09-19 配色与 Utonia 升级后，当前启动器默认输出到 `outputs/academic_upgrade/`；请使用[新版说明](academic_upgrade.md)及 `scripts/package_academic_upgrade.py`。本页中的灰色截图、旧视频和旧打包验证保留用于对照。

核对日期：2026-09-19。这个演示使用 Go2-W 在 OfficeWorld 的同一次传感器记录，在一个原生 RViz 窗口切换三种地图表示，也可以融合叠加。不是网页。

## 打开 demo

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy
bash scripts/three_view_demo.sh
```

默认使用独立 ROS 1 master `http://127.0.0.1:11324`，不修改 V2 配置，不发送机器人控制指令。关闭窗口会清理此次启动的发布进程。需要自定义 ROS 环境时使用 `SCENE_ROS_SETUP`、`SCENE_ROS_MASTER`；默认 Noetic 环境为 `/home/unitree/flycar/tools/setup_noetic.bash`。

默认打开融合叠加；可以用 `--mode graph`、`--mode semantic` 或 `--mode raw` 指定初始模式。

- **Graph**：在地图坐标中显示节点、包含关系及楼梯连接候选，背景为淡化点云。
- **语义颜色 + Box**：楼梯蓝色、坡道紫色、圆柱橙色、墙面浅灰蓝、地面绿色；未分类点保持灰色。Box 是预测语义簇的范围。
- **纯点云**：单色测量几何，无 Box 和 Graph。可勾选“按高度着色”；高度色不是语义类别。
- **融合叠加**：同一坐标中同时显示 Graph、语义颜色和 Box。
- **HOV-SG 原版分层对照**：运行原版分层模块后可用；蓝色与金色表示两个楼层区间，灰色为区间外点。这是最终地图的静态结果，不是语义类别颜色。
- 鼠标拖动旋转、滚轮缩放；支持总览、侧视、俯视。时间轴和 1/2/4/8 倍速用于回放累积过程；“轨迹”显示原 LIO 记录。

前三张总览采用相同相机、时间终点和坐标。切换图层不移动或配准点云；`lio_map` 是源记录 `map` 的既有显示别名。

## 本次实际结果

| 表示 | 数据来源与范围 | 数量 |
|---|---|---:|
| 纯点云 | `/wavego/slam/registered_points`；4 cm 体素保留首个原始测量点 | 1,647,372 点 |
| 语义 | 保存的 Grounding DINO + SAM2 掩膜、公制 UE 深度、LIO 位姿和时间融合；12 cm 内关联到稠密点云 | 503,170 个已分类点 |
| 未分类几何 | 不强行填补类别 | 1,144,202 点 |
| 物体 Box | 楼梯 3 簇、坡道 1 簇、圆柱 3 簇；Box 只用该原始预测类别的关联点计算 | 7 个预测簇 |
| Graph | 真实调用 Spark-DSG 保存/加载；上游输入是本项目的结构候选提取 | 28 节点 / 28 边 |

Graph 包含 1 个数据容器、2 个候选楼层、5 个观测区域、7 个物体簇及 13 个墙面候选。区域尚未验证为封闭房间；楼梯边未作通行验证。容器节点的位置是展示锚点，不是测量得到的建筑重心。其余节点沿用已保存地图中的位置，未采用独立树状图排版。

**这是同一地图的三种表示对照，不是三个 SLAM 算法的精度排行榜。** 原扫描约 173.322 秒，共 1,735 帧。回放按测量点首次出现时间增长；语义使用最终离线标注，Graph 也来自最终地图，不能当作在线推理延迟实验。原模型的误分类仍可能存在；本记录没有有效行人检测。没有完整语义真值，因此不提供未经测量的 mIoU、ATE 或准确率。

## 与公开方法对比

本次查阅作者仓库和项目页，包含 2026 年的新方法及仍在维护的相关系统。表中“提供”指上游能力，不代表本机已完整复现。各仓库默认分支提交及日期见 [upstream_versions.json](upstream_versions.json)；提交日期不等于论文发表日期。

| 方法 / 公开来源 | 核心输入 | Graph 与语义表示 | 与当前任务的关系 | 本机本次状态 |
|---|---|---|---|---|
| [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) | LiDAR + IMU | 几何点云与轨迹；本体不提供语义场景图 | 纯点云与位姿基础，需要另接语义层 | 使用已有 LIO 注册扫描；本次不重跑里程计 |
| [LiDAR S-Graphs](https://github.com/snt-arg/lidar_situational_graphs) | LiDAR / 里程计 | 结构层级和优化约束 | 更适合墙体、房间、楼层的结构化地图；物体语义需另接视觉 | 已检查源码，未运行其因子图优化 |
| [Hydra](https://github.com/MIT-SPARK/Hydra) | 标定、位姿及几何/语义传感器流 | 层级三维场景图，几何与对象等节点 | 与 Graph + 物体 + 几何共同表达直接相关 | 运行了 Spark-DSG 数据结构；完整 Hydra 未编译通过、未运行 |
| [HOV-SG](https://github.com/hovsg/HOV-SG) · RSS 2024 | 有位姿 RGB-D | 开放词汇对象及楼层—房间—对象层级 | 多层语义地图的合适参照 | **本次运行原版楼层分割阶段**；不是完整 SAM/CLIP/房间/对象流程，实验结果见下方 |
| [ConceptGraphs](https://github.com/concept-graphs/concept-graphs) | 有位姿 RGB-D，分割与视觉语言特征 | 三维对象地图及对象关系 | 对照语义实例、物体合并与关系图；仅 DINO/SAM 标签不等于运行它 | 已查阅官方流程，未完整运行 |
| [Khronos](https://github.com/MIT-SPARK/Khronos) · RSS 2024 / ROS 2 更新 | 连续几何与语义观测 | 随时间变化的度量语义地图 | 动态人/物体与长期场景变化的后续候选 | 未运行；官方 ROS 2 版本要求至少 Iron，本机主环境为 Humble |
| [KeySG](https://github.com/keysg-lab/KeySG) · ICRA 2026 | 有位姿 RGB-D、关键帧与视觉语言模型 | 楼层—房间—物体/功能元素，查询上下文 | 本次检索到有公开实现的近期层级场景图方法 | 已核对源码入口，未完整运行；当前包缺连续 RGB-D |
| [Hydra++](https://hydra-plusplus.github.io/) · IROS 2026 | RGB-D；项目还展示 LiDAR 混合输入 | 在场景图中增加对象形状估计 | 与“更完整的对象几何”相关 | 项目页标注代码待集成、时间待定；不能标为已复现 |

我的实现选择：保留现有 LIO 几何，提供同坐标的语义实例层与 Graph 层；用明确版本的 HOV-SG 模块检查实际数据适配情况。若继续做完整上游复现，需要采集同步 RGB、公制深度、相机内外参、位姿和时间戳，再分别送入 HOV-SG / ConceptGraphs / KeySG；不能用点云截图代替 RGB-D 输入。Hydra/Khronos 建议独立部署适配的 ROS 2 环境。这里没有把不同数据集的论文指标混在一起比较。

## HOV-SG 原版模块实验

脚本：`scripts/run_hovsg_floor_probe.py`。固定提交 `d6e65a53c8be6faec3f01f00d1644d967f89e605`，直接编译并调用 `hovsg/graph/graph.py` 内的 `Graph.segment_floors` 原函数 AST 和原 `Floor` 类。这样绕开无关的模型初始化依赖，**不改变该函数的阈值、步骤或代码分支**。输入仅作 ROS Z-up → HOV-SG Y-up 刚体坐标转换，不缩放、不补点。

输出 `outputs/three_representations/hovsg_floor_probe/result.json` 记录源码 SHA、函数 SHA、输入点数、层区间、状态和单模块耗时；同时保留上游输出的高度直方图与楼层 PLY。该实验用于检测兼容性，既不是完整 HOV-SG 重建，也不是完整流程速度测试。

实际运行成功，输入 **1,647,372 点**。原版模块内部按 5 cm 降采样计算高度直方图，共 981,546 点；最后用分割区间裁取原输入点云：

| 原版输出 | ROS Z 高度范围 / m | 原输入点数 |
|---|---:|---:|
| 层区间 0 | −0.954 ～ 3.168 | 976,504 |
| 层区间 1 | 3.438 ～ 6.429 | 504,592 |
| 区间外 / 未分配 | 不作补全 | 166,276 |

这里的区间覆盖楼层的三维体积，不是楼面表面的标高。本项目原有 Graph 的两个地面候选约在 −0.50 m、4.68 m，分别落在两个区间中；仅这一点相符，不能据此宣称楼层边界或房间识别精度已验证。没有把未分配点强行并入某一层。

上游保存的 `0.ply`、`1.ply` 保留 HOV-SG 的 Y-up 坐标；RViz 则把 `floor_labels.npz` 关联回原始 ROS Z-up 测量点，仍在 `lio_map` 中显示。不要直接把这些上游 PLY 当作 Z-up PCD 使用。

运行方式：

```bash
git clone https://github.com/hovsg/HOV-SG.git .runtime/HOV-SG
git -C .runtime/HOV-SG checkout d6e65a53c8be6faec3f01f00d1644d967f89e605
uv pip install --python .runtime/venv/bin/python -r requirements-hovsg-probe.txt
env -u PYTHONPATH -u LD_LIBRARY_PATH .runtime/venv/bin/python scripts/run_hovsg_floor_probe.py
```

如果源码目录已存在，跳过 clone。原仓库许可证保留在 `.runtime/HOV-SG/LICENSE`。实验不调用外部 VLM API，不上传场景图像。

## 图片与视频

均保存到 `outputs/three_representations/`：

| 文件 | 内容 |
|---|---|
| `01_graph.png` / `01_graph_map.png` | Graph 总览 / 纯 RViz 画布 |
| `02_semantic_boxes.png` / `02_semantic_boxes_map.png` | 语义颜色和物体 Box |
| `03_raw_cloud.png` / `03_raw_cloud_map.png` | 单色纯测量点云 |
| `04_combined.png` / `04_combined_map.png` | 同图融合叠加 |
| `05_stairs_side.png` / `05_stairs_side_map.png` | 跨层侧视 |
| `06_hovsg_upstream_floors.png` / `06_hovsg_upstream_floors_map.png` | 实际运行 HOV-SG 原版楼层分割后的同场景结果 |
| `three_representations.png` | 三种表示并排对照图 |
| `three_representations_demo.mp4` | 实际 RViz 窗口录屏，含累积回放、模式切换和侧视 |
| 同名 `.json` | 截图时相机、图层状态、回放时间和点数 |
| `validation.json` | 同视角、点数、语义来源、Graph 和输出完整性检查 |
| `live_ros_audit.json` | 实际 ROS 消息的点数、Box 数、图节点和关系、坐标核对 |

重新截图和录屏：

```bash
bash scripts/three_view_demo.sh --capture
bash scripts/three_view_demo.sh --record
bash scripts/three_view_demo.sh --capture-hovsg
env -u PYTHONPATH -u LD_LIBRARY_PATH .runtime/venv/bin/python scripts/package_three_views.py
```

原始文件不变。可复用数据：`outputs/reference_demo/dense_measured_map.pcd` 为 XYZ；`dense_model_semantic.pcd` 为 XYZ/RGB/label；`outputs/scene_graph.json` 为含来源的节点与边；`outputs/spark_dsg.json` 为 Spark-DSG 序列化图。

完整源包位于：
`/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_rosbag_20260916_01/go2w_pointcloud_odom_noetic.bag`。

需要重新生成稠密点云时，先按照主 README 运行 `scripts/build.sh`，再执行：

```bash
source /home/unitree/flycar/tools/setup_noetic.bash
.runtime/venv/bin/python scripts/extract_mapping_replay.py \
  --bag /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_rosbag_20260916_01/go2w_pointcloud_odom_noetic.bag
env -u PYTHONPATH -u LD_LIBRARY_PATH .runtime/venv/bin/python scripts/prepare_native_semantics.py
```

计算脚本使用项目隔离的 Python 3.11 环境；RViz 使用 Noetic Python。不要混入 ROS 2 Python 路径。
