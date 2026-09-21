# Wavego 立体语义地图与场景图

**新地图的完整表示入口：[校正地图的 Graph、语义 Box、点云与模型结果](docs/fresh_semantics.md)。** 执行 `bash scripts/show_fresh_semantics.sh`。新记录上已重跑 RGB-D 三模型、HOV-SG 楼层模块与 Utonia 特征，并重新构建 Spark-DSG；原生 RViz 内切换，旧版数据全部保留。本次保存图片，不录制视频。

按北京理工大学 2026 年学位论文规范生成的中文论文图副本位于 `outputs/fresh_semantic_20260919/thesis_figures_cn/`，图内文字已汉化，图题置于图下方并居中；英文原图仍保留。

RGB-D 图中的类别框直接来自 Grounding DINO + SAM2 的模型输出；`label` 保留模型原始英文文本，`label_zh` 只做中文显示翻译，不重新识别、不添加人工框。中文模型结果位于 `outputs/fresh_semantic_20260919/outputs/rgbd_models/panels_cn/`，类别映射见 `outputs/fresh_semantic_20260919/outputs/rgbd_models/label_translation_cn.json`。

之前的二层地图结果已按方法集中保留在 `outputs/fresh_semantic_20260919/outputs/two_floor_maps/`：FAST-LIO 纯点云、DINO+SAM2 语义点云、Graph/Spark-DSG、HOV-SG 楼层分割、Utonia 特征和结构校正点云。目录中使用只读链接指向原始结果，未覆盖任何 PCD。

**最新：[旧版保留、重新采集与楼层水平校正](docs/level_mapping.md)。** 执行 `bash scripts/show_level_comparison.sh`，切换旧图、新采集纯 LIO 和楼层结构校正版。已完成 OfficeWorld 12/12 阶段；新校正版使用已知层高先验，纯 LIO PCD 另存。旧版配色与全部原 PCD 保留。

**旧版配色入口：[FAST-LIO 风格、论文蓝金配色与三组对比](docs/fastlio_style.md)。** 执行 `bash scripts/three_view_demo.sh --style fastlio --mode raw`。支持同图 Graph、语义 Box、按高度着色点云；默认隐藏多彩 Utonia 特征。最新图片位于 `outputs/fastlio_comparison/`，按最新要求只输出图片，不录制。

此前的[学术配色与 Utonia 方法实验](docs/academic_upgrade.md)及 `outputs/academic_upgrade/` 结果保留用于研究记录。

上一版：[三种表示、开源方法对比与 HOV-SG 原版分层](docs/three_representations.md)。旧截图与录屏保留在 `outputs/three_representations/`。

新增：[单地图语义融合、164.7 万点建图回放与模型类别配色](docs/reference_demo.md)。本机运行 `bash scripts/reference_demo.sh`。默认在同一地图叠加语义点云、标签和轨迹；旧双视图可通过 `--layout dual` 查看。

在原有 V2 建图之外新增独立目录：读取 OfficeWorld 的 Go2-W 实际仿真记录，显示彩色语义点云、物体簇、候选楼层／墙面和楼层—区域—物体图。使用本机原生 **RViz + Qt**，不需要网页。

**这是已录制地图的语义后处理演示。还不是 Elevator-LIO、S-Graphs、Hydra、Nav2 全链路在线系统，也没有测量绝对地图精度。**

![原生 RViz 演示](outputs/screenshots/05_native_demo.png)

## 本机启动

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy
bash scripts/build.sh
.runtime/venv/bin/python scripts/validate_outputs.py
bash scripts/view.sh
```

## 中文语义与 Graph 启动命令

下面的命令启动当前二层地图版本。`SCENE_CHINESE=1` 会启用中文语义类别、Graph 节点和 RViz 标签；英文 ontology、类别 ID、坐标、置信度和关系键仍保留在 JSON 中，中文字段仅用于显示。

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy

# 1. 生成/刷新 scene_graph.json 中的中文显示字段
python3 scripts/translate_outputs_cn.py

# 2. 启动三视图：Graph、语义框、纯点云和融合视图
export SCENE_CHINESE=1
export SCENE_DATASET_ROOT="$PWD/outputs/fresh_semantic_20260919"
export SCENE_SESSION_DIR="$PWD/.runtime/fresh_semantic_view"
export SCENE_ROS_MASTER="http://127.0.0.1:11328"
bash scripts/show_fresh_semantics.sh
```

如果需要只查看某一种表示：

```bash
# Graph 场景图
SCENE_CHINESE=1 bash scripts/three_view_demo.sh --style fastlio --mode graph

# Grounding DINO + SAM2 语义点云和目标框
SCENE_CHINESE=1 bash scripts/three_view_demo.sh --style fastlio --mode semantic

# FAST-LIO 风格纯点云
SCENE_CHINESE=1 bash scripts/three_view_demo.sh --style fastlio --mode raw

# Graph + 语义 + 纯点云融合
SCENE_CHINESE=1 bash scripts/three_view_demo.sh --style fastlio --mode combined
```

中文类别和 Graph 关系的固定映射见 `scripts/semantic_labels_cn.py`；二层多方法地图索引见 `outputs/fresh_semantic_20260919/outputs/two_floor_maps/README.md`。

窗口左侧可切换三维总览、楼梯侧视、俯视和层级图。鼠标拖动旋转，滚轮缩放。`?` 表示预测类别；绿色是地面语义，不是激光雷达扫描线。浅蓝为墙面，蓝色楼梯，紫色坡道，橙色圆柱。灰色保留未分类的测量几何。

ROS 1 使用独立 master `http://127.0.0.1:11323`，不控制机器人，不停止 V2 的其他 master。关闭此启动窗口会清理它自己启动的发布节点及 master。

环境可覆盖：`SCENE_ROS_SETUP`、`SCENE_ROS_MASTER`、`SCENE_PYTHON`。本机 Noetic 的 source 文件为 `/home/unitree/flycar/tools/setup_noetic.bash`；不要混用 ROS 2 的 Python 环境。

首次建立计算环境（Python 3.11）：

```bash
uv venv --python 3.11 .runtime/venv
uv pip install --python .runtime/venv/bin/python -r requirements.txt
```

## 已实现与待接入

| 用户提出的模块 | 当前实际状态 |
|---|---|
| MID-360 + IMU → LIO | 使用此前仿真运行保存的 LiDAR/IMU LIO 地图与轨迹。原语义示例对应旧记录；2026-09-19 新采集和独立 LIO 修正版见 `docs/level_mapping.md`。 |
| 电梯 Elevator-LIO | 已核对和下载官方源码，未编译、未运行；此记录没有电梯运动，未创建电梯拓扑边。 |
| 楼层／房间／墙体 S-Graphs | 本目录实现几何候选提取和层级结构，**未运行 S-Graphs 因子图优化**。观测区域不等于已经识别的封闭房间。 |
| RGB/RGB-D 语义投影 | 原版本复用已有模型输出；新记录入口已对 **378 帧新 RGB** 重新运行 Depth Anything V2、Grounding DINO、SAM2，以同步 UE 公制深度和校正 LIO 位姿投影。Depth Anything 相对深度不能直接当作米制深度。 |
| Hydra 物体级场景图 | **实际调用官方 Spark-DSG 1.1.3** 建图、保存和重新加载。未运行完整 Hydra 前端／后端，不能称为 Hydra 重建结果。 |
| 各楼层 Nav2 子地图 | 导出 10 cm PGM/YAML 预览；保留未知区域。没有启动 Nav2，没有验证这些投影图可直接导航。 |
| 楼梯／电梯连接 | 导出记录中高度变化与楼梯观测支持的 `stairs_candidate`；未作通行验证。没有电梯边。 |

官方源码参考：[Elevator-LIO](https://github.com/xiaofan4122/Elevator-LIO)、[LiDAR S-Graphs](https://github.com/snt-arg/lidar_situational_graphs)、[Hydra](https://github.com/MIT-SPARK/Hydra)、[Spark-DSG](https://github.com/MIT-SPARK/Spark-DSG)。本次查看的版本记录在 [integration.md](docs/integration.md)。

## 数据与处理

- 几何：`../WavegoV2/data/office_map.pcd`，292,652 个真实仿真测量点；本流程不补点、不平滑 XYZ、不生成缺失表面。
- 视觉语义：`data/semantic_map.npz`，复制自 `office_rosbag_20260916_01/semantic/semantic_map.npz`，185,841 个带预测标签的点。
- 轨迹：`data/lio_trajectory.npy`，从同一记录的 `/wavego/slam/odom` 提取，共 1,735 帧，列为 `[timestamp_seconds, x, y, z]`。没有使用 MuJoCo 真值位姿。
- 坐标：`lio_map`，与源 LIO 地图采用同一坐标系。地图漂移没有通过本功能修正。
- 标签以 0.16 m 邻域关联到较密的测量地图；原始标签另存 `original_labels`。
- 法向由 20 近邻估计；地面需水平且接近候选层高，墙面需局部竖直。与几何矛盾的结构预测退回未标注，避免整面墙被涂成地面。
- 楼层候选由起终点轨迹平台附近的水平面支持；本例约 `z=-0.50 m` 与 `z=4.68 m`。不是建筑测绘层高，也不推断屋顶为楼层。
- 区域为同层观测水平面的连通簇；墙面为竖直平面 RANSAC 候选；物体为视觉类别内 DBSCAN 簇，可能存在分裂或误分。
- 默认显示 ROI：`x[-3,21]、y[-8,13]、z[-1.5,8.2] m`。全体点保留在输出，RViz 配置提供 `Full measured cloud` 图层。
- 楼层边框仅表示已观测范围包围框，不代表整块区域均可通行。场景图采用单独的示意排版；JSON 中保留物体真实度量位置。

本次没有有效行人检测，不能把圆柱改名为行人。语义类别、连通区域及楼梯通行性都需要后续对齐真值评估。

## 输出文件

输出写入 `outputs/`，输入数据保持不变：

| 文件 | 内容 |
|---|---|
| `semantic_map.pcd` | 全量 XYZ、RGB、label、evidence，二进制 PCD |
| `semantic_cloud.npz` | 点云、精修标签、原始预测、颜色、来源、ROI |
| `scene_graph.json` | 类型节点、结构关系、楼梯候选边、来源和限制 |
| `spark_dsg.json` | 真正 Spark-DSG 序列化文件，已验证可重新加载 |
| `floor_maps/F0.pgm` / `F1.pgm` | 10 cm 三值占据投影，黑色障碍、白色观测地面、灰色未知 |
| `floor_maps/F0.yaml` / `F1.yaml` | Nav2 map-server 格式；XY 沿用 LIO 坐标，高度在场景图中 |
| `report.json` | 点数、节点数、层高候选、来源 SHA-256 |
| `validation.json` | 数据完整性和序列化检查结果 |
| `screenshots/01_semantic_overview.png` | 三维语义总览 |
| `screenshots/02_stairs_side.png` | 楼梯跨层侧视 |
| `screenshots/03_semantic_top.png` | 俯视图 |
| `screenshots/04_hierarchy.png` | 楼层—区域—物体结构图 |
| `screenshots/05_native_demo.png` | 带图例的原生窗口截图 |

`evidence`：0=未标注；1=历史视觉预测关联；2=几何候选；3=被几何检查否定且未重新分配的结构预测。

## 后续仿真运行采集接入口

已增加只读 ROS 2 采集脚本，订阅原有运行节点的地图、语义地图和里程计。它不启动模型或仿真、不产生运动指令；拒绝坐标系不一致或陈旧数据。本次现有传感器流已停止，已验证它会拒绝没有新 LIO 观测的采集、不会生成伪造快照；正常在线采集尚未验证。

在新的 ROS 2 Humble 终端中，先启动原有 OfficeWorld 仿真和视觉语义节点，再执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy
python3 scripts/collect_ros2_inputs.py --output outputs/new_run --seconds 180
```

采集成功后，在不混入 ROS 2 Python 路径的计算环境处理：

```bash
bash scripts/build.sh \
  --geometry outputs/new_run/geometry.pcd \
  --semantics outputs/new_run/semantic_map.npz \
  --trajectory outputs/new_run/lio_trajectory.npy \
  --output outputs/new_scene
```

默认阈值和 ROI 面向本次 OfficeWorld 数据；换场景必须复核。`validate_outputs.py` 的两个层高断言也是本例的回归检查，不是通用建筑楼层标准。
