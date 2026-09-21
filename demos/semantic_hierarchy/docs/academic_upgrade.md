# 学术配色与 Utonia 三维特征升级

> 此页记录上一轮实验。最新用户要求改为 FAST-LIO 风格、隐藏默认 Utonia 多彩页、只提供图片，见 [最新运行说明](fastlio_style.md)。本页旧录屏被用户取消，不作为最终交付；当前启动器输出目录与默认配色已更新。

核对和运行日期：2026-09-19。使用同一份 OfficeWorld / Go2-W 记录，在原生 RViz 显示 Graph、语义颜色 + Box、测量点云，并增加官方 Utonia 特征视图。原始数据和此前截图保留；新版结果位于 `outputs/academic_upgrade/`。

## 本机启动

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy
bash scripts/three_view_demo.sh --mode raw
```

顶部可切换 Graph、语义颜色 + Box、纯点云、融合叠加、HOV-SG、Utonia。默认启动为融合叠加；`--mode utonia` 直接打开特征图。拖动旋转，滚轮缩放，底部可选总览、侧视、俯视。回放是已录制扫描累积，语义是最终离线标注。HOV-SG / Utonia 是最终地图的静态结果，因此禁用回放时间轴。

ROS 1 使用独立 master 11324。这里没有新增在线 SLAM，也没有控制机器人。当前包缺少连续同步 RGB-D，不能称作完整开放词汇在线建图。

## 配色

纯点云默认采用 [Scientific colour maps 的 Batlow](https://www.fabiocrameri.ch/colourmaps/)。连续色表示高度，范围固定 **−1.5～8.2 m**，不随视野或回放进度拉伸。关闭“Batlow 高度色”时改用靛蓝单色。背景为近白色；范围外点使用端点色，所有 XYZ 保留。

语义、Box、对应 Graph 节点统一使用以下分类色：

| 语义 | 配色 | 十六进制 |
|---|---|---|
| 楼梯 | 靛蓝 | `#3b5ba5` |
| 坡道 | 紫 | `#9d6db1` |
| 圆柱 | 赭金 | `#d9954a` |
| 行人 | 玫瑰色；本记录无有效检测 | `#c65d7b` |
| 墙面 | 青蓝 | `#68adb7` |
| 地面 | 青绿 | `#63a68f` |
| 未分类 | 浅蓝；淡化背景 | `#709ab8` |

原 DINO + SAM2 标签保持不变。自定义分类色没有声称经过色觉障碍可辨性认证。`config/academic_palette.json` 保存配色及 Batlow 256 色 LUT；LUT 生成使用 cmcrameri 1.9，RViz 运行不依赖这个包。

## 已实际运行：Utonia

调用 [Utonia 官方 ICML 2026 编码器](https://github.com/Pointcept/Utonia) 及官方 `get_pca_color` 函数。RGB、法向量缺失时按官方方式置零，没有输入已有语义标签。源码未修改。

| 项目 | 本机结果 |
|---|---|
| 源测量地图 | 1,647,372 点，4 cm 累积体素 |
| 编码器输入 | 12 cm 体素各取一个原始点，共 199,124 点 |
| 官方变换后 | 198,842 点 |
| 特征 | 两级拼接后 1,224 维 |
| 显示 | PCA 颜色按体素对应映射回全部原始 XYZ |
| 设置 | seed 37、scale 0.5、官方 non-Flash 分支、patch size 256 |
| GPU | RTX 4090 |
| 单次编码 / 回映射 / PCA 计时 | 3.085 秒，不含模型和文件加载 |
| 峰值 PyTorch allocated 显存 | 6,855,099,392 bytes，约 6.38 GiB |

非 FlashAttention 分支和 patch size 256 用于适配本机环境，因此不是论文默认设置下的速度复现；上述计时不是在线建图帧率。没有训练分类头，特征色不代表“墙/楼梯”等类别或置信度。此次没有证明减少误检、提高地图精度或增加测量密度。

源码提交：`da776a0bd3a48c6df83ac2ae0e27b26141cc7e31`；模型 `Pointcept/Utonia`，revision `fb39b40d33d215d29d1d4717ffb86287f4700c4d`。权重 SHA-256：`a0a2e5e234ce943082ae9ba26bcff74e285f16ed8d04fed5c36ffe8dc30ea353`。只下载推理权重，没有上传本机地图。

## 最新方法对比

均查阅作者仓库。适配建议基于当前输入条件；论文能力不等于本地图已取得的效果。

| 方法 | 时间与定位 | 可升级内容 | 本次状态 / 适配条件 |
|---|---|---|---|
| [Utonia](https://github.com/Pointcept/Utonia) | ICML 2026，跨域点云编码器 | 学习三维特征，为实例区分、相似度与标签传播提供基础 | **已跑官方编码器 + PCA**；未训练分类头 |
| [RAG-3DSG](https://github.com/CYandYue/RAG-3DSG) | 作者仓库标注 ECCV 2026 | 重渲染对象视角，与已有描述对照，利用可靠邻近对象修正不确定描述及场景图 | 优先后续候选；需带位姿的连续 RGB-D 与描述模型。本次查阅，未运行 |
| [SAM 3.1](https://github.com/facebookresearch/sam3) | 官方 2026-03-27 更新 Object Multiplex | 文本概念分割与多目标视频跟踪，可升级视觉前端 | 需视频、公制深度、标定和位姿才能投影到图；权重需官方访问授权。本次未运行 |
| [vS-Graphs](https://github.com/snt-arg/visual_sgraphs) | RA-L 2026，视觉 SLAM + 层级图 | 墙地构件、场景图优化，扩展房间/走廊/楼层结构 | RGB-D / RGB-D-IMU，主要测试 ROS 2 Jazzy / Ubuntu 24.04.2；需独立环境，本次未运行 |
| [Interactive-ConceptGraphs](https://github.com/BrunoGeorgevich/Interactive-ConceptGraphs) | 作者公开扩展 | 对象过滤、房间分割和交互查询 | 已查看 DBSCAN 去噪、包围框等源码入口；未运行完整流程 |

建议顺序：先补录同步 RGB、公制 depth、相机内外参、时间戳和 LIO 位姿；独立评估 SAM 3.1 分割/跟踪，再评估 RAG-3DSG 对象融合和描述修正。Utonia 可作为几何特征支路，与视觉标签交叉核对。vS-Graphs 涉及前端和优化系统替换，适合单独完整系统实验。组合尚未验证，不能假定混接一定提高精度。

要量化改善，需要本场景的对象、关系和几何参考，分别评估语义 mIoU、实例合并/误检率、关系准确性和地图误差；当前只做数据完整性及可视化检查。版本记录见 `outputs/academic_upgrade/upstream_versions.json`。

## 复现与导出

本机已准备 `.runtime/utonia_env`（Python 3.10、Torch 2.7.1/cu128）、源码和权重：

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy
env -u PYTHONPATH HF_HUB_OFFLINE=1 OMP_NUM_THREADS=8 \
  .runtime/utonia_env/bin/python scripts/run_utonia_features.py
env -u PYTHONPATH -u LD_LIBRARY_PATH \
  .runtime/venv/bin/python scripts/export_academic_maps.py
```

本机隔离环境通过 `wavego_torch_runtime.pth` 只读复用 `.venv-anything` 和 `unitree_lerobot` 的已有 Torch / 基础依赖，新增依赖仅安装在 `.runtime/utonia_env`。迁移机器时按 Utonia 官方说明创建独立环境，不复制这些本机路径。`requirements-utonia-local.txt` 是版本记录，不是跨 CUDA 的通用安装清单。

源码和权重重新准备方式（已有源码目录时跳过 clone）：

```bash
git clone https://github.com/Pointcept/Utonia.git .runtime/Utonia
git -C .runtime/Utonia checkout da776a0bd3a48c6df83ac2ae0e27b26141cc7e31
hf download Pointcept/Utonia utonia.pth \
  --revision fb39b40d33d215d29d1d4717ffb86287f4700c4d \
  --local-dir .runtime/utonia_weights
```

Apache-2.0 许可证保留在 `.runtime/Utonia/LICENSE`。PCA 函数从固定提交的官方文件 AST 加载，未改写。

## 图片、视频和 PCD

新版文件均位于 `outputs/academic_upgrade/`：

| 文件 | 内容 |
|---|---|
| `before_after_height.png` | 原灰色 / Batlow，同视角对照 |
| `academic_three_representations.png` | Graph、语义 Box、纯点云并排图 |
| `01_graph.png`～`05_stairs_side.png` | 原生截图和跨层侧视 |
| `06_hovsg_upstream_floors.png` | 上次已跑通的原版 HOV-SG 分层 |
| `07_utonia_features.png` | 本次 Utonia 特征 |
| 同名 `_map.png` / `.json` | RViz 画布 / 相机与点数状态 |
| `three_representations_demo.mp4` | 实际 RViz 录屏，含回放和特征视图 |
| `academic_height.pcd` | 原 XYZ + Batlow RGB |
| `academic_semantic.pcd` | 原 XYZ + 新分类 RGB + 原 label |
| `utonia_features.pcd` | 原 XYZ + 特征 RGB，无类别标签 |
| `utonia_report.json` | 版本、规模、配置和资源记录 |
| `map_integrity.json` / `validation.json` | 坐标/标签/特征映射及截图视频检查 |

关闭当前演示窗口后，可以重新截图录屏：

```bash
bash scripts/three_view_demo.sh --capture
bash scripts/three_view_demo.sh --record
env -u PYTHONPATH -u LD_LIBRARY_PATH \
  .runtime/venv/bin/python scripts/package_academic_upgrade.py
```

自动截图/录屏期间画布暂时忽略鼠标以固定相机，普通启动保持交互。PCD 已逐点验证 XYZ 完全相等、原标签来源一致及特征回映射符合体素成员关系。这些不是语义或 SLAM 精度评估。
