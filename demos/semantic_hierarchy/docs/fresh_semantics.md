# 新采集校正地图：同一坐标下的三种表示

新记录 `fresh_20260919_02` 已重新处理。旧地图、旧配色和旧 PCD 保留；新结果独立存于 `outputs/fresh_semantic_20260919/`。

## 启动原生演示

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy
bash scripts/show_fresh_semantics.sh
```

窗口内可切换 **Graph、语义颜色 + Box、纯点云、融合叠加、HOV-SG 分层、Utonia 特征**。点击 **RGB-D 三模型** 查看新录制图像上的 Depth Anything V2、Grounding DINO 和 SAM2 实际输出。鼠标旋转、滚轮缩放；底部可切换总览／侧视／俯视。

默认保留 FAST-LIO 风格的 RViz Z 高度着色。Utonia 多彩特征仅在选择该页时显示。需要之前的蓝金或 Batlow 风格：

```bash
bash scripts/show_fresh_semantics.sh --style paper
bash scripts/show_fresh_semantics.sh --style batlow
```

旧语义地图入口仍是 `bash scripts/three_view_demo.sh --style fastlio --mode raw`；新旧几何比较入口是 `bash scripts/show_level_comparison.sh`。

新演示使用独立 ROS 1 master `11328`，不驱动机器狗。关闭窗口清理本启动器拥有的节点。新旧数据通过 `SCENE_DATASET_ROOT` 分开，旧入口默认数据不变。

## 实际运行范围

| 方法／表示 | 本次实际处理 | 输出 |
|---|---|---|
| MID-360 + IMU / LIO 几何 | 使用已验收的结构校正地图，显示 ROI 中 1,858,378 个原测量点，体素 4 cm；所有表示的 XYZ 完全一致 | 纯点云、高度着色 PCD |
| Depth Anything V2 Small | 对新记录的 378 帧 RGB 重新推理 | 相对逆深度图；不当作米制深度 |
| Grounding DINO Tiny + SAM2.1 Hiera Tiny | 新 RGB 检测和掩膜；同步的 UE 米制深度投影，按校正 LIO 位姿关联到同一地图 | 261,143 个多帧确认的语义点、物体 Box |
| Spark-DSG 1.1.3 | 实际调用官方库建图、序列化和读回验证 | 26 节点、26 条关系：1 容器、2 楼层、5 区域、8 物体簇、10 墙面候选 |
| HOV-SG | 同前版，运行固定版本的原版 `Graph.segment_floors`，得到 2 个区间 | 楼层颜色视图；不是完整 HOV-SG 物体／房间流程 |
| Utonia | 官方预训练编码器、官方 PCA 函数；237,854 个 12 cm 采样输入点，1,224 维特征 | 特征颜色映射回全部显示点；颜色不代表语义类别 |

模型确认类别为楼梯、坡道、圆柱、墙面、地面；**本记录没有可靠行人检测**。Box 是类别内空间聚类，可能分裂或合并物体，不是人工真值实例。

几何地图包含已知 `0 / 3.215 / 5.51 m` 场景高度的结构校正先验；本次语义处理不再次改动坐标。Graph 楼层使用 `0 / 5.51 m` 先验并要求实测水平面支持。HOV-SG 原版模块独立输出的高度区间为 `[-0.729, 3.888]`、`[4.138, 7.329] m`，不是地面高度或独立测绘精度结果。

这里没有运行完整 Hydra、LiDAR S-Graphs、Elevator-LIO 或 Nav2 导航流程。时间轴是扫描可见性回放，叠加最终离线语义，不是在线模型推理测速。

## 投影和核验

- 从新 ROS 2 记录提取 378 对 RGB-D，最大时间差约 15 微秒。深度消息宽高字段为 0，使用录制时 `sensor_config.json` 的 640×480，并严格校验每帧字节数和 32FC1 编码。
- RGB 和深度同一安装位姿，按各自水平 FOV 与画幅对齐；UE 外参转换到 ROS 坐标后，扣除机身到 IMU 原点的固定偏移；校正 LIO 位姿按时间插值，无越界外推。
- SAM 掩膜边缘腐蚀 2 像素；深度 0.15–8 m；地图关联半径 0.12 m；每帧每点每类最多投 1 票。至少 2 帧支持、获胜类别占比至少 60% 才着色；未分类区域保持未知。
- 物体类别来自新模型输出。Graph 的楼层、区域、墙面属于几何候选，证据字段与视觉类别分开保存。不存在旧地图标签直接迁移。
- 原始地图坐标逐点精确核验；Spark-DSG 保存后重新读回核验；原先备份清单的 11 个文件 SHA-256 全部相同。

## 输出目录

相对于 `outputs/fresh_semantic_20260919/`：

| 路径 | 内容 |
|---|---|
| `outputs/fastlio_comparison/fresh_overview_comparison.png` | 同视角三种表示总览 |
| `outputs/fastlio_comparison/fresh_side_comparison.png` | 同视角跨层侧视 |
| `outputs/fastlio_comparison/fresh_top_comparison.png` | 同视角俯视 |
| `outputs/fastlio_comparison/fastlio/` | 各模式独立截图、截图时的相机与显示状态 |
| `outputs/reference_demo/dense_measured_map.pcd` | 原测量坐标的显示区域点云 |
| `outputs/reference_demo/dense_model_semantic.pcd` | 同一测量坐标 + 视觉模型类别、RGB 和证据字段 |
| `outputs/academic_upgrade/academic_semantic.pcd` | 统一类别配色 PCD |
| `outputs/academic_upgrade/utonia_features.pcd` | 特征 PCA 颜色 PCD |
| `outputs/scene_graph.json` / `outputs/spark_dsg.json` | 可审查 Graph 与官方 Spark-DSG 文件 |
| `outputs/rgbd_models/frames/` | 每帧语义掩膜、相对深度和模型检测信息 |
| `outputs/rgbd_models/panels/` | RGB、相对深度、检测与语义掩膜四格图片 |
| `outputs/validation.json` / `outputs/old_files_integrity.json` | 新输出一致性、旧文件 SHA 校验 |
| `thesis_figures_cn/` | 按北京理工大学 2026 年规范制作的中文论文图副本；图题在图下方居中，原英文图保留 |
| `outputs/rgbd_models/panels_cn/` | Grounding DINO + SAM2 原生目标框的中文显示副本；保留原始模型文本和类别 ID，不新增人工识别 |
| `outputs/rgbd_models/label_translation_cn.json` | 原始模型类别到中文显示名称的固定映射；仅改变显示语言 |
| `outputs/two_floor_maps/` | 二层地图的多方法保留索引：FAST-LIO、DINO+SAM2、Graph/Spark-DSG、HOV-SG、Utonia 与结构校正 |

完整新校正 PCD 仍为 `outputs/level_fix/fresh_20260919_02/structural_calibrated/office_two_floors_structural_4cm.pcd`，共 2,389,801 点；ROI 之外的点没有删除。

## 重新计算与截图

执行下列命令会重新计算新 profile 的输出，不覆盖旧记录：

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy
export SCENE_DATASET_ROOT="$PWD/outputs/fresh_semantic_20260919"
env -u PYTHONPATH -u LD_LIBRARY_PATH \
  /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/.venv-anything/bin/python \
  scripts/infer_fresh_rgbd.py --dataset "$SCENE_DATASET_ROOT"
env -u PYTHONPATH -u LD_LIBRARY_PATH OPENBLAS_NUM_THREADS=4 .runtime/venv/bin/python \
  scripts/build_scene.py --output "$SCENE_DATASET_ROOT/outputs" \
  --geometry "$SCENE_DATASET_ROOT/data/graph_geometry.pcd" \
  --semantics "$SCENE_DATASET_ROOT/data/semantic_map.npz" \
  --trajectory "$SCENE_DATASET_ROOT/data/lio_trajectory.npy" \
  --source-recording fresh_20260919_02 --z-max 9.2 --floor-heights 0 5.51
env -u PYTHONPATH -u LD_LIBRARY_PATH .runtime/venv/bin/python scripts/run_hovsg_floor_probe.py
env -u PYTHONPATH -u LD_LIBRARY_PATH .runtime/utonia_env/bin/python scripts/run_utonia_features.py
env -u PYTHONPATH -u LD_LIBRARY_PATH .runtime/venv/bin/python scripts/finalize_fresh_semantics.py
env -u PYTHONPATH -u LD_LIBRARY_PATH .runtime/venv/bin/python scripts/export_academic_maps.py
bash scripts/show_fresh_semantics.sh --capture-compare
env -u PYTHONPATH -u LD_LIBRARY_PATH .runtime/venv/bin/python scripts/package_fresh_comparisons.py
```

已缓存的逐帧模型输出会直接复用；首次没有缓存时才重新运行模型。截图使用原生 RViz，不录制视频。截取完整 2040×1140 窗口建议显示器分辨率至少 2200×1250。

论文图预览使用 Noto Serif CJK 的宋体风格字形；粘贴到论文后，将图题设置为宋体五号、居中、单倍行距，图序与图题之间空一格，段前和段后各 6 磅。图题编号是图像副本中的占位编号，按论文所在章节重新编号即可。
