# 单地图语义融合与建图回放

## 当前默认：语义、点云和轨迹在同一地图

按照后续要求，默认窗口已改成**单一三维地图**，不再左右分屏。点云、类别颜色、物体簇标签与 LIO 轨迹均处于 `lio_map`，没有可视化平移、复制地图或伪造配准。

语义来自已有 Grounding DINO + SAM2／米制深度投影／时序语义体素融合输出。本次保持原类别 ID 和原应用配色：楼梯蓝、坡道紫、圆柱橙、行人粉、墙面灰蓝、地面绿；未知点灰色。它们通过 0.12 m 邻域关联到更密的记录点云，**不再混入新写的法向、楼层、墙面规则标签**。

`model_semantics_provenance.json` 记录源 SHA-256、完整类别与配色，`dense_model_semantic.pcd` 是当前单地图显示对应的导出。共有 503,170 点带语义类别，其余保持未知。模型的历史误分类也被保留，不能把颜色完整当作语义准确。7 个物体框是预测类别的可视化聚类，不是 Hydra 的原生实例输出。

**“保持模型类别一致”不等于“已跑通完整 Hydra／S-Graphs”。**2026-09-19 对 Hydra 做了独立 CMake 构建检查，缺少 `config_utilities` 时停止；完整上游建图仍未运行。用户尚未指定“开源算法一致”具体指哪一套原生结果，当前明确使用已有 DINO＋SAM2 语义，不冒充其他算法。

默认启动：`bash scripts/reference_demo.sh`。仅回看之前双视图时使用 `bash scripts/reference_demo.sh --layout dual`。

截图：`outputs/reference_demo/01_single_semantic.png`、`02_single_stairs.png`、`03_single_top.png`。

下面保留参考来源、数据累积方式与双视图功能记录；双视图中的几何候选不是默认单地图模型语义层。

本次参考是用户提供的[智元远征 A3 Ultra 空间智能视频号页面](https://weixin.qq.com/sph/ASrpWPQXZL)和截图。页面标题及文字可通过浏览器读取，完整视频需要微信扫码观看。官方[规控导航文档](https://open.agibot.com/docs/aimdk/a3-ultra/v3_2/dev_guide/07-09-navigation)公开了产品导航调用接口；本次没有取得其演示原始楼宇地图、全部类别表或相同建图算法源码。

**当前复现的是显示方式和本地记录的建图回放，不是视频中圆形楼宇的一比一重建，也不是智元软件的复现。**原 OfficeWorld 几何保持不变，不凭截图编造楼层、家具、房间名称、行人或电梯。

## 实际新增

- 白色背景，左侧半透明语义实例包围框，右侧高度着色点云，两侧使用同一度量坐标，鼠标相机联动。
- 27 个有观测点支持的标注：3 个楼梯簇、1 个坡道簇、3 个圆柱簇、13 个墙面候选、2 个候选楼层、5 个观测区域。簇不保证等于物理物体数量。
- 实例编号、英文图内标签、中文列表、点击定位与来源说明。`?` 为视觉预测，`*` 为几何候选。
- 按真实扫描时间回放点云形成过程，播放／暂停／拖动，1×／2×／4×／8×速度，以及原始 LIO 轨迹。
- 可选点云剖切：保留楼层附近 1.25 m 高度和楼梯／坡道区域，便于看到室内轨迹；仅影响显示，PCD 保留完整 ROI。
- 两个 RViz 渲染器独立进程，避免 ROS1 RViz 的 Ogre 全局材质重名问题；没有网页渲染器。

## 密度与来源

从 `office_rosbag_20260916_01/go2w_pointcloud_odom_noetic.bag` 的 `/wavego/slam/registered_points` 读取 1,735 帧，共 32,641,618 个输入返回点。显示 ROI 内以 4 cm 体素保留每格首次测量的原始 XYZ，得到 **1,647,372 点**。原演示同 ROI 为 184,512 点。

每个点保存真实首次观测时间。没有插值补点、复制点或移动点坐标，密度提高不代表定位精度提高；累积仍会保留原里程计漂移和传感器噪声。语义以 0.12 m 邻域关联旧版本最终语义地图，328,953 点获得类别，其余保持未知。未识别点不会因“全标注”的显示要求被强行分配类别。

时间轴上的语义是**最终离线标注叠加**，不是重跑 RGB-D 模型的在线推理结果。实例框只在该时刻已有相关测量点时出现，边界根据当时已观测的关联点计算；最终类别可能使用后续帧的信息，因此不能把这个回放当作在线语义精度实验。

## 一键查看

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy
bash scripts/reference_demo.sh
```

录制建图回放（原生窗口，4×回放，录完自动关闭，可重新正常启动）：

```bash
bash scripts/reference_demo.sh --record
```

## 从原记录重建

```bash
source /home/unitree/flycar/tools/setup_noetic.bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy
.runtime/venv/bin/python scripts/extract_mapping_replay.py \
  --bag /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_rosbag_20260916_01/go2w_pointcloud_odom_noetic.bag
```

运行 `.runtime/venv/bin/python scripts/prepare_native_semantics.py` 准备原类别映射；启动一次发布器，生成实例关联和带标签 PCD 后，可以检查：

```bash
source /home/unitree/flycar/tools/setup_noetic.bash
python scripts/validate_reference_demo.py
```

本机 `.runtime/venv` 与 Noetic 同为 Python 3.11。提取脚本用 ROS1 的 rosbag 加虚拟环境中的 scipy；查看脚本只需 Noetic 环境。

## 输出目录

全部在 `outputs/reference_demo/`：

- `semantic_mapping_demo.mp4`：实际 RViz 窗口的建图回放录像。
- `01_dual_map.png`、`02_stairs_dual.png`、`03_top_dual.png`：总览、侧视、俯视原生窗口截图。
- `dense_measured_map.pcd`：164.7 万个测量点。
- `dense_semantic_map.pcd`：同样 XYZ，附 RGB、label、evidence。
- `semantic_annotations.json`：27 个实例／结构候选的中英文标签、原始来源、边界、关联点数。
- `mapping_replay.npz`：逐点首次观测时间、类别、原语义点索引及每帧统计。
- `replay_report.json`、`validation.json`：来源、计数、限制和原始测量抽查。

若要与视频里建筑和全部语义逐一一致，仍需要其原始点云／RGB-D 数据或完整可读视频及类别表；仅凭一张截图不能恢复建筑背面、物体尺度和完整实例清单。
