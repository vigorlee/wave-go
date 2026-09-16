> 本文记录几何规则基线；真实预训练模型接入请看 [README_ANYTHING_CN.md](README_ANYTHING_CN.md)。

# Go2-W 实时密集点云与语义避障观察

在楼梯、坡道、平地圆柱及动态代理三类场景中，同屏显示 UE、MuJoCo、Cosmos 和 RViz。新增观察节点不发布速度或导航目标。

## 本次采集结果

640×480 版本结果目录：`artifacts/dense_depth_20260911_take4`。本轮完整导航 **18/18 通过**，60.59 m、332.72 s；35 组快照及六组图片、原始 NPZ 已保存。运行过程中 100 帧窗口实测约 28 万有效点/帧、深度 5 Hz、语义显示约 4.3 Hz；完整运行统计另见该目录 `POINTCLOUD_QA.json`。先前 320×240 接入轮次 `dense_depth_20260911_take3` 已通过完整 18 阶段导航验收。

![楼梯点云和语义](artifacts/dense_depth_20260911_take4/pointcloud_figures/stairs_up.png)

[坡道结果](artifacts/dense_depth_20260911_take4/pointcloud_figures/ramp_up.png) · [动态避障结果](artifacts/dense_depth_20260911_take4/pointcloud_figures/dynamic_crossing.png)

![实时点云实录片段（回放）](artifacts/dense_depth_20260911_take4/pointcloud_figures/live_dynamic_clip.gif)

## 当前显示方案

默认显示改为 **UE 深度传感器 → 实时三维点云 → 几何分类与时序跟踪**。配置为 640×480、90° 水平视场、5 Hz；这些是设定值，实际帧率见 `/go2w/dense/status`。这不是更换了实体雷达，也不是 Ouster/Velodyne 仿真驱动。当前 UE 包仅提供 MID360 雷达配置；原 MID360 继续供导航使用。

深度图每个有效像素反投影成一个点，不插值补点，不从楼梯模型生成显示表面。深度小于 0.15 m、大于 8 m 和无效值丢弃。当前 UE 实测输出 `32FC1` 轴向深度（米），即使 `cloudmode=true` 仍是 Image，且 width/height/step 为零；适配器从启动时的有效配置读取分辨率并严格检查数据长度。

传感器安装参数在现有工程中是 UE 厘米；适配器转换成米，并转换 UE 右向 Y 为 ROS 左向 Y。深度像素先转到机身坐标，再使用 `/odom/mujoco_odom` 完整四元数与高度变换到 `odom`。位姿与深度时间差超过 0.2 秒则丢弃。**导航 TF 使用平面位姿，不能直接用它重建上下楼时的三维点云。**

## 一键运行

入口会重启本项目的仿真导航栈，并执行已批准的 18 阶段路线：

```bash
cd /home/unitree/Go2w+cosmos3Nav/matrix_go2w_lcm_demo
OUT="$PWD/artifacts/dense_depth_$(date +%Y%m%d_%H%M%S)"
GO2W_KEEP_LIVE=1 COSMOS_VLN_ARTIFACT_DIR="$OUT" bash scripts/record_go2w_dense_demo.sh
```

成功后保留实时窗口及观察节点（`go2w-dense-observers.service`）。采集器正常结束并写出采集统计。省略 `GO2W_KEEP_LIVE=1` 则结束时关闭。停止：

```bash
bash scripts/stop_go2w_dense_demo.sh
```

入口将分辨率和频率通过 systemd 环境传入 UE，避免被启动默认值 320×240、2 Hz 覆盖。有效传感器文件是 `matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json`。

## 信息来源与算法

| 话题 | 内容与来源 |
| --- | --- |
| `/go2w/depth/image_raw` | UE 深度 **Image**，不是 PointCloud2 |
| `/go2w/dense/points_world` | 深度像素反投影的密集 XYZ，完整三维位姿对齐 |
| `/go2w/dense/status` | 实际分辨率、有效点数、帧率、变换耗时与位姿时间差 |
| `/go2w/perception/semantic_points` | 密集观测点的算法类别与 RGB |
| `/go2w/perception/markers` | 聚类包围盒、ID、速度、距离、运动箭头与实时状态 |
| `/go2w/perception/status` | 类别计数、跟踪对象、处理时间；`uses_scene_prior=false` |
| `/livox/lidar_raw` | UE MID360 实际模拟射线点 |
| `/livox/lidar` | 原始雷达 + 静态圆柱模型点 + 动态 qpos 代理点，供导航使用 |
| `/go2w/semantic/points` | 原始雷达点的场景先验几何标签；默认隐藏，独立用于核对 |
| `/go2w/semantic/labels` | `[prior]` 场景文字，默认隐藏 |
| `/go2w/world_model/status` | MuJoCo 实际动态代理状态与避让状态 |
| `/cosmos_vln/route_command` | Cosmos 高层路线选择，非逐点语义分割 |

在线算法先以 7.5 cm 体素提取代表点，计算 18 邻域法向量、曲率和局部高度变化；提取紧凑障碍聚类，跨帧关联 ID 并估计速度。再将代表点的类别映射回原始有效深度点，保留密集轮廓。地面/平台为绿，楼梯候选蓝，坡面候选紫，墙体灰，障碍簇橙，运动簇粉。

**这是几何启发式分类与时序跟踪，不是训练出的语义神经网络，也不识别人类身份。** 楼梯/坡道是候选类别；墙角、遮挡和稀疏观测可能误分类、使 ID 切换或产生速度误差。动态场景使用移动圆柱代理。场景先验标签和模型融合点与算法结果分开，不冒充传感器观测或识别结果。

RViz 默认显示实时算法点云、包围盒和路线，隐藏 TF、二维扫描线、模型代理和先验文字，减少遮挡。可在 Displays 中切换原始雷达/先验标签作对照；不要将叠加点误认为更高传感器密度。

## 图片、原始数据与验收

程序自动保存上楼、下楼、平地绕柱、上坡、下坡、动态横穿区关键帧，并导出四联图：相机、原始雷达高度、在线算法语义、导航融合俯视图。

```bash
/usr/bin/python3 scripts/render_go2w_semantic_evidence.py "$OUT/pointcloud"
```

每个关键帧目录包含：

- `camera.jpg`、`clouds.npz`：相机、实际雷达点及独立先验标签、导航融合点、轨迹。
- `dense_depth.npz`：单帧原始深度反投影点；只保存新鲜数据。
- `algorithm.npz`：在线算法点与类别；只保存 0.8 秒内数据。
- `metadata.json`：各数据时间戳、年龄、实际位姿、来源、任务及跟踪状态。
- 上层 `dense_status.jsonl`、`perception.jsonl`：逐帧运行指标；`capture_summary.json`：采集统计。

雷达静态关键帧最多累积 1.5 秒；运动点可能拖影，动态先验关联仅使用当前代理状态。相机和算法与雷达快照是近时采集，非硬件同步。生成图和 PLY 位于输出目录的 `pointcloud_figures`，以实际文件为准。

完整导航是否成功以本次 `result.json` 的状态和 `acceptance_passed` 为准，不能以有图片替代通过验收。9 月 10 日成功版本完成 18/18 阶段；当时未保存原始点云，本指南的点云均为后续新采集。9 月 11 日 take1 存在变换错误，不作结果；take2 三类场景已采集，但最终导航丢失里程计，不算完整通过。

## 本地验证

```bash
source /opt/ros/humble/setup.bash
source genisom_roamerx_open/install/setup.bash
/usr/bin/python3 -m unittest discover -s tests -p 'test_dense_depth.py'
/usr/bin/python3 -m unittest discover -s tests -p 'test_live_perception.py'
/usr/bin/python3 -m unittest discover -s tests -p 'test_semantic_geometry.py'
```

测试覆盖深度投影方向、无效像素不补全、平面与斜面几何分类、静止与移动圆柱跟踪、场景先验标签隔离。它们不替代运行时导航验收与显示检查。
