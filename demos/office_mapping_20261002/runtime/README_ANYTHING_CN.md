# Go2-W：Depth Anything V2 + Grounded SAM 2 实时感知

> 当前入口是 [Wavegov1](README_WAVEGOV1_CN.md)：YardWorld、MID360、`draw_points=false`、`random_scan=false`。下方 OfficeWorld 绿色显示与长路线记录属于历史实验。

这是用户要求的**真实开源预训练模型版本**。之前的几何规则着色是独立基线，不是 SAM，也不是学习式语义分割。

## 实际加载的模型

| 模型 | 官方公开权重 | 本项目作用 |
| --- | --- | --- |
| Depth Anything V2 Small | `depth-anything/Depth-Anything-V2-Small-hf` | 从 RGB 预测相对逆深度，显示深度层次 |
| Grounding DINO Tiny | `IDEA-Research/grounding-dino-tiny` | 按文本类别提示输出目标框、预测类别和分数 |
| SAM 2.1 Hiera Tiny | `facebook/sam2.1-hiera-tiny` | 以 DINO 框为提示生成精细掩膜 |

代码资料：[Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2)、[Grounded SAM 2](https://github.com/IDEA-Research/Grounded-SAM-2)、[SAM 2](https://github.com/facebookresearch/sam2)。使用 Transformers 实现加载公开权重，未使用付费推理 API。权重目录 `models/anything`，具体 revision 和 SHA-256 见 `models/anything/manifest.json`。

GOAT / Go To Anything 是目标搜索与导航系统，支持语言、图像等目标条件；资料见[官方项目](https://theophilegervet.github.io/projects/goat/)。本轮没有移植 GOAT 的探索策略，也没有把现有 Nav2 改名为 GOAT。这里接入的是上述深度、开放类别检测与分割模型，现有已验证的导航负责移动。

## 实时链路

```text
UE RGB → Depth Anything V2 → 相对深度可视化
      └→ Grounding DINO → SAM 2 → 预测类别 + 像素掩膜
UE 米制深度 + 同步 RGB 掩膜 + 完整三维机身位姿 → 辅助米制语义点云
MID360 原始测量点 + 邻近语义关联 → MID360 语义点云 → RViz
```

界面四个面板分别显示原图、Depth Anything、Grounded SAM 叠加和语义掩膜。所有类别来自模型预测。输入提示是 `stairs. ramp. cylinder. person. wall. floor.`；这是类别词汇，不是场景坐标或真值标签。

**Depth Anything 的相对深度不能直接当作米。** 主 RViz 点云距离来自 MID360 原始测量，类别通过辅助深度点云与 DINO + SAM 掩膜关联；辅助点云距离来自 UE 深度传感器。模型相对深度保存在 `relative_depth`，不替换测量距离。新增感知节点不发布速度、目标或导航代价地图。

RGB 和深度传感器设置为相同安装位置和朝向。依据各自分辨率和水平 FOV 投影，保留共同视场；RGB/深度差超过 0.15 s 或深度/位姿差超过 0.1 s 时不发布这一帧米制语义点云。深度与 RGB 不同安装位置时也拒绝直接投影。UE 深度 Image 缺失的尺寸从有效配置读取。

## 本地运行

```bash
cd /home/unitree/Go2w+cosmos3Nav/matrix_go2w_lcm_demo
OUT="$PWD/artifacts/anything_$(date +%Y%m%d_%H%M%S)"
GO2W_KEEP_LIVE=1 COSMOS_VLN_ARTIFACT_DIR="$OUT" bash scripts/record_go2w_anything_demo.sh
```

入口重启本项目仿真，执行既有 18 阶段路线并录制；模型在 `go2w-anything-models.service` 中独立运行。保留运行开关使结束后仍能观察实时输出。原来的几何分类节点在此模式下关闭，RViz 使用 `/go2w/anything/mid360_points`（MID360 实测几何）。

模型图像窗口可单独打开：

```bash
source /opt/ros/humble/setup.bash
source genisom_roamerx_open/install/setup.bash
export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp
/usr/bin/python3 scripts/go2w_anything_image_view.py
```

停止：

```bash
systemctl --user stop go2w-anything-models.service
bash scripts/stop_go2w_dense_demo.sh
```

首次安装使用独立 `.venv-anything`，继承本机 `unitree_lerobot` 的 PyTorch 2.7.1 + CUDA 12.8；安装 Transformers 4.57.6。没有升级原导航/控制环境。重新下载权重时使用 manifest 中的 revision：

```bash
HF_HUB_DISABLE_XET=1 hf download depth-anything/Depth-Anything-V2-Small-hf --revision 5426e4f0f36572d16453bbda7a8389317b1bef99 --exclude '*.bin' --local-dir models/anything/depth-anything-v2-small
HF_HUB_DISABLE_XET=1 hf download IDEA-Research/grounding-dino-tiny --revision a2bb814dd30d776dcf7e30523b00659f4f141c71 --exclude '*.bin' --local-dir models/anything/grounding-dino-tiny
HF_HUB_DISABLE_XET=1 hf download facebook/sam2.1-hiera-tiny --revision de431c4043854a71d8101e17995dfe596bf101a5 --exclude '*.pt' --local-dir models/anything/sam2.1-hiera-tiny
```

## 输出与限制

- `/go2w/anything/annotated_image`：实际模型四面板画面。
- `/go2w/anything/mid360_points`：MID360 原始实测点，关联 DINO/SAM2 标签，主显示话题。
- `/go2w/anything/semantic_points`：深度传感器测量距离 + 学习式语义的辅助点云。
- `/go2w/anything/status`：各模型推理耗时、时间差、类别、检测分数和来源。
- 运行目录 `anything/frames.jsonl`：逐帧指标；各场景文件夹保存 `models.png`、原图、`predictions.npz`、`semantic_cloud.npz` 和元数据。
- 运行目录 `pointcloud/` 为原始雷达/深度基线记录，不能与 `anything/` 的学习式结果混称。

SAM 本身不输出类别名；类别由 Grounding DINO 给出。SAM 的预测 IoU 和检测分数均不是在此仿真数据上测出的准确率。模型会把白色楼梯误判为坡面、漏检远处圆柱，也可能产生假阳性；若同一检测文本同时含 stairs/ramp，标为歧义类别。显示层按检测分数处理掩膜重叠，不会用场景真值更改模型标签。没有做针对本场景的训练，也未宣称未知环境导航成功率。

## 验证

```bash
source /opt/ros/humble/setup.bash
source genisom_roamerx_open/install/setup.bash
.venv-anything/bin/python -m unittest discover -s tests
```

投影测试检查测量距离和标签对应关系、不同安装位姿时拒绝投影、无效深度不生成点。模型效果由实际保存的图片、原始输出和在线运行指标验证；导航验收单独看 `result.json`，不能代替语义准确率评估。


## 2026-09-11：MID360 与检测整理

保留 `sensor_type: mid360`、10 Hz 原始话题 `/livox/lidar_raw`。当前保存版设置 `draw_points: false`、`random_scan: false`，关闭 UE 绿色扫描点，保留 RViz 实测点云。主显示不做插点补密，也不以深度点云冒充激光雷达。雷达回调独立于模型推理，选择时间差不超过 0.1 秒的里程计做完整三维变换；将不超过 0.6 秒的语义结果以 0.12 米最近邻半径关联到当前测量。未匹配、过期或共同视野外的点保持灰色 unknown。RViz `Decay Time: 0` 保留最新一帧，直到下一帧替换，避免此前 0.25 秒寿命短于模型间隔导致的空白闪烁。几何刷新跟随雷达，语义更新仍受模型推理速度限制。

检测展示采用分类阈值（person/cylinder 0.48）、框 IoU > 0.55 去重、连续两帧确认、SAM 预测质量 >= 0.65。同一文本同时含 wall/cylinder 等不相容类别时拒绝赋类；stairs/ramp 仍显示 ambiguous。人体框形状约束可能漏掉坐姿、截断或被遮挡的人，不能作为唯一避障输入。没有新鲜检测时不保留旧框，原始候选及过滤原因保存在 `detection_filter_audit`。

Nav2 局部/全局膨胀半径分别改为 0.45/0.40 米。旧圆柱代理穿越策略的等待范围改为 2.20 米（迟滞 2.50 米），预测双方运动的最近中心距离 1.05 米；这些参数不等于实测人体表面净空。仍须以整段仿真和真实人物位置验证，不应把代理测试当作人体避障验证。

## 原生人物与 Super-LIO 的验证状态

MATRiX 文档和课程第 17 页描述了 `human1`、`trajectory` 和 `avoid` 配置。现有 YardWorld 配置中的三条动态轨迹仍由 MuJoCo 圆柱代理驱动。测试了 CustomWorld 和正/负 Y 人物位置配置，但当前打包 UE 的保存画面尚未确认加载出对应行走人物，因此**尚未完成原生行人替换**。原路线保留，不把隐藏圆柱或 JSON 时间推算伪称为已同步人物。探测记录位于 `artifacts/native_people_revision/`。

[Super-LIO 官方仓库](https://github.com/Liansheng-Wang/Super-LIO) ROS 2 源码已下载到 `third_party/Super-LIO`，目前未接管定位。实际采样 40 帧 `/livox/lidar_raw`：逐点 `timestamp` 全为 0，约 801 条 IMU 同时可用，详情见 `artifacts/native_people_revision/sensor_audit.json`。其标准 Hesai 输入使用逐点绝对时间，其 Livox 分支要求 CustomMsg；现有零值 PointCloud2 不能直接视为有效逐点时间输入。后续需要确认仿真是否为整帧瞬时扫描、外参和时基，再实现适配及独立轨迹对比；不人工编造逐点扫描时间。

### 本轮实测记录

- `artifacts/mid360_semantics_20260911/`：MID360 语义点云在线运行，完成 14/18 阶段；远端走廊 `continuous_route_no_progress`，完整路线验收失败。`ANYTHING_QA.json` 保留 false，不用前次成功代替本次验收。
- `artifacts/native_people_revision/detection_comparison/`：同一批历史 RGB 的离线复测，四幅图显示对象数从 6/4/7/7 减为 2/3/5/3。只说明显示过滤减少，不是语义准确率评估。
- 本轮距离审计发现原上坡经过圆柱时，配置的 0.60×0.30 m 机身轮廓最小间隙约 0.062 m。坡道入口/坡顶/下坡入口的目标 x 改为 0.70/0.78/0.78 m，坡道最大速度降为 0.35 m/s，最低前进速度改为 0，允许停下。
- `artifacts/native_people_revision/ramp_clearance_probe/`：改后坡道独立复测成功，实际记录 4937 个位姿，坡道上/下圆柱最小轮廓间隙分别 0.511/0.389 m。起点约 `(0.743,13.148)`，终点约 `(0.841,18.027)`；这是从坡道入口开始的专项验证，不覆盖前面绕桩到坡道的接近过程，也不是整条路线通过。详见 `probe_result.json` 和 `CLEARANCE_QA.json`。
- `artifacts/native_people_revision/tests.log`：54 项测试通过，包括模型过滤、MID360 标签关联及 36 组穿越相位的代理避让测试（中心距离下限 0.95 m）。

主窗口截图：

![MID360 实测点云与 DINO/SAM2 语义叠加](artifacts/native_people_revision/mid360_semantic_ramp.png)

当前在线 MATRiX 官方仓库已发布与本地旧运行包不同的版本。上述 native human1 及零值点时间的结论仅针对本地实测；本轮没有将整个运行时升级，也没有宣称新版本同样缺少这些能力。


## 2026-09-12 完整路线复测

本次完成 1/18 阶段，下楼过程中 MuJoCo 退出（监控记录 status=0），随后里程计中断；验收未通过，坡道衔接和远端走廊本次未覆盖。详细结果见 [复测记录](artifacts/mid360_retest_20260912/RETEST_CN.md)。

## 2026-09-12：OfficeWorld 绿色扫描点

启动：

```bash
cd /home/unitree/Go2w+cosmos3Nav/matrix_go2w_lcm_demo
bash scripts/start_go2w_office_people.sh
```

OfficeWorld v0.1.2 已下载并校验，使用办公室专用 MuJoCo 碰撞模型及 `g1_office.yaml` 导航地图。该配置没有旧的动态圆柱代理。原生行人已在 CrowdWorld 观察到，但 OfficeWorld 的人物加载与行人避障尚未通过验证；启动脚本名称不代表这项验收已完成。

本次静止观察 12 秒：原始 MID360 10.01 Hz、语义 MID360 10.05 Hz，中位每帧 18,961 点；最大到达间隔分别为 0.200 / 0.201 秒。RViz 保留最新帧可避免等待新帧期间清空，原生绿色随机扫描本身仍有点位变化。此项验证不代表整条路线或行人避障通过。

- [发布频率原始记录](artifacts/office_green_mid360/refresh_audit.json)
- [测试记录](artifacts/office_green_mid360/tests.log)

![OfficeWorld 原生 MID360 绿色扫描点](artifacts/office_green_mid360/ue_green_scan.png)

原生调试点也会进入 RGB 图像，因此当前绿色显示模式不能用于声称检测算法在干净图像上的准确率。做检测质量评估时，使用下列入口关闭 UE 调试点；RViz 的 MID360 语义点云仍会发布：

```bash
MATRIX_LIDAR_DRAW_POINTS=0 bash scripts/start_go2w_office_people.sh
```

此 OfficeWorld 配置属于历史实验；当前 Wavegov1 默认关闭绿色显示。54 项单元测试通过；[12 秒实时扫描录像](artifacts/office_green_mid360/ue_green_scan.mp4) 展示静止机器人周围连续刷新的扫描点。
