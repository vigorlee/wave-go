# 两份实测点云：Z ≤ 5 m 裁切与几何特征图

`12` 与 `MT` 分别处理，原 ZIP、RGB PCD 和上一版去顶结果均保留。新裁切**沿原始 PCD 的 Z 坐标**，只保留有限值且 `Z ≤ 5.00 m` 的测量点；没有缩放、配准、插值或补点。两份扫描地面约在 `Z=-1.18 m`、`-1.24 m`，因此这里的 `5.00 m` 是坐标阈值，并非相对地面的五米高度。需要相对地面裁切时，应另设阈值，不能把两者混称。

| 数据 | 原始点 | 完整裁切 PCD | 显示点 | 超过阈值移除 | 楼梯候选 | 实测楼梯平台 | 紧凑障碍候选 | 坡道候选 | 垂直墙 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `12` | 28,071,719 | 20,821,040 | 1,200,501 | 7,250,679 | 5 处（其中二层 2 处） | 1 处 | 13 处 | 2 处 | 1 面 |
| `MT` | 12,789,345 | 9,846,349 | 1,166,355 | 2,942,996 | 4 处（其中二层 1 处） | 1 处 | 18 处 | 2 处 | 1 面 |

原始 `Z > 5 m` 的上部点不在新 PCD 中。低于阈值的水平楼板、平台或天花板可能仍存在；按这个用户指定的高度阈值，不能仅凭朝向把楼板和天花板断言成不同语义对象。

## 几何识别与颜色

先在显示点云中对每点的 32 个近邻估计法向量和局部平面曲率。**楼梯**使用 5 cm 水平表面高度图，在相邻剖面中寻找 12–21 cm 高、间距连续的重复踏步；只为至少四级且多条剖面重复支持的区域生成 Box。分别分析低层和二层楼梯：二层高度层由点云的水平面峰值（本次约 `Z=2.425 m`）定位，并在该层以下取每格最低踏面，避免高平台把踏步覆盖。`12` **右侧**下段原先按 9 级的剖面检测值限制了最高着色高度；新算法按局部实测踏面着色，又在已着色点的 1.5 cm 高度直方图中验证出 13 个水平高度峰、12 次上升。该段因此标为 **12 级观测台阶**，T5 的高度峰支持把另一端二楼台阶由 9 级修正为 11 级；原始点云和楼梯位置没有移动。右侧两段之间的黄色平台有 477 个近水平支持点、447 个着色点，10%–90% 高度跨度仅 3.9 cm。`MT` 在左侧完整记录一楼段与中间平台；`12` 左侧的二楼段仍有实测点，但左侧一楼段在单次扫描中较稀疏。

**垂直墙**在二层楼梯相邻的走廊区域，从竖直法向点 RANSAC 拟合一条 XY 平面线，并检查纵向覆盖、立面高度与残差。`12` 的平面残差约 2.5 cm、沿走廊覆盖约 43.6 m；`MT` 的残差约 2.2 cm、覆盖约 23.3 m。红色点和沿拟合平面绘制的窄框表示有数据支持的墙面，普通零散立面仍保持浅蓝灰色。**坡道**先提取倾斜法向点，再用 25 cm 网格连通并拟合平面；当前两份地图的四个坡道候选约为 19.7–20.4°，平面残差为 1.6–7.4 cm。紧凑三维障碍从原先只取前 6 个改为保留全部通过 18 cm 连通体、点数和尺寸门槛的候选（`12` 为 13 个、`MT` 为 18 个）。它们只标为“其他障碍”，不会凭形状猜成人或设备。Graph 仅连接近邻几何特征；楼梯与平台另有实测邻接边，但这些边都不代表可通行性已经验证。

| 颜色 | 类别 | 含义 |
|---|---|---|
| 绿色 | 地面 | 近水平、接近本次扫描主地面高度 |
| 蓝色 | 楼梯 | 重复踏步和支持的踏步／立板点 |
| 青蓝 | 二层楼梯 | 二层高度剖面中的重复踏步 |
| 黄色 | 楼梯平台 | 两段楼梯之间有密集实测点支持的近水平连接面 |
| 浅橄榄色 | 二楼楼层平台 | 上完二楼楼梯后、由水平测点支持的局部楼面；仅在拼接演示图中单列 |
| 紫色 | 坡道 | 连续斜面、倾斜平面拟合 |
| 红色 | 垂直墙 | 竖直法向和 RANSAC 平面支持的墙面 |
| 金色 | 水平结构 | 高于主地面的水平表面，可能为平台或楼板 |
| 浅蓝灰 | 立面结构 | 近竖直的墙面或结构面，不细分物体名称 |
| 橙色 | 其他障碍 | 紧凑的残余三维连通体，仅为候选 |
| 深灰 | 未分类 | 证据不足的点，保留其原始坐标 |

这套颜色是**几何候选分类**，并非 RGB 模型分割或标注真值。原 LAZ 的 `classification` 全为 0。箱体范围、踏步数、坡角、支持点数和拟合残差可在各自的 `feature_graph.json` 核对；逐点类别在 `class_ids.npy`。

## 直接打开原生交互窗口

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy
bash scripts/show_z5_features.sh 12
bash scripts/show_z5_features.sh mt
```

两个终端可同时运行，默认分别使用 ROS master `11333` 和 `11334`。各自窗口提供原始 RGB、全点特征颜色、Box、度量 Graph 和融合视图；可以旋转、缩放及切换总览、俯视、侧视、障碍细节、二层楼梯／墙视角。不带 `--capture` 时窗口保持交互。截图：

```bash
bash scripts/show_z5_features.sh 12 --capture
bash scripts/show_z5_features.sh mt --capture
```

各自输出目录保留纯点云、着色点云、Box、Graph 和融合图；同名 JSON 记录显示模式、视角及实际标签。`12` 的负 Y 端是**右侧**完整楼梯，`MT` 的正 Y 端是**左侧**完整楼梯；新截图文件按实际侧别命名，先前误命名的 `09_left_stairs_detail.png` 等旧截图仍保留但不作为侧别依据。`--capture` 完成后关闭窗口，不控制机器人。输入 PCD 不含时间和位姿序列，因此这是静态测量地图的后处理可视化，不是实时建图回放。

需要重算时：

```bash
python3 scripts/process_z5_features.py --name 12
python3 scripts/process_z5_features.py --name mt
python3 scripts/validate_z5_features.py --name 12
python3 scripts/validate_z5_features.py --name mt
```

处理脚本使用系统 Python 的 NumPy、SciPy；重复运行时可用 `--geometry-cache .runtime/z5_probe/12_all_normals.npz` 或对应 `mt` 缓存，它会先逐点核对 XYZ，再重用法向量。启动脚本自动准备本机 Noetic 环境和内嵌 RViz 的中文标签组件。验证脚本逐点比较新 PCD 与原 PCD 的筛选结果，检查坐标、RGB、强度完全一致，语义着色仅改 RGB；同时检查特征证据、最高 Z 和截图。修改前的着色与八张图另存于各输出目录的 `baseline_before_20260927/`。

输出分别在 `outputs/imported_12_20260926/z5_features/` 和 `outputs/imported_mt_20260926/z5_features/`：

- `full_z5_rgb.pcd`：全部裁切点，保留原 RGB 和强度；
- `display_z5_rgb.pcd`：显示用的原测量体素代表点；
- `display_z5_semantic.pcd`：与显示点逐点相同 XYZ、强度，仅 RGB 改为类别色；
- `feature_graph.json`、`feature_report.json`、`feature_heightmap.npz`、`class_ids.npy`：可复查证据和分析结果；
- `validation.json`、PNG 及同名 JSON：验证结果和原生窗口截图；旧截图也保留。

## 左右楼梯与二楼楼层平台同图显示

先运行上述 `12` 和 `MT` 的特征处理，再运行：

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy
python3 scripts/stitch_z5_left_stairs.py
bash scripts/show_z5_features.sh stitched
bash scripts/show_z5_features.sh stitched --capture
python3 scripts/validate_z5_stitched.py
```

窗口默认显示**完整融合地图**；需要放大二楼时点击“二楼长廊”或“二楼长廊全景”。左侧的 `T6` 一楼 12 级台阶 → `L2` 中间平台 → `T5` 二楼 11 级台阶 → `U2` 楼梯口平台，右侧为 `T3 → L1 → T4 → U1`；`U1/U2` 都接到 `C1` 二楼长廊。两侧均可在同一坐标系下切换点云、语义颜色、Box、Graph、融合图和局部视角。

`C1` 不是在两处楼梯口之间画一块合成楼板。算法沿实测垂直墙一侧追踪二楼近水平测点，在 50 cm 的纵向格中估计各自的局部高度，仅给距该高度 6.5 cm 内且有法向支持的点着色。共找到 **12,668 个实测长廊点**、约 **46.0 m** 纵向跨度，并按 19 个有测点的短段画轮廓；没有补点或填充空白。长廊沿线存在约 `Z=2.29 m` 与 `Z=2.43 m` 两个主要高度段，图中保留实测高度变化，不强行拉平成同一平面。`U1/U2` 是上层楼梯尽头的实测水平小平台，各有 768 个着色点；这些标签均为**几何候选**，不是模型语义真值或已验证的通行区域。

拼接仅对左侧一楼楼梯和中间平台做**局部平移配准**：用两扫描共有的左侧二楼踏步求得 `MT → 12` 平移约 `(0.537, -1.722, -0.018) m`，重叠二楼踏步的最近点中位距离约 `4.4 cm`、90% 分位约 `13.1 cm`。将 `MT` 的 12,949 个实测楼梯／平台点加入显示副本，另有 `12` 原有的 5,980 个近邻测点按 10 cm 对应关系重新着色；`12` 的其他点、两份原 PCD 和各自旧地图均未修改。这不是两次扫描的全局配准，也不证明拼接地图满足机器人定位或通行要求。Graph 中连接 `T6–L2–T5–U2` 和 `T3–L1–T4–U1` 的线表示几何邻接，不是导航路线。

拼接副本在 `outputs/imported_12_20260926/z5_stitched_12_mt/`，包含 `display_z5_rgb.pcd`、`display_z5_semantic.pcd`、`class_ids.npy`、`feature_graph.json`、`feature_report.json`、`validation.json`，以及 23 张截图和对应的显示状态 JSON。优先看 `20_upper_corridor.png`（二楼长廊融合）、`21_upper_corridor_semantic.png`（长廊轮廓）、`22_upper_corridor_graph.png`（长廊与楼梯口关系）、`23_upper_corridor_color.png`（只看点云颜色）、`09_left_stairs_detail.png`（左侧楼梯）、`15_right_stairs_detail.png`（右侧楼梯）和 `05_z5_fusion.png`（全图）。

## RelateAnything 试验与适用范围

用 2026 年开源的 [RelateAnything](https://github.com/Maelic/RelateAnything) `relsgg-vits16plus` 在 `12` 地图的最终 RViz 楼梯侧视截图上做了一次探索性关系推断。输入是截图中四个已有几何 Box（下段楼梯、平台、上段楼梯、墙）；输出最高分仅 **0.22**，还把平台和墙判成了不合理的“on top of”关系。原模型接受**图像和图像坐标的 Box/Mask**，输出物体间关系；它既不读取 PCD，也不独立检测楼梯。两份原始 ZIP 仅含 LAZ，没有可配准的相机帧与相机位姿。因此试验输入、区域图和结果分别保存在 `12/z5_features/relateanything_input.png`、`relateanything_regions.png`、`relateanything_probe.json`，**没有写入逐点类别、Graph 或导航地图**。几何标签依然是实测候选，不冒充模型语义。

## MATRiX 场景导入与上楼运动测试

直接把 PCD 作为 MuJoCo mesh 加载的复现实验返回 `no decoder found for mesh file ... .pcd`。点云本身也没有完整无孔洞的碰撞面，无法直接作为整栋楼的物理场景。在此限制下，`scripts/export_z5_stair_mujoco.py` 把每份地图已识别的相邻两段楼梯及平台的**实测高度剖面**转成独立的盒体碰撞代理。`12` 的 54 个 15 cm 采样格都有测量支持；高度量化到 15 cm 后中位误差 2.8 cm、最大误差 7.5 cm。该代理覆盖约 8.1 m 长、2 m 宽的局部走廊，不是整栋楼的重建。两份代理都经 MuJoCo XML 编译并可显示；`12` 的 MATRiX MuJoCo 进程实际运行了 20 秒以上。

用现有 Go2-W DreamWaQ 控制器在 `12` 代理上进行了四轮同参数前进运动测试（旧版固定到达阈值 `3.8 m`）：第一、二、四轮到达上层平台（终点机体 `z=3.89/3.90/3.91 m`，未触发安全停机），第三轮停在上段楼梯 `x=5.81 m、z=2.38 m` 附近（未跌倒，未到顶）。这是**3/4 的局部运动测试**，不是 Nav2 在原始 PCD 整栋建筑中的导航成功率。每轮的日志分别在 `trial1/`、`trial2/`、`trial3/` 及主目录的 `locomotion_trial.json`。`live_frames/` 的四阶段截图全部来自第四轮同一条真实 MATRiX MuJoCo 轨迹，并附 `frame_provenance.json` 坐标与时间。当前测试脚本改用实测平台顶面加 `0.25 m` 的机体到达阈值，后续试跑应与历史四轮分开统计。完整场景导航仍需把整栋点云转换为经过人工检查的碰撞网格、补齐缺测区，再配准 LiDAR/定位和 Nav2 多楼层地图。

`MT` 代理也完成 MuJoCo 编译和三轮实跑，但**尚未有一轮稳定到顶并停住**：`0.75 m/s` 第一轮停在上段楼梯 `x=5.81 m、z=2.23 m`；`1.0 m/s` 第二轮曾到 `x=9.13 m、z=3.795 m`，比当时固定的 `3.8 m` 到达阈值仅低 5 mm，控制器未停而越过平台末端，触发安全停机；按测得的代理顶面 `3.45 m` 加 `0.25 m` 机体余量修正阈值后，第三轮却在下段 `x=2.04 m、z=0.52 m` 停滞。三轮日志分别保存在 `MT/.../mujoco_stair_proxy/trial1/`、`trial2/` 和主目录。这个结果说明当前固定速度控制对实测楼梯剖面的通过性不稳定，不能用第二轮的瞬时最高点宣称导航成功。

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy
MUJOCO_GL=egl python3 scripts/export_z5_stair_mujoco.py --name 12
python3 scripts/test_z5_stair_proxy.py --name 12 --duration 55
python3 scripts/test_z5_stair_proxy.py --name mt --duration 58 --vx 1.0
```

两份扫描没有已知的全局共同世界坐标；上面的局部楼梯配准仅用于可视化，不应直接用于完整场景导航地图。
