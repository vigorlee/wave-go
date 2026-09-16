# Flycar PCD 处理与自适应膨胀迁移

已把 Flycar 的点云测宽、栅格占据和膨胀思路迁移成可运行的 Python 模块，应用到 Flycar 和 Go2-W 的真实 PCD，并接入交互网页及独立 RViz 预览。浏览器改变观察位置、朝向、速度后，算法重新计算，RViz 同步更新。输入为历史观测，未接管实时导航。

打开 http://127.0.0.1:8877/?dataset=go2w 。选择原始、固定、自适应层，切换宽窄观察位置，或在俯视图点击设置位置。它们是分析探针，不是已执行路线。网页三维颜色沿用已有语义标签；Flycar 没有语义标签，仅按高度着色。膨胀区域是计算出的地面投影，不能当作新增传感器点云。

## 源代码分析

源根目录：`/home/unitree/flycar/src/planning/lapp/planner/`。

| 文件与行号 | 实际行为 / 问题 | 本次处理 |
|---|---|---|
| `uav_simulator/map_generator/src/random_forest_sensing.cpp:74` | PCL 读取 PCD，但路径硬编码为 `/home/lll/flycar_sim/scene%d.pcd` | 读取明确指定的本机 PCD；支持 binary / ascii 的字段类型；检查点数、NaN |
| `TIE_navigation/plan_env/src/sdf_map.cpp:1047` | 每 5 帧处理一次 | 此次参数变化即计算，离线数据不冒充实时传感器 |
| `sdf_map.cpp:1082` | 5 cm VoxelGrid | 分离显示和分析分辨率；Flycar 2.5 cm 显示、5 cm 分析 |
| `sdf_map.cpp:1093` | `updateInflationAdaptive` 调用被注释 | 在 `analyze()` 中实际调用测宽与自适应策略 |
| `sdf_map.cpp:1114` | 膨胀循环仍遍历 latest_cloud，未使用刚得到的 downsampled_cloud | 使用分析点云构建占据栅格，通过欧氏距离场产生膨胀 |
| `sdf_map.cpp:1620` | 原始距离分档 0.1 / 0.3 / 0.5；0.52 的两个方向阈值相同，并未形成该处滞回区间；调用也被注释 | 连续余量 + 速度项 + 2 cm 的小幅增大滞回 |
| `sdf_map.cpp:1749` | 多朝向、6 个前向截面、左右步进测宽 | 保留此思路；KD-tree 加速；必须两侧都命中，至少 3 个有效截面才给宽度 |
| `sdf_map.cpp:1795` 附近 | min_width 初始为 1.5，之后只取 min，限制了宽度上限；单侧未命中也可能产生宽度 | 不人为封顶；单侧观测不足不返回伪造宽度、不复用陈旧缓存 |
| `sdf_map.cpp:1900` 附近 | getTurnDirection 开头直接 return 0，后续逻辑不可达 | 由页面朝向输入提供方向 |
| `sdf_map.cpp:1986` | 三个宽度区间全部给 0.1 m，自适应实际上恒定 | 根据实际宽度调节额外舒适余量，尺寸与制动下限始终保留 |
| `sdf_map.cpp:1123` | 硬过滤 z < 0 | 用明确的地面高度和相对高度切片，负 z 点不被一概删除 |

这不是原 C++ 的逐行复制，也没有宣称原项目已经实现有效的自适应。原项目、原 PCD、原 Go2-W 代码均未改动；本次可执行实现位于本目录。

## 算法与参数

`adaptive_core.py` 提供 PCD 读取、体素质心、占据栅格、距离场、双侧测宽、自适应策略。

- 分辨率 0.05 m；距离为栅格欧氏距离，减去半格对角线作保守修正。
- 取地面以上 0.18–1.65 m 的水平障碍高度切片；已知地面语义剔除。Flycar 地面设为 -0.15 m，Go2-W 为 -0.27 m，均为本次分析设置。复杂坡面、上下层应改为局部地面和 3D 可通行性模型。
- 基础尺寸半径 `R=0.35 m`，最小余量 `0.08 m`，舒适余量最多 `0.30 m`。
- 反应时间 `0.25 s`，减速度 `1 m/s²`。尺寸和动力学均为示例参数，未实机标定。
- `hard = R + 0.08 + v×0.25 + v²/(2×1)`。
- 双侧宽度有效时，`radius = hard + clip(width/2 − 0.05 − hard, 0, 0.30)`；缺少双侧观测时保留最大舒适余量，并明确“观测不足”。
- 宽度 `< 2×hard` 时标记当前参数下宽度不足，决不减小硬下限。这个局部几何判断不保证路径可行。
- 固定对照半径是 0.73 m，即零速尺寸、最小余量、最大舒适余量之和；并非把 Flycar 的 0.1 m 直接当作机器狗尺寸。
- 行人类额外增加 0.45 m；逻辑经单元测试，本次全局静态地图中无行人点，页面未伪造行人。
- 地面/楼梯/斜坡语义来自已有模型记录，未重新推理、也未验证分类精度。
- 无回波格保留未知。网页“有回波 XY 格”不是可通行自由空间；RViz OccupancyGrid 未知=-1，膨胀占据=100，没有伪造自由格。

## 数据

- Flycar：`/home/unitree/flycar/src/localization/fast_lio/PCD/ch31-2.pcd`，原始 1,863,997 点，显示 331,547 个 2.5 cm 体素质心。5 cm 分析数据另做半径邻域孤立点过滤，具体数量见 datasets.json。
- Go2-W：`/home/unitree/Go2w+cosmos3Nav/wave-go-Wavegov1/demos/Wavegov1/evidence/semantic_v1/semantic_map.pcd`，153,530 点，完整显示已有存档点；没有把原 12 cm 归档凭空增密。
- 扫描过另外三份 Flycar PCD：`scans.pcd` 范围达到千米级，`ch31-3.pcd` 出现数百米延伸，和这次局部场景用途不符，未选用、未修改。`ch31-1.pcd` 也保留不动。
- 两个数据集坐标系独立，未配准混合。

## 运行

```bash
cd /home/unitree/semantic_map_preview/adaptive_inflation
# 首次生成或重新处理源文件
/usr/bin/python3 prepare.py
# 网页服务（本机端口 8877）
/usr/bin/python3 server.py
```

独立终端启动 RViz 发布桥及显示；两者同用域 75，隔离现有导航：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=75
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/usr/bin/python3 /home/unitree/semantic_map_preview/adaptive_inflation/rviz_bridge.py
```

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=75
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
rviz2 -d /home/unitree/semantic_map_preview/adaptive_inflation/adaptive.rviz
```

话题前缀 `/wavego/adaptive_preview/`：points、inflated、buffer_cells、markers、status。帧名 `adaptive_preview` 是离线显示坐标，不能冒充实时 lio_map。没有速度命令、导航目标或在线代价地图写入。

批量应用到任意 PCD：

```bash
/usr/bin/python3 analyze_pcd.py /绝对路径/输入.pcd --floor=-0.27 --position 13.33826 2.42028 --yaw 3.14159 --speed .2 --output /新的结果目录
```

批量输出 `analysis.json`、`costmap.npz`、`derived_inflation_projection.pcd`。最后一个文件仅包含计算出的膨胀栅格投影，绝不能作为原始传感器数据。

## 验证与结果

- `test_adaptive.py`：7 项测试通过，覆盖窄通道尺寸下限、速度制动、单侧无效测宽、双侧测宽、负 z、未知格、行人额外余量及 PCD 格式/截断。
- `integration_checks.json`：2 数据集 × 2 位置 × 2 速度的真实 PCD 参数检查。
- Go2-W 零速：1.35 m 宽对应 0.625 m 总半径；3.10 m 宽对应 0.73 m。0.8 m/s 时分别为 0.95 m（窄通道宽度不足）、1.25 m。
- `ros_verification.json`：订阅确认 153530 点云、528198 栅格、标记；自由格为 0，未知未被误写为自由。
- `datasets.json`：源文件 SHA256、数据量、参数和观察位置。
- `current_costmap.npz` / `current_state.json`：最新网页计算结果；NPZ 用原子替换，含一致的参数元数据供 RViz 读取。
- `export_go2w/`：实际运行批量接口的 Go2-W 结果。
- `flycar_visualization.png`、`go2w_visualization.png`、`rviz_visualization.png`：实际渲染截图。

原始点云未修改；此工作是离线点云分析与可视化接口，不是已经完成实机导航验证的控制策略。
