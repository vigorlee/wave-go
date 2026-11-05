# OfficeWorld 连贯导航与实时传感器 Demo

## 本轮交付结果

- [四分屏连续录像](artifacts/office_live_20260916_15_final/officeworld_continuous_four_screen.mp4)：314.85 秒，1920×1080、20 fps，完整解码检查通过。
- [阶段预览](artifacts/office_live_20260916_15_final/contact_sheet.jpg)：开始、上下楼、房间导航、圆柱区、上下坡、到达终点。
- [任务与录像校验](artifacts/office_live_20260916_15_final/verification.json)：23/23、1 个 action goal、0 次中间重启。
- [3 cm 稠密 PCD](artifacts/office_live_20260916_15/capture/go2w_live_dense_3cm.pcd)：61,491,700 个原始点经去重得到 2,652,907 点，42.45 MB；全部数值有限。坐标系为本轮 LIO 的局部 `lio_map`。
- [PCD 数据报告](artifacts/office_live_20260916_15/capture/pcd_report.json)：包含范围、密度对照和 SHA256。
- 该轮配置快照：`artifacts/office_live_20260916_15/config_snapshot/`。

![阶段预览](artifacts/office_live_20260916_15_final/contact_sheet.jpg)

## 原版备份与隔离

原始 `/home/unitree/Go2w+cosmos3Nav/matrix_go2w_lcm_demo` 未修改。1,240 个源码和配置文件哈希复核一致，记录在 `artifacts/frozen_source_recheck.json`。

- GitHub：[freeze-20260916-before-office](https://github.com/vigorlee/wave-go/tree/freeze-20260916-before-office)
- 冻结提交：`2f2c3c742152f3f6e8f35d92b937228ea58c473f`
- 标签：`frozen-20260916`
- 快照包含源码、配置、既有演示和校验清单；不包含大型 UE 二进制、DLC、模型权重及全部原始采集数据。

本目录 `/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916` 是独立实验副本。运行环境和权重部分复用本机的只读依赖，因此不是可直接搬到任意电脑的独立安装包。

## 场景与展示

OfficeWorld 地图 13（DLC pakchunk23），采用 Venice 示例中的 RGB、深度、LiDAR 展示方式。“Venice”是示例场景名，不是传感器型号。

| 数据 | 配置 | 用途 |
|---|---|---|
| RGB | 960×540，5 Hz | 实时相机、模型输入 |
| 模拟深度相机 | 640×480，5 Hz | 原始深度数据 |
| Mid360 | 10 Hz，随机扫描，不绘制绿色射线 | LIO、RViz 点云、PCD |
| Depth Anything V2 | 预训练模型 | 相对深度可视化，非公制深度 |
| Grounding DINO + SAM2 | 预训练模型 | 检测和语义 mask，标签未经精度评定 |

迁移原 Yard 的 41 个几何体，保留尺寸、坡角、接触和摩擦参数，只调整位置。楼梯整体平移 (-3.5,-6.0) m；楼梯圆柱额外向左 0.45 m，为行走留出空间。圆柱区平移 (+7.2,-12.57) m，上下坡平移 (+17.2,-18.5) m。Office 原生碰撞体保持不变。

起点 (-3.2,-6.0)，经过上楼、下楼、侧通道、房间、圆柱区、办公走廊、上下坡，终点 (19,1)。23 个阶段使用同一个 Nav2 `NavigateThroughPoses` 任务。下楼后加载含台阶侧壁占用的地图，坡底转入前切换坡道模式。中间航点删除半径 0.75 m，监督器通过半径 0.70 m。终点到达、姿态、无进展退出和坡道实际升降高度仍单独检查。

这是已知场景路线导航；不宣称 Cosmos 模型在线规划或无图探索。

## 四分屏和点云来源

1. **MuJoCo 只读全局视图**：接收实际物理进程的 UDP 状态，不运行第二套控制器；仅在视图中隐藏高层，便于观察一楼。
2. **RViz**：Mid360 实际模拟射线经 LIO 配准，6 cm 体素累计显示，叠加轨迹、语义及自适应膨胀。自适应膨胀为可视化旁路，未接管导航控制。
3. **模型视图**：RGB、Depth Anything V2、Grounding DINO + SAM2、语义 mask。
4. **UE 场景视图**：同一机器人、同一轮任务的实时画面。

一次 X11 抓屏裁成四块，1920×1080、20 fps。传感器和模型各有处理延迟，不代表硬件时间同步。

导航 `/livox/lidar` 沿用原系统的圆柱模型辅助点云；LIO 和 PCD 使用独立的 `/livox/lidar_raw` 输入，不把模型补点当测量。离线 PCD 从保存的真实配准点以 3 cm 体素去重导出，不插值加点。它是稠密累计地图，可能含动态历史和 LIO 漂移，未做高精地图精度认证。

## 启动与录制

每次使用新名称，脚本拒绝覆盖已有录像：

```bash
cd /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916
bash scripts/start_office_demo.sh office_example_01
bash scripts/record_office_take.sh artifacts/office_example_01
/usr/bin/python3 scripts/office_live/export_dense.py artifacts/office_example_01/capture
# 只有完整通过的任务才能生成最终交付证据：
/usr/bin/python3 scripts/finalize_office_demo.py artifacts/office_example_01
```

启动命令暂不行走，录制命令开始连续任务；任务结束后封装视频并停止该次原始点云采集。四个窗口录制期间不要移动或缩放。

## UE 坐标与迁移验证

Office 的渲染偏移为 `UE_cm=(100*x-760, -100*y+2420, 100*z)`。构建脚本同时生成物理 XML、地图、路线和 31 个 UE 实体；10 个重复外观壳不单独生成实体。

Office 默认不启动自定义场景加载器。启动脚本先 `LoadPackage /Game/BluePrint/BP_SceneLoader`，再 `summon /Game/BluePrint/BP_SceneLoader.BP_SceneLoader_C`，并验证 UE 实际导出的全部 31 个实体的尺寸、位置、旋转。所有写入都在本副本。

```bash
/usr/bin/python3 scripts/build_go2w_office_course.py
/usr/bin/python3 -m pytest tests/test_office_migration.py -q
```

6 项检查覆盖原尺寸和接触参数、Office 原生碰撞体、静态航点净空、UE 坐标映射、楼梯侧壁占用、运行时路线协议验证。动态任务是否通过以每轮 `course/result.json` 为准。

## 实测记录

所有尝试保存在 `artifacts/office_live_20260916_*`，索引为 `artifacts/office_test_history.json`。保留失败录像，不拼接不同轮次充当连续成功。

- 01–04：场景加载、坐标偏移和实时状态对齐调试。
- 05–11：楼梯策略、侧通道宽度及错过中间航点问题；保留各轮结果。
- 12：14/22 阶段，连续通过上下楼、圆柱区、走廊；平地模式把坡面视为障碍，在坡道入口无进展退出。
- 13：路线配置的 profile/mode 组合被启动校验拒绝，未开始行走；已补充协议测试。
- 14：2/23，楼梯出口转弯余量不足导致无进展退出。
- **15：完整通过 23/23 阶段；一个 NavigateThroughPoses 任务，中间重启 0 次。** 楼梯整体向后平移 0.5 m，增加出口空间。坡道实测升高 0.508 m、降低 0.439 m，上下坡最大俯仰分别 0.162/0.175 rad。

本结果证明这一轮完整运行成功，前面存在失败尝试，不代表已完成重复可靠性验证。
