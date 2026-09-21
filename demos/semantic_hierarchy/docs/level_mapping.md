# 保留旧版、重新采集和楼层水平校正

2026-09-19：旧配色、语义 Box、Graph、原始 PCD 均保留。本次重新运行 OfficeWorld，完成全部 12 个路线阶段并到达二楼。没有录制视频。

## 直接查看

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy
bash scripts/show_level_comparison.sh
```

原生 RViz 窗口可以切换 **旧地图 / 新采集纯 LIO / 新采集楼层校正**，提供立体、正侧、俯视和楼梯剖面视角。鼠标拖动旋转、滚轮缩放。只读地图查看，不发送机器人运动指令。显示裁剪到场景范围，完整 PCD 包含更大范围的实际扫描点。

切回之前的显示版本：

```bash
# 先前 Batlow 高度色与语义融合
bash scripts/three_view_demo.sh --style batlow --mode combined
# FAST-LIO 风格
bash scripts/three_view_demo.sh --style fastlio --mode raw
# 之前白底蓝金风格
bash scripts/three_view_demo.sh --style paper --mode raw
# 保留的特征实验
bash scripts/three_view_demo.sh --style batlow --show-features --mode utonia
```

旧语义和 Graph 仍对应旧记录，未把旧预测直接贴到新地图上。启动另一个旧版窗口前先关闭已打开的同类旧版窗口，避免共用节点；新的楼层对比使用独立 ROS master 11326。

## 新记录和两份 PCD

所有以下路径相对于本目录：

`outputs/level_fix/fresh_20260919_02/`

| 文件 | 内容 |
|---|---|
| `structural_calibrated/office_two_floors_structural_4cm.pcd` | 水平校正后的 2,389,801 点，4 cm 体素，每格保留一个实测点 |
| `export/office_two_floors_4cm.pcd` | 同次新采集的纯 LIO 结果，1,912,334 点，未做楼层结构校正 |
| `capture/registered_xyzi.float32` | 1,514 帧、28,252,635 个原始配准返回点，保留完整密度 |
| `sensors_rosbag2/` | 新采集 LiDAR、IMU、估计里程计、RGB、深度及独立验证参考话题 |
| `course/result.json` | 完成 12/12 阶段，机器人抵达二楼 |
| `structural_calibrated/report.json` | 校正方法、先验、平面拟合、点数和校验值 |
| `artifact_validation.json` | 原文件未变、PCD 完整、逐帧刚体几何保持等检查 |
| `comparison_views/` | 三份地图在一致相机下的截图与相机参数 |
| `comparisons/side_before_after.png` | 新旧地图跨层侧视对比图 |
| `comparisons/stairs_before_after.png` | 新旧地图楼梯剖面对比图 |

PCD 为二进制 XYZ + intensity。高度色是 RViz 按 Z 实时计算，不改变强度或坐标。

## 校正的真实范围

原记录拟合得到一楼约倾斜 1.14°、二楼约 4.33°，两层法向夹角约 3.95°。问题包括累积姿态/高度漂移，并非仅仅相机观察角度。

新建了独立 `robot_slam` 副本，增加从静止加速度初始化、独立积分陀螺仪的重力方向约束；过滤大于 30 m 的异常返回，修复初始化时上一帧结束时间的赋值。原运行工程的 LIO 源码和冻结的 V2 没有被替换。新副本属于本项目的实验修改，不能称为官方 FAST-LIO2 新版本。

对旧数据回放，最终采用权重 30：估计上升 5.461 m，独立仿真验证参考上升 5.509 m，差 0.048 m。该数字只属于旧数据的回放验证，**不代表新记录也达到同样精度**。权重 100 以及单独 ICP 的诊断结果未通过高度检查，保留为实验记录，不作为交付地图。

新采集纯 LIO 地图的两层法向夹角约 1.24°，仍存在高度累积误差。因此另外输出 **楼层结构校正版**：

1. 根据加速度和角速度获得重力方向；保留 LIO 的水平位置和航向估计。
2. 在已知平地/平台阶段，从实测点中寻找地面高度观测。
3. 使用场景已有高度先验：一楼 0 m、中间平台 3.215 m、二楼 5.51 m。
4. 在楼梯、坡道的过渡阶段连续插值位姿校正。每帧扫描整体做刚体变换，不把单个点投影到平面，不补造点。

校正版拟合的一楼/二楼倾角约 **0.42° / 0.57°**，法向夹角约 **0.38°**。这是带楼层先验的结构校正效果，不是无先验 SLAM 的绝对精度评测。层高没有被压成零；楼梯和坡道仍保留高度变化。局部墙面/动态物体重影尚未完全消除。

校正器不读取仿真位置或姿态；楼层高度和路线阶段属于显式场景先验。原路线控制器使用仿真里程计驱动 RL 连续行走，这也不是纯感知无图探索的验证。相关区别均写入报告。

## 旧文件保留位置

原路径未改变，另有逐文件 SHA-256 校验备份：

`outputs/backups/20260919_195139_before_level_fix/`

`manifest.json` 列出 11 份原 PCD/数据文件的来源、备份路径和散列。显示代码及仿真配置另存于归档。旧版 `outputs/reference_demo/`、`outputs/academic_upgrade/`、`outputs/three_representations/`、`outputs/fastlio_comparison/` 均保留。

`fresh_20260919_01` 是第一次重新采集，停在二楼楼梯入口，只完成 10/12 阶段；不是最终结果。第二次将平台转向位置后移 0.30 m，留出上楼接近距离，完成整条路线。

## 再采集与导出

下面会重启本机 OfficeWorld **仿真**并自动执行路线；不会控制真机。输出名称必须是新的，避免覆盖。需要现有 MATRiX 工程和 ROS 2 Humble 环境。

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy
bash scripts/build_level_lio.sh
bash scripts/record_level_office.sh fresh_my_next_run

env -u PYTHONPATH -u LD_LIBRARY_PATH .runtime/venv/bin/python \
  scripts/export_level_map.py outputs/level_fix/fresh_my_next_run/capture

source /opt/ros/humble/setup.bash
/usr/bin/python3 scripts/extract_capture_imu.py outputs/level_fix/fresh_my_next_run

env -u PYTHONPATH -u LD_LIBRARY_PATH .runtime/venv/bin/python \
  scripts/calibrate_floor_structure.py outputs/level_fix/fresh_my_next_run
```

仅在路线成功且地面观测充分时才会输出结构校正版。换场景后必须重新提供真实楼层先验及阶段，不能沿用 OfficeWorld 的 5.51 m。
