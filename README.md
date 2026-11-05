<div align="center">

# WAVE-Go

**Go2-W 连续导航 · OfficeWorld · 彩色三维点云 · 动态避障**

MuJoCo × Unreal Engine · ROS 2 Humble · MID-360 + IMU

[运行指南](demos/office_mapping_20261002/docs/LOCAL_MACHINE.md) · [最新源码](demos/office_mapping_20261002/runtime/) · [验收记录](demos/office_mapping_20261002/evidence/2026-10-12/) · [版本说明](demos/office_mapping_20261002/README.md)

</div>

![Go2-W 到达二楼门前：MuJoCo、RViz 彩色点云、视觉模型和 OfficeWorld](demos/office_mapping_20261002/evidence/2026-10-12/screenshots/office_four_windows_20261012_211514_159.png)

<p align="center"><sub>2026-10-12 实际运行截图：二楼门前停靠，四窗口同步展示。房号识别仍待确认；图片不代表 SLAM 精度已通过验证。</sub></p>

## 2026-11-05 · 同步本机项目

更新了原有 OfficeWorld 源码覆盖包，并将此前建图分支的内容同步到主分支。保留历次提交、Wavegov1、V2 与旧建图资料。此次为源码备份和发布检查，没有重新启动机器人。

| 本次包含 | 内容 |
|---|---|
| 楼梯入口避障 | 调整远侧转向区域、平地控制参数与零速度平衡；加入受实时点云检查约束的短距离后退恢复 |
| 实时建图 | 原生 LIO 数据处理修复、彩色稠密点云、测得地面支撑的路线与膨胀显示 |
| 连续导航 | Smac A* / NavFn Dijkstra 入口、两名动态行人、上下楼梯与坡道、二楼门前目标 |
| 实验操作 | 四窗口启动、四分屏录制、独立截图、到达后 WASD 接管 |
| 可追溯内容 | 运行源码、参数、场景 XML、测试、源文件 SHA-256 与原始验收 JSON |

## 最近一次完整导航记录

| 2026-10-12 · Smac A* | 结果 |
|---|---:|
| 从起点连续完成 | **15 / 15** |
| 导航耗时 | **376.301 s** |
| 门前目标距离 | **0.03667 m** |
| 最终航向误差 | **4.842°** |
| 本次发布重新执行的回归检查 | **86 / 86** |

原始事件顺序为 0–14，任务中没有位姿重置，也没有将续测拼接成一次全程。新增后退恢复在这轮完整运行中没有触发；该结果验证的是入口转向、控制参数与站稳策略。导航使用仿真里程计，**不等于原生 LIO 精度验证或房号识别成功**。[完整记录与限制](demos/office_mapping_20261002/runtime/docs/LOCAL_ESCAPE_20261012.md)

## 下载与运行

~~~bash
git clone https://github.com/vigorlee/wave-go.git
cd wave-go/demos/office_mapping_20261002
python3 install_snapshot.py
~~~

已有本机 MATRiX / RoamerX 运行环境时：

~~~bash
cd /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916
unset OFFICE_PREPARE_GATE OFFICE_START_STAGE INLINE_START_X INLINE_START_Y INLINE_START_YAW
OFFICE_LIO_EXECUTABLE="$PWD/.run/go2w/native_lio_build/mapping" OFFICE_RECORD=1 bash scripts/start_office_global_auto.sh SmacAStar
~~~

新电脑需先准备场景包、模型权重及 ROS 工作空间；GitHub 提供源码覆盖包，不捆绑这些大型运行资源。[恢复、编译、录制与手动控制](demos/office_mapping_20261002/docs/LOCAL_MACHINE.md)

## 其他入口与历史资料

| 入口 | 用途 |
|---|---|
| [OfficeWorld 当前版本](demos/office_mapping_20261002/README.md) | 本次更新的导航、建图与四窗口演示 |
| [保存地图回放](demos/office_mapping_20261002/scripts/replay_map.sh) | 查看仓库已有的 10 月 1 日静态点云，独立于当前在线任务 |
| [FAST-LIO / 自适应体素 V2](demos/WavegoV2/README.md) | 独立建图与栅格显示 |
| [中文语义与 Graph](demos/semantic_hierarchy/README.md) | 此前发布的语义层级查看器 |
| [Wavegov1](demos/Wavegov1/README.md) | 历史稳定版本 |
| [10 月 2 日建图首页](README_MAPPING_20261002.md) | 历史图像、参数与两种规划器的旧运行记录 |

原始视频保留在本机，仓库附源路径、SHA-256、解码检查与截图。PCD、rosbag、虚拟环境、编译缓存和其他研究目录没有加入本次源码更新。
