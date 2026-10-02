# 923 LAZ：点云、几何 Box 与 Graph

微信压缩包 `923-20260923-173628.zip` 内含一份 LAS 1.2 / 点格式 2 的 LAZ。它在本机通过 CRC 检查并解码为 1,670,795 个点，字段为 XYZ、RGB、强度等。转换后的 PCD 保存于 `outputs/imported_923_20260923/923_rgb_intensity.pcd`；没有变换原始坐标。逐点检查的 XYZ 最大浮点转换误差为 0.000000702 m，RGB 与强度完全一致。原 ZIP 未被修改。

## 打开单窗口演示

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy
bash scripts/show_imported_923.sh
```

界面包含 **RGB 点云、高度点云、几何簇 + Box、Graph、融合叠加** 五页，以及总览／俯视／侧视按钮。点云、Box、Graph 使用同一 `imported_923` 坐标系，地图控件直接嵌入 Qt 主窗口。关闭窗口会清理本次启动的发布节点与自有 ROS master；默认 master 是 `11330`，可用 `SCENE_923_ROS_MASTER` 修改。

重新截图：

```bash
bash scripts/show_imported_923.sh --capture
```

截图和每页显示状态的 JSON 写入 `outputs/imported_923_20260923/01_rgb.*` 到 `05_combined.*`。

## Graph 如何得到

本文件的 LAS `classification` 全为 0，没有 GPS 时间或 CRS 声明。因此图中 **P0 是统计得到的主水平面，G1–G6 是连通几何簇**，不是模型识别到的楼梯、墙面或行人。P0 来自 1 cm 的 Z 直方图峰，当前峰位约 1.485 m。对低于峰值 0.13 m 的点按 12 cm 三维体素做 26 邻域连通，保留点数至少 2,000 的前六个簇。Box 使用各簇 XYZ 的第 2–98 百分位范围。Graph 的边只表示节点来自同一份点云；没有推断房间、物体关系或可通行性。

保留的 `geometry_graph.json` 存有各节点位置、Box 范围、点数和处理证据；`cluster_ids.npy` 与转换 PCD 一一对应。高度页使用 Z 映射的低饱和蓝—青—金色，不把颜色当语义类别。

原始点中约 54% 位于 Z=1.4–1.5 m，Z>3 m 仅 19 点。它更适合作为当前坐标下的局部静态几何图。并入原 OfficeWorld 二层地图前，需要提供或估计两者间的刚体变换；当前没有做未经核验的配准。没有逐帧位姿、时间戳或独立 RGB 图像，也无法从此文件重放在线建图或沿用先前 DINO/SAM2 的标签。

处理脚本：`scripts/process_imported_923.py` 需要 NumPy 与 SciPy，读取现成 PCD。ROS 1 发布器与查看器分别是 `scripts/publish_imported_923.py`、`scripts/imported_923_demo.py`，使用本机 Noetic 环境。若只看离线概览图，打开 `outputs/imported_923_20260923/923_cloud_preview.png`。
