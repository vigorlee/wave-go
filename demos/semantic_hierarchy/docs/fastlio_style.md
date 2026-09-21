# FAST-LIO 风格与三组地图表示对比

按 2026-09-19 最新要求：默认采用本机 FAST-LIO 的黑底、RViz `AxisColor`、Z 高度着色；不再默认展示 Utonia 多彩特征，不录制视频。额外提供白底低饱和蓝金风格。

参考的是本机 `/home/unitree/flycar/demos/fastlio_office/office.rviz` 和 V2 `demos/WavegoV2/config/fastlio.rviz` 的显示设置。本次只借鉴显示风格，没有重跑或替换 FAST-LIO 里程计。为保证截图间颜色可比，自动高度范围改成固定 −1.5～8.2 m。

## 运行命令

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy

# 默认：FAST-LIO 黑底、高度色纯点云
bash scripts/three_view_demo.sh --style fastlio --mode raw
```

关闭上一个演示窗口后再运行其他组合：

```bash
# 白底、低饱和蓝金高度色，适合论文排版
bash scripts/three_view_demo.sh --style paper --mode raw

# 语义类别颜色 + 物体 Box
bash scripts/three_view_demo.sh --style paper --mode semantic

# 场景 Graph
bash scripts/three_view_demo.sh --style paper --mode graph

# 同一地图融合 Graph、语义和 Box
bash scripts/three_view_demo.sh --style paper --mode combined
```

顶部直接切换表示；鼠标旋转/缩放，底部选择总览、侧视、俯视。时间轴回放已录制的扫描累积。高度色是几何量，语义色是类别，两者分别显示。原模型类别和点云坐标未改变。

独立 ROS 1 master 默认 `http://127.0.0.1:11324`，使用 `/home/unitree/flycar/tools/setup_noetic.bash`。这是已有 OfficeWorld 地图的后处理演示，不发送机器人运动指令。没有实时新传感器输入；最终语义、Graph 叠加不能当作在线语义推理。

## 对比图片

目录：`outputs/fastlio_comparison/`。每种配色都有三组：

| 文件 | 视角和内容 |
|---|---|
| `height_styles_comparison.png` | 黑底 FAST-LIO / 白底蓝金，同图同视角 |
| `fastlio_overview_comparison.png` / `paper_overview_comparison.png` | 总览：纯点云、语义 Box、Graph |
| `fastlio_side_comparison.png` / `paper_side_comparison.png` | 跨层侧视：纯点云、语义 Box、Graph |
| `fastlio_top_comparison.png` / `paper_top_comparison.png` | 俯视：纯点云、语义 Box、Graph |
| `fastlio/` 与 `paper/` 下的独立 PNG | 每种风格 9 张带面板截图、9 张纯画布截图 |
| `validation.json` | 相机、时间、点数和图层对应核对 |

所有结果都是同一份 1,647,372 点记录，Graph 为 28 节点 / 28 关系，Box 为历史预测类别簇。这些是地图表示和显示风格的对比，不是不同 SLAM 算法的精度实验；未检测到的行人不补造标签。

## 只截图、不录屏

先关闭当前演示窗口：

```bash
bash scripts/three_view_demo.sh --style fastlio --capture-compare
bash scripts/three_view_demo.sh --style paper --capture-compare
env -u PYTHONPATH -u LD_LIBRARY_PATH \
  .runtime/venv/bin/python scripts/package_fastlio_comparisons.py
```

自动截图期间固定相机并暂时忽略鼠标，以确保对应图片视角一致；普通启动可自由操作。旧的 Utonia 推理结果及此前图片仍保留；如研究需要，可显式用 `--show-features --mode utonia` 查看特征实验，特征颜色不代表语义类别。
