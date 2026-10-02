# 2026-10-02 发布校验

本次验证针对发布包、数据读取、ROS 话题与已有核心测试；没有重新驱动机器人执行两轮全程导航。

| 检查 | 结果 |
|---|---|
| 源码/数据/证据摘要 | `install_snapshot.py` 核对 381 个文件 SHA-256，通过；其中运行覆盖文件 346 个 |
| 本机恢复预检 | 对已有 OfficeWorld 工程执行只读检查，通过；没有执行 `--apply` |
| 启动脚本语法 | 54 个 shell 脚本通过 `bash -n` |
| Python 语法 | 快照运行脚本/测试及 `semantic_hierarchy/scripts` 均通过解析 |
| 点云读取 | 3,316,238 个源测点；ROI 显示 2,410,731 点；剖切 1,746,643 点 |
| 语义关联 | 19,454 点通过 5 cm 几何一致性筛选，3,191 点被拒绝；楼梯 4,070、坡道 333、圆柱 243、墙 5,238、地面 9,570 |
| ROS 2 发布/订阅 | 独立 Domain 137 上收到完整点云、语义点云与 5 个轨迹节点；两种点云坐标均为 `lio_map` |
| 原有核心测试 | 13 项通过：语义聚类/时序、检测过滤、深度投影、雷达安装坐标 |
| 截图文件 | 16 张 PNG 可解码；导航六帧与地图图像已人工查看，误截桌面未纳入 |

话题检查结果见 [topic_smoke.json](../evidence/reports/topic_smoke.json)。已有导航结果见两轮各自的 `verification.json`；它们保留 10 月 1 日日期，不作为本次测试结果。

## 重复核验

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/office_mapping_20261002
python3 install_snapshot.py
bash scripts/replay_map.sh --check
```

测试需要 ROS 2 消息与 Torch/Transformers 环境。此次复用本机已有视觉 Python，并补充系统 pytest 模块搜索路径，没有安装新依赖：

```bash
source /opt/ros/humble/setup.bash
/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/.venv-anything/bin/python - <<'PY'
import sys
sys.path.append('/usr/lib/python3/dist-packages')
import pytest
raise SystemExit(pytest.main([
    '-q',
    'runtime/tests/test_semantic_map_core.py',
    'runtime/tests/test_detection_filter.py',
    'runtime/tests/test_anything_projection.py',
    'runtime/tests/test_lidar_mount.py',
]))
PY
```

纯 ROS 系统 Python 没有 Torch，视觉虚拟环境没有单独安装 pytest，因此必须使用上面的组合或自行准备具有全部依赖的测试环境。测试不调用完整模型推理或控制机器狗。

未在本次重跑：全量 C++ 工作空间编译、两轮 UE 运动、模型重新推理、原始 bag 全回放和绝对地图精度评测。相关历史结果、版本差异和数据边界在各自说明中保留。
