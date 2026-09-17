# 这台电脑直接启动

本机源码位置：`/home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/WavegoV2`。
已经用现有 Noetic/SDK 编译了 V2 自己的工作空间，不需要重新下载环境。

每个新终端先执行：

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/WavegoV2
export WAVEGO_V2_ENV=/home/unitree/flycar/.noetic
export WAVEGO_V2_SDK_PREFIX=/home/unitree/flycar/.deps/install
export WAVEGO_V2_ANALYSIS_PYTHON=/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/.venv-anything/bin/python
```

点云建图（新端口，避免和已有演示冲突）：

```bash
bash scripts/start_fastlio.sh --port 12321 --bag /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_rosbag_20260916_01/go2w_pointcloud_odom_noetic.bag
```

自适应三维栅格全景：

```bash
bash scripts/start_grid.sh data/office_map.pcd --port 12322
```

停止：在对应终端 Ctrl+C。

重新编译：`bash scripts/build.sh`。构建只操作 V2 自己的 workspace，使用已安装的共享依赖，不替换原 flycar 的构建目录。这里的分析 Python 用已有 SciPy 环境；新电脑经 bootstrap 安装后无需该覆盖。
