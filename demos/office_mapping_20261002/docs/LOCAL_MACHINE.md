# 本机启动与恢复

本机有两套独立环境：实时 OfficeWorld 使用 **ROS 2 Humble**；9 月中文 Box / Graph 单窗口使用 **ROS 1 Noetic**。每组命令在新终端执行，避免混合环境。

## 1. 直接查看随分支保存的地图

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/office_mapping_20261002
python3 install_snapshot.py
bash scripts/replay_map.sh --check
bash scripts/replay_map.sh --side
```

ROS Domain 为 137，固定坐标 `lio_map`。关闭 RViz2 后，启动器结束自己的发布进程。地图和语义来自保存的数据，不产生新扫描。完整高度、单色几何、剖切和抽样位姿图可在左侧开关；“当前扫描”和“原生 ikd-Tree”需要实际在线话题，静态回放入口不伪造这些输入。

## 2. 之前的中文单窗口

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy
bash scripts/show_fresh_semantics.sh
```

中文 Box 与 Graph、纯点云、融合叠加在同一窗口内切换；启动时自动编译 Qt/Ogre 中文标签组件。原校正地图已在本机 `outputs/fresh_semantic_20260919/`，没有把它当作 10 月 1 日记录。换电脑需要另外复制该目录；本分支附带的是 [9 月中文截图](../evidence/semantic_20260923/08_chinese_boxes_no_question.png)和代码。

## 3. 在已有运行工程上恢复本快照

项目：`/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916`。

```bash
cd /home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/office_mapping_20261002
python3 install_snapshot.py \
  --runtime /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916
```

默认只核验。若确实需要把运行工程恢复为分支版本，先结束当前仿真，再给同一命令增加 `--apply`。它逐文件备份到目标 `.backups/mapping_20261002_*`，不复制或删除已有模型、权重和传感器记录，也不会自动启动机器人。

覆盖包依赖已准备的 MATRiX OfficeWorld 资源、RoamerX ROS 2 工作空间、Go2-W 网格与 DreamWaQ 检查点、`.venv-anything` 及三种视觉模型权重。上游版本和外部依赖详见 [来源说明](SOURCE_AND_LIMITATIONS.md)。

源码恢复后重新编译：

```bash
cd /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916

# 本机已存在 CMake 构建目录时，可复用其编译/链接配置
/usr/bin/python3 scripts/build_inline_bridge.py

source /opt/ros/humble/setup.bash
source genisom_roamerx_open/install/setup.bash
cd genisom_roamerx_open
colcon build --packages-select robot_slam --cmake-args -DCMAKE_BUILD_TYPE=Release
```

全新准备的运行工程没有 `build/go2w_rl_bridge` 时，用 `bash scripts/build_demo.sh` 完整配置、编译控制桥和 ROS 工作空间；设置实际 `TORCH_CMAKE_PREFIX` 与 `CUDACXX`，不要依赖本机绝对路径。源码包本身不包含这些外部构建依赖。

## 4. 连续导航与四分屏录制

这组命令会启动仿真并在录制命令中执行运动。规划坐标已包含在覆盖包；首次安装后可直接使用。

```bash
cd /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916

# 含日期且每轮唯一的运行名，避免启动器自动追加日期后路径不一致
scene_take="dijkstra_$(date +%Y%m%d_%H%M%S)"
bash scripts/start_office_global_demo.sh "$scene_take"
bash scripts/record_office_global_take.sh "artifacts/$scene_take" NavFnDijkstra

# 第二轮重新初始化相同场景
scene_take="astar_$(date +%Y%m%d_%H%M%S)"
bash scripts/start_office_global_demo.sh "$scene_take"
bash scripts/record_office_global_take.sh "artifacts/$scene_take" SmacAStar
```

完整仿真使用 Domain 89 / `rmw_zenoh_cpp`。启动会初始化指定的仿真服务；录制显式转交手动控制权给路线执行器。判断成功应检查本轮 `course/result.json` 的 `state` 与 12 个完成阶段，并复核录像。新的一轮不能沿用旧验证报告。

每轮输出 `go2w_four_screen_demo.mp4`、`course/`、`capture/`、语义结果和配置摘要。**当前默认不保存原始 float32，也不自动重建历史 PCD。** `scripts/office_live/export_dense.py` 是旧版原始扫描导出器；没有显式录下 `registered_xyzi.float32` 时不要直接运行它。

重新求解 Nav2 路线（准备好同一仿真运行环境后）：

```bash
bash scripts/global_compare/start_planners.sh
source /opt/ros/humble/setup.bash
source genisom_roamerx_open/install/setup.bash
export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp
/usr/bin/python3 scripts/global_compare/maps.py
/usr/bin/python3 scripts/global_compare/plan_routes.py \
  --output artifacts/global_compare_20261001/plans
```

停止本次仿真及其显示服务：

```bash
systemctl --user stop go2w-course.service go2w-office-record.service \
  go2w-global-display.service go2w-live-dense-capture.service \
  go2w-live-dense-mapping.service go2w-semantic-map.service \
  go2w-office-physics-view.service go2w-anything-view.service \
  go2w-anything-models.service go2w-global-planner.service
bash scripts/stop_go2w_systemd.sh
```

## 5. 旧 V2

独立 FAST-LIO 和密度自适应三维栅格的完整命令仍见 [V2 本机说明](../../WavegoV2/docs/LOCAL_MACHINE.md)。其示例 PCD、配置与版本不被本快照替换。
