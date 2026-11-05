# 本机启动、恢复与录制（2026-11-05）

当前本机运行工程：/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916。
所有实时命令都从运行工程执行，不直接在 GitHub 的 runtime 源码覆盖目录启动。

## 1. 本机直接运行四窗口与完整导航

~~~bash
cd /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916
unset OFFICE_PREPARE_GATE OFFICE_START_STAGE INLINE_START_X INLINE_START_Y INLINE_START_YAW
OFFICE_LIO_EXECUTABLE="$PWD/.run/go2w/native_lio_build/mapping" OFFICE_RECORD=1 bash scripts/start_office_global_auto.sh SmacAStar
~~~

使用项目内修复版原生 LIO。若该文件不存在，先执行下一节构建。省略 OFFICE_LIO_EXECUTABLE 会运行已安装的 mapping，其版本可能较旧。

会打开 MuJoCo 红色物理布局、RViz 彩色点云、视觉模型、UE 场景四个窗口。输出按日期保存在 artifacts/office_auto_YYYYMMDD_HHMMSS；录像在本轮 recordings 下。控制端使用 ROS 2 Humble / Domain 89 / rmw_zenoh_cpp。

默认 SmacAStar 的 2026-10-12 完整记录为 15/15。也可将末尾参数改为 NavFnDijkstra；新版本的完整 15 阶段没有提供 Dijkstra 的同轮通过记录，旧 12 阶段证据不能代替它。

截图和接管当前机器人：

~~~bash
/usr/bin/python3 scripts/capture_office_four_windows.py
bash scripts/open_office_manual_here.sh
~~~

接管脚本会停止自动路线，保留位姿、地图和四窗口。点击面板接管手动后使用 WASD；Space / Esc 停止并锁定。R 是仿真复位，不是原地站起。

## 2. 从 GitHub 恢复到已有运行环境

~~~bash
git clone https://github.com/vigorlee/wave-go.git
cd wave-go/demos/office_mapping_20261002
python3 install_snapshot.py
python3 install_snapshot.py --runtime /path/to/prepared-runtime
~~~

默认只核验。确需恢复时，停止运行服务，再为最后一条命令加 --apply；差异会先保存到目标 .backups 目录。本次上传没有在运行工程执行 --apply。

需要已有 MATRiX OfficeWorld 资产、RoamerX ROS 工作空间、Go2-W 网格、DreamWaQ 权重和 .venv-anything 视觉环境。以下构建使用本机已有 CMake 配置：

~~~bash
cd /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916
source /opt/ros/humble/setup.bash
source genisom_roamerx_open/install/setup.bash
/usr/bin/python3 scripts/build_inline_bridge.py
/usr/bin/python3 scripts/build_office_native_lio.py --jobs 2
~~~

独立 LIO 构建输出 .run/go2w/native_lio_build/mapping，不替换已安装的原版可执行文件。
它需要 genisom_roamerx_open/build/robot_slam/compile_commands.json 与对应链接配置。首次准备新工作空间需先配置：

~~~bash
cd /path/to/prepared-runtime/genisom_roamerx_open
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_slam --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
~~~

控制桥首次构建见 runtime/scripts/build_demo.sh，须准备正确的 Torch、CUDA 和外部依赖路径。当前覆盖包不是包含所有资产的一键安装镜像。

## 3. 只看历史地图

在下载的 demos/office_mapping_20261002 目录执行：

~~~bash
bash scripts/replay_map.sh --check
bash scripts/replay_map.sh --side
~~~

回放使用独立 ROS Domain 137，不启动运动；数据是 10 月 1 日静态累积记录。

## 4. 停止本次仿真

~~~bash
cd /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916
systemctl --user stop go2w-auto-handoff.service go2w-live-audit.service   go2w-course.service go2w-office-record.service go2w-manual.service   go2w-global-display.service go2w-live-dense-capture.service   go2w-live-dense-mapping.service go2w-semantic-map.service   go2w-office-physics-view.service go2w-anything-view.service   go2w-anything-models.service go2w-global-planner.service   go2w-router-watchdog.service
bash scripts/stop_go2w_systemd.sh
~~~
