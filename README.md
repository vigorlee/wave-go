# Go2-W OfficeWorld 完整 Demo · 2026-11-06

专用分支：**officeworld-go2w-complete-demo-20261106**。
工程直接位于仓库根目录，完整保存下面这条命令对应的自动导航工作流程：
楼梯与坡道 → 动态行人避障 → 原生楼梯上二楼 → 门前停靠 → WASD 接管；
同步打开 MuJoCo 红色布局、RViz 彩色点云、视觉模型、UE 场景四个窗口，并录制四分屏。

## 在这台电脑下载并使用

~~~bash
git clone --single-branch --branch officeworld-go2w-complete-demo-20261106 \
  https://github.com/vigorlee/wave-go.git wave-go-officeworld-complete
cd wave-go-officeworld-complete

# 一次性接入本机已安装的 OfficeWorld、ROS 工作空间、视觉模型与环境
/usr/bin/python3 scripts/prepare_complete_demo.py

# 只读检查文件校验和及运行依赖
source /opt/ros/humble/setup.bash
/usr/bin/python3 scripts/prepare_complete_demo.py --check

# 完整启动，命令和原工作工程一致
unset OFFICE_PREPARE_GATE OFFICE_START_STAGE INLINE_START_X INLINE_START_Y INLINE_START_YAW
OFFICE_LIO_EXECUTABLE="$PWD/.run/go2w/native_lio_build/mapping" \
OFFICE_RECORD=1 \
bash scripts/start_office_global_auto.sh SmacAStar
~~~

如果继续使用原工程，直接进入
/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916 后执行最后四行即可。
避免同时启动两份工程，它们使用相同的 systemd 服务名和 ROS Domain 89。

## 这次补齐的内容

- 根目录 scripts/config/controllers/tests/docs、两侧机器人场景 XML、15 阶段门前路线。
- 原生 LIO 完整包源码及 RoamerX 工作空间源码；MATRiX 运行辅助脚本。
- 原命令指定的 .run/go2w/native_lio_build/mapping 修复版可执行文件及构建记录。
- install/bin 中 Go2-W 控制桥、DreamWaQ 紧凑策略文件；保留控制桥源码。
- 四窗口、四分屏录制、截图、到达后手动接管和门牌处理代码。
- 2026-10-12 全程 15/15 的原始结果、事件流、截图与校验值。
- complete_demo_manifest.json 文件清单和 dependency 说明，可核验实际下载的文件。

## 外部运行资源

MATRiX OfficeWorld UE 场景安装包、系统 ROS/CUDA/Torch 动态库、视觉模型权重和 Python
虚拟环境需已安装。它们由准备脚本接入本机现有运行环境，**未打包进 Git**。
因此本分支是完整 Demo 工程及本机可用运行文件，不是脱离系统依赖的离线镜像。
换电脑先准备对应 MATRiX/RoamerX 环境，再执行：

~~~bash
/usr/bin/python3 scripts/prepare_complete_demo.py --source /path/to/prepared-runtime
~~~

准备脚本仅填入缺失依赖链接，不覆盖仓库源码，不启动服务。依赖来源写入检查输出。
预编译文件针对本机 Ubuntu 22.04 / x86_64 环境；其他电脑若动态库不同需按源码重新编译，
参见 scripts/build_demo.sh 和 scripts/build_office_native_lio.py。
视觉模型来源及固定版本保存在 models/anything/manifest.json。
旧上游许可证与说明按原文件保留；策略文件沿用本机公开预训练版本，发布使用需遵循其上游条款。

## 截图、接管和输出

~~~bash
/usr/bin/python3 scripts/capture_office_four_windows.py
bash scripts/open_office_manual_here.sh
~~~

接管会停止自动导航并保留位姿、地图和四窗口。点击面板接管手动后使用 WASD；
Space/Esc 停止锁定，R 是仿真复位。输出按日期保存于 artifacts/office_auto_YYYYMMDD_HHMMSS，
四分屏视频在该轮 recordings 下。默认不录制原始 PCD。

## 验证范围

最近完整实测为 2026-10-12 Smac A*：15/15，376.301 秒，门前目标距离 0.03667 m。
本次发布执行依赖检查和回归测试，未重新启动仿真。新增后退动作在该完整运行中没有触发。
房号识别本轮未确认，原生 LIO 上楼精度仍需独立验证；3 cm 是采样分辨率。
详见 docs/LOCAL_ESCAPE_20261012.md 和 evidence/2026-10-12/。
