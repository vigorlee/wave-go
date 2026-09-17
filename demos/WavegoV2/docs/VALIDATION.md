# V2 验证记录

日期：2026-09-17；Ubuntu 22.04 x86_64。

- 在独立 `WavegoV2/workspace` 从源码编译成功：3 个项目包加 catkin_tools_prebuild，共 4/4。
- 复用本机已安装 Noetic 与 SDK 依赖；没有在干净电脑重新执行整套 bootstrap，首装流程尚未单独验证。
- 新启动器使用隔离端口 12321/12322，实际启动并读到消息。
- FAST-LIO `/cloud_registered` 实测一帧 19665 点、frame=odom。
- 自适应栅格 `/map_generator/adaptive_10cm` 97605 点、20cm 56302 点、40cm 6822 点、frame=world；5cm 层另有 2 点。
- 算法对全部示例点检查唯一覆盖、体素不重叠。
- `tests/test_adaptive.py` 验证合成稠密/稀疏平面的尺度适配和大间隙保留。
- Bash 启动脚本语法与 Python 启动器语法检查通过。
- RViz 全景实际显示已检查，截图见 evidence。测试端口的无窗口启动在验证后已结束，原演示窗口保留。

JSON 话题证据：`topic_validation.json`。完整构建日志在本机工作目录的 `/tmp/wavegov2-build.log`；Git 内不提交生成的构建目录。

以上验证不代表地图精度、回环一致性或导航避障安全性评测通过。FAST-LIO 使用仿真点云的零逐点时间信息；自适应栅格仅表示实测占据表面，不补齐未观测区域。
