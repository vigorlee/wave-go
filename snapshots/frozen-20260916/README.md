# Frozen Go2-W baseline — 2026-09-16

用户要求在 OfficeWorld 迁移前冻结的源码快照；本分支不用于新场景开发。`runtime/` 文件从当前工作目录逐字节复制，没有修改原始源码。`files.sha256.json` 保存校验值。

保留控制器、导航/感知源码、MuJoCo/UE 场景 XML、传感器配置、ROS 源码与地图、测试，以及 9 月 15 日密集点云/自适应膨胀和录制脚本。外部膨胀实现保存在 `external/adaptive_inflation/`。

[evidence/go2w_four_screen_demo.mp4](evidence/go2w_four_screen_demo.mp4) 为交付的 96.6 秒视频；RViz 是同一次成功运行的时间戳回放，其他三路为现场实录。成功运行完成 11/11 阶段，其他重跑存在楼梯停滞或偏移保护，详见各 attempt 结果。自适应膨胀是显示分析，并未接管导航控制。

源码覆盖层需要已有 MATRiX、OfficeWorld/Venice DLC、ROS 2 Humble/Zenoh、模型权重和构建环境。大型二进制依赖与原始点云保留本地，未上传到 Git；不要把本快照误认为完整可启动磁盘镜像。

恢复时先逐文件核验 SHA256，再把 runtime 覆盖到新的准备好的工作目录，并安装 external 中的膨胀代码；脚本保留原始路径，异机需显式配置/调整路径并重新构建。不要覆盖原冻结目录。
