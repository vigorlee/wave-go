# 发布检查（2026-11-05）

在这次准备上传的 runtime 源码上重新执行了 86 项回归检查：0 失败、0 错误、0 跳过。
覆盖楼梯入口状态机、局部短距恢复、恢复停止条件及路线/实测地面显示；没有启动仿真或重建大型依赖。

- Python 语法解析：130 个文件通过。
- Bash 语法检查：60 个文件通过。
- 导航核心四个文件与 2026-10-12 最终验收时的 SHA-256 一致。
- 本次运行结果见 [regression.json](../evidence/publication_20261105/regression.json)，逐项输出见 [regression.txt](../evidence/publication_20261105/regression.txt)。
- 全部发布文件的来源和摘要见 [source_manifest.json](../source_manifest.json)，可运行 python3 install_snapshot.py 复核。

最近完整导航的 15/15 记录仍来自 2026-10-12，未将今天的静态检查作为新的全程运动验证。
本次导航没有证明原生 SLAM 精度或房号识别成功，也没有触发新增后退动作。

本机原始工程、视频和其他 Git 工作区的未提交更改保持原样。
