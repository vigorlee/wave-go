# Evidence index

**最新展示**：[2026-09-10 成功复测图片与验收摘要](2026-09-10/README.md)。本页以下文件属于 9 月 3 日历史发布证据，不作为 9 月 10 日修复版的视觉验收。

本目录只保留适合放进源码仓库的精简证据。完整视频不进入 Git 历史，下载地址见仓库根 README 的 Release 链接。

- [`result.json`](result.json)：去除机器绝对路径、时间戳和 mission UUID 后的可复核摘要。
- [`physical_continuous_contact_sheet.jpg`](physical_continuous_contact_sheet.jpg)：坡道、动态区和终点关键帧总览。
- [`physical_ramp_ue_sequence.jpg`](physical_ramp_ue_sequence.jpg)：坡前、上坡、坡顶/下坡 UE 画面序列。
- [`video_probe.txt`](video_probe.txt)：完整成功视频的编码、尺寸、帧率和帧数。
- [`runtime_env.txt`](runtime_env.txt)：成功运行使用的关键环境变量。

完整运行目录（包含 `events.jsonl`、`odom_physics.csv`、完整 MP4 和控制日志）保留在实验工作站；发布版只提交上述摘要，避免泄露机器路径并控制仓库体积。
