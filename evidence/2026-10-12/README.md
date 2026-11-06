# 2026-10-12 完整导航证据

源运行：/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/artifacts/office_auto_20261012_210928。

本目录的截图、JSON、事件流和录制清单均按原文件复制，没有修改源结果。源视频保留本机，来源与校验和在 video_source.json 中；本次没有上传 MP4。

- course/result.json：从 stage 0 开始，15/15 succeeded。
- course/events.jsonl：实际事件顺序、位置与等待行为。
- local_escape_navigation_validation.json：376.301 s，目标距离 0.03667 m，航向误差 4.842°。
- tested_source_sha256.json：当时最终导航核心源码摘要，本次逐文件核验一致。
- actual_bridge_parameters.json：实际进程采用 GO2W_RL_IDLE_STAND=0。
- door_identity_handoff.json：门前导航通过，房号核验没有通过。
- recording_decode_validation.json：当时原 MP4 全片解码检查。
- screenshots/office_four_windows_20261012_211514_159.png：二楼门前实际四窗口原图。

这次完整成功没有触发新增短距后退恢复，不作为所有恢复动作的实景验证。导航成功不验证地图绝对精度；最小候选中心距离也不是人体表面碰撞间距。
