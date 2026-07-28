# Cosmos3 连续未来导航视频验证：Attempt 01

本目录验证以下离线链路：

```text
导航指令 + HouseWorld 巡游历史视频 + 目标图像语义
    -> Cosmos3-Edge video2video
    -> 连续未来导航视频
```

## 输入融合边界

Cosmos3 的标准 `video2video` schema 只有一个 `vision_path`，不能同时传入独立巡游视频和目标图。因此本次采用可复现的两阶段融合：

1. `patrol_history.mp4` 作为视觉条件前缀。
2. `target_image.jpg` 的目标类别、ArUco ID、外观和局部场景关系写入 `target_info.json`，再逐项注入 `request.json` 的 prompt。

这使三类输入都参与生成，但目标图是语义条件，不是第二路视觉 tensor。

## 固定参数

- 模型：本机 `/home/unitree/matrix_g1_lcm_demo/Cosmos3-Edge`
- 模式：`video2video`
- 历史前缀：5 帧，10 FPS，HouseWorld 低视角相机
- 输出：25 帧，10 FPS，256p，16:9
- 条件 latent 索引：`[0, 1]`
- diffusion steps：12
- guidance：4.0
- seed：560
- guardrails：关闭，仅用于离线机器人场景生成

## 结果解释

该视频是世界模型预测，不会直接发布 ROS 速度命令，也不证明机器人已在闭环中到达目标。最终是否到达必须由真实 RGB-D、LiDAR 和安全门控再次确认。

## 实测结果

- 推理状态：`success`
- 输出：`320 x 192` H.264，25 帧，10 FPS，2.5 秒，679,570 bytes
- 端到端墙钟时间：21.30 秒
- 12 步 diffusion sampling：约 9.7 秒（首步包含编译开销）
- 主机最大常驻内存：6,220,360 KiB
- `freezedetect(n=0.002,d=0.2)`：未检出冻结区间
- 视觉检查：历史前缀、白墙、深色踢脚线、花纹窗帘和白色目标外形保持连续；相机持续向前，目标缓慢变大，但 2.5 秒内没有到达并停车。
- ArUco 检查：生成帧未能稳定解码 ID 560。原始 640 x 360 历史帧中也只有 1/5 帧能稳定解码，说明 256p 尺度和生成细节不足以支持身份验证。

结论：Attempt 01 证明 Cosmos3-Edge 能从 HouseWorld 巡游前缀生成非静态、连续、朝目标靠近的未来视频，但没有证明“到达目标”或“保持可机器解码的 ID 560”。这两项留给更长、更高分辨率的 Attempt 02。
