# 节点二：Cosmos 连续未来导航视频

记录时间：2026-07-28
状态：**离线 PoC 完成；连续接近成立，真实到达未证明**

## 任务

```text
导航指令 + HouseWorld 巡游历史视频 + 目标图像信息
    -> Cosmos3-Edge video2video
    -> 连续未来第一视角视频
```

“节点一”的 Go2-W 纯世界模型盲探索保持暂停，状态和恢复条件单独记录在
[`../id_experiments/FAR-NE/NODE-1.md`](../id_experiments/FAR-NE/NODE-1.md)。本目录的推理完全离线，
没有启动 ROS 控制、发布速度命令或改变机器人状态。

## 输入融合

Cosmos3 的标准 `video2video` 请求只有一个 `vision_path`，不能同时接收巡游视频和独立目标图像两个视觉 tensor。本次采用：

1. `patrol_history.mp4` 作为 5 帧、10 FPS 的视觉条件历史。
2. 从 `target_image.jpg` 和已有 Reasoner 结果提取目标类别、ArUco 字典/ID、外观、场景关系及导航目标，保存为 `target_info.json`。
3. 将结构化目标信息写入生成 prompt。

因此，目标图信息参与了生成，但不是第二路像素条件。精确纹理和 ArUco bit pattern 不受模型硬约束。

代码层还支持非连续 latent 条件。后续若要让目标图直接约束终点，可先把巡游首 5 帧和目标图末帧合成一个 121 帧条件视频，再设置 `condition_frame_indexes_vision=[0,1,30]`：前两个 latent 固定巡游历史，最后一个 latent 固定目标图。这仍是单 `vision_path`，但能把目标像素带入终点约束。该方式尚未在本目录运行和验收，不能与当前语义融合结果混为一谈。

## 结果

| 尝试 | 输出 | 结论 |
|---|---|---|
| Attempt 01 | 2.5 秒、320x192、25 帧 | 连续接近；未到达；生成帧不能保持 ID 560 |
| Attempt 02 | 6.1 秒、832x480、61 帧 | 连续接近并居中；末段仍运动；ID 560 漂移 |
| Attempt 03 | 12.1 秒、832x480、121 帧 | 进入目标近景、基本居中、末段近静止；目标几何和 ID 漂移，安全到达未证明 |

当前最佳展示是 [Attempt 03 视频](attempt-03/output/go2w_target_video_prediction_12s_seed560/vision.mp4)，对应
[关键帧](attempt-03/keyframes.jpg)、[接触表](attempt-03/contact_sheet.jpg) 和
[机器可读验收](attempt-03/verification.json)。

## 严格结论

- **已验证**：同一主体场景内的连续未来视频、目标持续放大、水平趋中、末两秒明显减速。
- **部分验证**：末端形成近静止视觉状态，但仍有生成漂移和非零逐帧运动。
- **未验证**：真实机器人安全到达、正确停距、对位板关系、碰撞约束、可解码 ArUco ID 560。

这条长视频链路适合做目标导向视觉 rollout 和方案预览，不应直接充当闭环控制成功证据。接入机器人时仍需短时域滚动预测、真实 RGB-D/LiDAR、候选安全门控和终端 Reasoner 确认。
