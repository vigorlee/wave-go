# Cosmos3 首尾目标图条件验证：Attempt 04

本轮验证“直接把目标图作为视觉终点条件”是否优于只把目标信息写入 prompt。

## 条件构造

`inputs/patrol_to_target_condition.mp4` 共 121 帧：

- 第 0 至 4 帧来自真实巡游历史。
- 第 5 至 120 帧重复原始 `target_image.jpg`，仅作为终点 latent 的像素来源。
- `condition_frame_indexes_vision=[0,1,29,30]`，实际硬约束首 5 帧和末 8 帧；中间 latent 由模型生成。

这使巡游视频和目标图像像素通过同一个 `vision_path` 进入模型，不再依赖纯文字记忆目标纹理。

## 输出

- [输出视频](output/go2w_image_goal_endpoint_conditioned_seed561/vision.mp4)
- [条件视频关键帧](inputs/condition_keyframes.jpg)
- [输出关键帧](keyframes.jpg)
- [输出接触表](contact_sheet.jpg)
- [切换前帧 112](frame_112.jpg)
- [终帧 120](frame_120.jpg)
- [请求](request.json)
- [机器可读验收](verification.json)

推理成功：H.264，`832 x 480`，121 帧，10 FPS，12.1 秒；墙钟时间 45.53 秒。

## 结果

直接终点条件成功恢复了目标图像身份：终帧与按模型尺寸缩放/居中裁剪的目标图 SSIM 约为 0.964060，生成结果在帧 114、118、119、120 可解码 ID 560。

但时间连续性失败。模型先沿着自身生成的近距离目标轨迹运动，到帧 112 时目标已经很大；进入第一个终点条件帧 113 时，画面突然切回原始目标图的远一些视角：

- 帧 112→113 灰度 MAD：37.781283。
- 帧 112→113 scene score：0.315262，超过 0.25 硬切阈值。
- 相邻的帧 111→112 灰度 MAD 只有 1.124171。

因此本轮准确结论是：**非连续首尾 latent 条件能保住终点目标像素和 ArUco 身份，但不能自动保证长时间轨迹与终点平滑衔接。** Attempt 04 不满足“连续未来视频”验收，只作为失败对照；当前最佳连续结果仍是 Attempt 03。

若继续研究首尾条件，需要短窗口分层生成、在每一窗口内逐步逼近目标图，或训练专门的 goal-image-conditioned navigation adapter；不能把相距较大的终点图直接钉在单个长 rollout 的末端。
